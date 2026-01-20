/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/dynamics/mesh_shape.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/filesystem.hpp"
#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/resource.hpp"
#include "dart/common/uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/detail/assimp_input_resource_adaptor.hpp"
#include "dart/dynamics/mesh_material.hpp"

#include <assimp/Importer.hpp>
#include <assimp/cexport.h>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/postprocess.h>

#include <algorithm>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <cstring>

namespace dart {
namespace dynamics {

namespace {

class MemoryResource final : public common::Resource
{
public:
  explicit MemoryResource(std::string data)
    : mData(std::move(data)), mPosition(0)
  {
  }

  std::size_t getSize() override
  {
    return mData.size();
  }

  std::size_t tell() override
  {
    return mPosition;
  }

  bool seek(ptrdiff_t offset, SeekType origin) override
  {
    ptrdiff_t base = 0;
    switch (origin) {
      case SEEKTYPE_CUR:
        base = static_cast<ptrdiff_t>(mPosition);
        break;
      case SEEKTYPE_END:
        base = std::ssize(mData);
        break;
      case SEEKTYPE_SET:
        base = 0;
        break;
      default:
        return false;
    }

    const ptrdiff_t next = base + offset;
    if (next < 0)
      return false;
    if (static_cast<std::size_t>(next) > mData.size())
      return false;

    mPosition = static_cast<std::size_t>(next);
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    if (!buffer || size == 0 || count == 0)
      return 0;

    const std::size_t remaining = mData.size() - mPosition;
    const std::size_t availableCount = remaining / size;
    const std::size_t toReadCount = std::min(count, availableCount);
    const std::size_t toReadBytes = toReadCount * size;

    if (toReadBytes > 0) {
      std::memcpy(buffer, mData.data() + mPosition, toReadBytes);
      mPosition += toReadBytes;
    }

    return toReadCount;
  }

private:
  std::string mData;
  std::size_t mPosition;
};

class MemoryResourceRetriever final : public common::ResourceRetriever
{
public:
  MemoryResourceRetriever(
      std::unordered_map<std::string, std::string> data,
      common::ResourceRetrieverPtr fallback)
    : mData(std::move(data)), mFallback(std::move(fallback))
  {
  }

  bool exists(const common::Uri& uri) override
  {
    return findData(uri) != nullptr || (mFallback && mFallback->exists(uri));
  }

  common::ResourcePtr retrieve(const common::Uri& uri) override
  {
    const std::string* data = findData(uri);
    if (data)
      return std::make_shared<MemoryResource>(*data);

    if (mFallback)
      return mFallback->retrieve(uri);

    return nullptr;
  }

private:
  const std::string* findData(const common::Uri& uri) const
  {
    const auto byFull = mData.find(uri.toString());
    if (byFull != mData.end())
      return &byFull->second;

    const auto findByPath = [&](const std::string& path) -> const std::string* {
      const auto byPath = mData.find(path);
      if (byPath != mData.end())
        return &byPath->second;

      if (!path.empty() && path.front() == '/') {
        const auto byTrim = mData.find(path.substr(1));
        if (byTrim != mData.end())
          return &byTrim->second;
      }

      const auto slash = path.find_last_of("/\\");
      if (slash != std::string::npos) {
        const auto byName = mData.find(path.substr(slash + 1));
        if (byName != mData.end())
          return &byName->second;
      }

      return nullptr;
    };

    if (uri.mPath) {
      if (const std::string* data = findByPath(uri.mPath.get()))
        return data;
    }

    if (uri.mAuthority) {
      if (const std::string* data = findByPath(uri.mAuthority.get()))
        return data;
    }

    return nullptr;
  }

  std::unordered_map<std::string, std::string> mData;
  common::ResourceRetrieverPtr mFallback;
};

} // namespace

//==============================================================================
bool MeshShape::collectSubMeshRanges(
    const aiScene* scene,
    std::vector<SubMeshRange>& ranges,
    std::size_t expectedVertices,
    std::size_t expectedTriangles)
{
  ranges.clear();
  if (!scene)
    return false;

  std::size_t vertexOffset = 0;
  std::size_t triangleOffset = 0;

  for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
    const aiMesh* assimpMesh = scene->mMeshes[meshIndex];
    if (!assimpMesh) {
      continue;
    }

    std::size_t triangleCount = 0;
    for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
      const aiFace& face = assimpMesh->mFaces[i];
      if (face.mNumIndices >= 3) {
        triangleCount += static_cast<std::size_t>(face.mNumIndices - 2);
      }
    }

    SubMeshRange range;
    range.vertexOffset = vertexOffset;
    range.vertexCount = assimpMesh->mNumVertices;
    range.triangleOffset = triangleOffset;
    range.triangleCount = triangleCount;
    range.materialIndex = assimpMesh->mMaterialIndex;
    ranges.push_back(range);

    vertexOffset += assimpMesh->mNumVertices;
    triangleOffset += triangleCount;
  }

  if (expectedVertices != 0 && vertexOffset != expectedVertices) {
    ranges.clear();
    return false;
  }
  if (expectedTriangles != 0 && triangleOffset != expectedTriangles) {
    ranges.clear();
    return false;
  }

  return !ranges.empty();
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever,
    MeshOwnership ownership)
  : Shape(MESH),
    mTriMesh(nullptr),
    mPolygonMesh(nullptr),
    mCachedAiScene(nullptr),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  setMesh(mesh, ownership, uri, std::move(resourceRetriever));
  DART_SUPPRESS_DEPRECATED_END
  setScale(scale);
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<math::TriMesh<double>> mesh,
    const common::Uri& uri)
  : MeshShape(scale, std::move(mesh), uri, nullptr)
{
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<math::TriMesh<double>> mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(MESH),
    mTriMesh(std::move(mesh)),
    mPolygonMesh(nullptr),
    mCachedAiScene(nullptr),
    mMeshUri(uri),
    mMeshPath(),
    mResourceRetriever(std::move(resourceRetriever)),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  if (uri.mScheme.get_value_or("file") == "file" && uri.mPath) {
    mMeshPath = uri.getFilesystemPath();
  } else if (mResourceRetriever) {
    DART_SUPPRESS_DEPRECATED_BEGIN
    mMeshPath = mResourceRetriever->getFilePath(uri);
    DART_SUPPRESS_DEPRECATED_END
  } else {
    mMeshPath.clear();
  }

  mMaterials.clear();

  const std::string uriString = uri.toString();
  if (!uriString.empty()) {
    common::ResourceRetrieverPtr materialRetriever = mResourceRetriever;
    if (!materialRetriever && uri.mScheme.get_value_or("file") == "file"
        && uri.mPath) {
      materialRetriever = std::make_shared<common::LocalResourceRetriever>();
    }

    if (materialRetriever) {
      struct AiSceneReleaser
      {
        void operator()(const aiScene* scene) const
        {
          if (scene)
            aiReleaseImport(const_cast<aiScene*>(scene));
        }
      };

      DART_SUPPRESS_DEPRECATED_BEGIN
      const std::unique_ptr<const aiScene, AiSceneReleaser> scene(
          loadMesh(uri, materialRetriever));
      DART_SUPPRESS_DEPRECATED_END

      if (scene) {
        extractMaterialsFromScene(
            scene.get(),
            mMeshPath.empty() ? uri.getPath() : mMeshPath,
            mMeshUri);

        const auto expectedVertices
            = mTriMesh ? mTriMesh->getVertices().size() : 0;
        const auto expectedTriangles
            = mTriMesh ? mTriMesh->getTriangles().size() : 0;
        collectSubMeshRanges(
            scene.get(), mSubMeshRanges, expectedVertices, expectedTriangles);
        extractTextureCoordsFromScene(scene.get());
      }
    }
  }

  setScale(scale);
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<const aiScene> mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(MESH),
    mTriMesh(nullptr),
    mPolygonMesh(nullptr),
    mCachedAiScene(nullptr),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  setMesh(std::move(mesh), uri, std::move(resourceRetriever));
  DART_SUPPRESS_DEPRECATED_END
  setScale(scale);
}

//==============================================================================
MeshShape::~MeshShape()
{
  // Clean up cached aiScene if it was created.
  if (mCachedAiScene) {
    aiReleaseImport(mCachedAiScene);
    mCachedAiScene = nullptr;
  }

  releaseMesh();
}

//==============================================================================
void MeshShape::releaseMesh()
{
  mMesh.reset();
  mPolygonMesh.reset();
}

//==============================================================================
std::shared_ptr<const aiScene> MeshShape::makeMeshHandle(
    const aiScene* mesh, MeshOwnership ownership)
{
  if (!mesh)
    return nullptr;

  switch (ownership) {
    case MeshOwnership::Imported:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene* scene) {
        aiReleaseImport(const_cast<aiScene*>(scene));
      });
    case MeshOwnership::Copied:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene* scene) {
        aiFreeScene(const_cast<aiScene*>(scene));
      });
    case MeshOwnership::Manual:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene* scene) {
        delete const_cast<aiScene*>(scene);
      });
    case MeshOwnership::Custom:
    case MeshOwnership::None:
    default:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene*) {});
  }
}

//==============================================================================
MeshShape::MeshHandle& MeshShape::MeshHandle::operator=(const aiScene* mesh)
{
  // Backward compatibility: historically MeshShape exposed a std::shared_ptr
  // member that derived classes could assign a manually constructed aiScene*
  // into (e.g., Gazebo's CustomMeshShape). In that case the default deleter
  // would invoke `delete`, so we preserve that behavior here.
  set(mesh, MeshOwnership::Manual);
  return *this;
}

//==============================================================================
MeshShape::MeshHandle& MeshShape::MeshHandle::operator=(
    std::shared_ptr<const aiScene> mesh)
{
  set(std::move(mesh));
  return *this;
}

//==============================================================================
const aiScene* MeshShape::MeshHandle::get() const
{
  return mMesh.get();
}

//==============================================================================
const aiScene* MeshShape::MeshHandle::operator->() const
{
  return mMesh.get();
}

//==============================================================================
MeshShape::MeshHandle::operator bool() const
{
  return static_cast<bool>(mMesh);
}

//==============================================================================
void MeshShape::MeshHandle::reset()
{
  mMesh.reset();
  mMeshOwnership = MeshOwnership::None;
}

//==============================================================================
MeshShape::MeshOwnership MeshShape::MeshHandle::getOwnership() const
{
  return mMeshOwnership;
}

//==============================================================================
const std::shared_ptr<const aiScene>& MeshShape::MeshHandle::getShared() const
{
  return mMesh;
}

//==============================================================================
void MeshShape::MeshHandle::set(const aiScene* mesh, MeshOwnership ownership)
{
  mMesh = MeshShape::makeMeshHandle(mesh, ownership);
  mMeshOwnership = mesh ? ownership : MeshOwnership::None;
}

//==============================================================================
void MeshShape::MeshHandle::set(std::shared_ptr<const aiScene> mesh)
{
  mMesh = std::move(mesh);
  mMeshOwnership = mMesh ? MeshOwnership::Custom : MeshOwnership::None;
}

//==============================================================================
std::string_view MeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view MeshShape::getStaticType()
{
  static constexpr std::string_view type = "MeshShape";
  return type;
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::getTriMesh() const
{
  // Handle backward compatibility: if a derived class bypasses setMesh() and
  // updates the underlying aiScene directly, we lazily populate the TriMesh.
  if (!mTriMesh) {
    const aiScene* scene = nullptr;
    if (mMesh) {
      scene = mMesh.get();
    } else if (mCachedAiScene) {
      scene = mCachedAiScene;
    }

    if (scene) {
      // Const cast is safe here - we're lazily populating internal cache.
      auto* self = const_cast<MeshShape*>(this);
      self->mTriMesh = convertAssimpMesh(scene, &self->mSubMeshRanges);
      self->extractTextureCoordsFromScene(scene);
    }
  }
  return mTriMesh;
}

//==============================================================================
std::shared_ptr<math::PolygonMesh<double>> MeshShape::getPolygonMesh() const
{
  if (!mPolygonMesh) {
    const aiScene* scene = nullptr;
    if (mMesh) {
      scene = mMesh.get();
    } else if (mCachedAiScene) {
      scene = mCachedAiScene;
    }

    auto* self = const_cast<MeshShape*>(this);
    if (scene) {
      self->mPolygonMesh = convertAssimpPolygonMesh(scene);
    } else if (mTriMesh) {
      self->mPolygonMesh = convertTriMeshToPolygonMesh(*mTriMesh);
    }
  }

  return mPolygonMesh;
}

//==============================================================================
const aiScene* MeshShape::getMesh() const
{
  if (mMesh) {
    return mMesh.get();
  }

  // Lazy conversion: only convert once and cache the result.
  // NOTE: This is still expensive on first call! Please use getTriMesh()
  // instead.
  if (!mCachedAiScene) {
    mCachedAiScene = convertToAssimpMesh();
  }

  return mCachedAiScene;
}

//==============================================================================
const aiScene* MeshShape::convertToAssimpMesh() const
{
  if (!mTriMesh || !mTriMesh->hasTriangles()) {
    return nullptr;
  }

  // Assimp's aiScene is not designed as a mutable in-memory structure and its
  // lifetime rules are complex (see #453). To keep getMesh() working for legacy
  // callers while avoiding cross-module deallocation hazards, we round-trip the
  // TriMesh through Assimp's importer so Assimp allocates and frees the scene.

  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10);

  const auto& vertices = mTriMesh->getVertices();
  for (const auto& vertex : vertices) {
    stream << "v " << vertex.x() << ' ' << vertex.y() << ' ' << vertex.z()
           << '\n';
  }

  const bool hasTexCoords = !mTextureCoords.empty()
                            && mTextureCoords.size() == vertices.size()
                            && mTextureCoordComponents > 0;
  if (hasTexCoords) {
    const int components = std::min(mTextureCoordComponents, 3);
    for (const auto& texCoord : mTextureCoords) {
      stream << "vt " << texCoord.x();
      if (components >= 2) {
        stream << ' ' << texCoord.y();
      }
      if (components >= 3) {
        stream << ' ' << texCoord.z();
      }
      stream << '\n';
    }
  }

  const auto& normals = mTriMesh->getVertexNormals();
  const bool hasNormals = !normals.empty() && normals.size() == vertices.size();
  if (hasNormals) {
    for (const auto& normal : normals) {
      stream << "vn " << normal.x() << ' ' << normal.y() << ' ' << normal.z()
             << '\n';
    }
  }

  const auto& triangles = mTriMesh->getTriangles();
  const bool hasSubMeshes = !mSubMeshRanges.empty() && mMaterials.size() > 1;
  bool useSubMeshes = hasSubMeshes;
  if (useSubMeshes) {
    std::size_t totalVertices = 0;
    std::size_t totalTriangles = 0;
    for (const auto& range : mSubMeshRanges) {
      if (range.vertexOffset + range.vertexCount > vertices.size()
          || range.triangleOffset + range.triangleCount > triangles.size()) {
        useSubMeshes = false;
        break;
      }
      totalVertices += range.vertexCount;
      totalTriangles += range.triangleCount;
    }
    if (useSubMeshes
        && (totalVertices != vertices.size()
            || totalTriangles != triangles.size())) {
      useSubMeshes = false;
    }
  }

  std::ostringstream mtlStream;
  std::string mtlName;
  if (useSubMeshes) {
    mtlName = "dart_mesh.mtl";
    stream << "mtllib " << mtlName << '\n';
  }

  if (useSubMeshes) {
    bool warnedIndex = false;
    for (const auto& range : mSubMeshRanges) {
      if (range.triangleCount == 0)
        continue;

      unsigned int materialIndex = range.materialIndex;
      if (materialIndex >= mMaterials.size()) {
        if (!warnedIndex) {
          DART_WARN(
              "[MeshShape::convertToAssimpMesh] Material index {} is out of "
              "range (materials: {}). Falling back to material 0.",
              materialIndex,
              mMaterials.size());
          warnedIndex = true;
        }
        materialIndex = 0;
      }

      stream << "usemtl material_" << materialIndex << '\n';
      for (std::size_t i = 0; i < range.triangleCount; ++i) {
        const auto& triangle = triangles[range.triangleOffset + i];
        const std::size_t i0 = static_cast<std::size_t>(triangle.x()) + 1;
        const std::size_t i1 = static_cast<std::size_t>(triangle.y()) + 1;
        const std::size_t i2 = static_cast<std::size_t>(triangle.z()) + 1;

        if (hasTexCoords && hasNormals) {
          stream << "f " << i0 << "/" << i0 << "/" << i0 << ' ' << i1 << "/"
                 << i1 << "/" << i1 << ' ' << i2 << "/" << i2 << "/" << i2
                 << '\n';
        } else if (hasTexCoords) {
          stream << "f " << i0 << "/" << i0 << ' ' << i1 << "/" << i1 << ' '
                 << i2 << "/" << i2 << '\n';
        } else if (hasNormals) {
          stream << "f " << i0 << "//" << i0 << ' ' << i1 << "//" << i1 << ' '
                 << i2 << "//" << i2 << '\n';
        } else {
          stream << "f " << i0 << ' ' << i1 << ' ' << i2 << '\n';
        }
      }
    }

    mtlStream.imbue(std::locale::classic());
    mtlStream << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (std::size_t index = 0; index < mMaterials.size(); ++index) {
      const auto& mat = mMaterials[index];
      mtlStream << "newmtl material_" << index << '\n';
      mtlStream << "Ka " << mat.ambient[0] << ' ' << mat.ambient[1] << ' '
                << mat.ambient[2] << '\n';
      mtlStream << "Kd " << mat.diffuse[0] << ' ' << mat.diffuse[1] << ' '
                << mat.diffuse[2] << '\n';
      mtlStream << "Ks " << mat.specular[0] << ' ' << mat.specular[1] << ' '
                << mat.specular[2] << '\n';
      mtlStream << "Ke " << mat.emissive[0] << ' ' << mat.emissive[1] << ' '
                << mat.emissive[2] << '\n';
      mtlStream << "Ns " << mat.shininess << '\n';
      mtlStream << "d " << mat.diffuse[3] << '\n';

      if (!mat.textureImagePaths.empty() && !mat.textureImagePaths[0].empty()) {
        mtlStream << "map_Kd " << mat.textureImagePaths[0] << '\n';
      }
      mtlStream << '\n';
    }
  } else {
    for (const auto& triangle : triangles) {
      const std::size_t i0 = static_cast<std::size_t>(triangle.x()) + 1;
      const std::size_t i1 = static_cast<std::size_t>(triangle.y()) + 1;
      const std::size_t i2 = static_cast<std::size_t>(triangle.z()) + 1;

      if (hasTexCoords && hasNormals) {
        stream << "f " << i0 << "/" << i0 << "/" << i0 << ' ' << i1 << "/" << i1
               << "/" << i1 << ' ' << i2 << "/" << i2 << "/" << i2 << '\n';
      } else if (hasTexCoords) {
        stream << "f " << i0 << "/" << i0 << ' ' << i1 << "/" << i1 << ' ' << i2
               << "/" << i2 << '\n';
      } else if (hasNormals) {
        stream << "f " << i0 << "//" << i0 << ' ' << i1 << "//" << i1 << ' '
               << i2 << "//" << i2 << '\n';
      } else {
        stream << "f " << i0 << ' ' << i1 << ' ' << i2 << '\n';
      }
    }
  }

  const std::string obj = stream.str();
  if (obj.empty()) {
    return nullptr;
  }

  const unsigned int flags = aiProcess_Triangulate
                             | aiProcess_JoinIdenticalVertices
                             | aiProcess_SortByPType | aiProcess_OptimizeMeshes
                             | aiProcess_GenNormals;

  if (!useSubMeshes) {
    const aiScene* scene = aiImportFileFromMemory(
        obj.data(), static_cast<unsigned int>(obj.size()), flags, "obj");

    if (!scene) {
      DART_WARN(
          "[MeshShape::convertToAssimpMesh] Failed to import TriMesh via "
          "Assimp: {}",
          aiGetErrorString());
      return nullptr;
    }

    return scene;
  }

  const std::string objName = "dart_mesh.obj";
  std::unordered_map<std::string, std::string> data;
  data.emplace(objName, obj);
  data.emplace(mtlName, mtlStream.str());

  auto retriever = std::make_shared<MemoryResourceRetriever>(
      std::move(data), mResourceRetriever);

  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = createFileIO(&systemIO);

  const aiScene* scene = aiImportFileExWithProperties(
      objName.c_str(), flags, &fileIO, propertyStore);

  aiReleasePropertyStore(propertyStore);

  if (!scene) {
    DART_WARN(
        "[MeshShape::convertToAssimpMesh] Failed to import TriMesh with "
        "materials via Assimp: {}",
        aiGetErrorString());
    return nullptr;
  }

  return scene;
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::convertAssimpMesh(
    const aiScene* scene)
{
  return convertAssimpMesh(scene, nullptr);
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::convertAssimpMesh(
    const aiScene* scene, std::vector<SubMeshRange>* subMeshes)
{
  if (!scene) {
    return nullptr;
  }

  // ALWAYS merge all meshes into a single TriMesh
  // This is necessary because:
  // 1. Files with multiple materials (common in COLLADA/OBJ) have multiple
  // aiMesh objects
  // 2. All meshes in a scene belong to the same shape for collision/rendering
  // 3. CustomMeshShape from gz-physics creates one aiMesh per submesh
  auto polygonMesh = convertAssimpPolygonMesh(scene);
  if (!polygonMesh) {
    return nullptr;
  }

  auto triMesh
      = std::make_shared<math::TriMesh<double>>(polygonMesh->triangulate());

  if (subMeshes) {
    subMeshes->clear();
    std::size_t vertexOffset = 0;
    std::size_t triangleOffset = 0;
    for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes;
         ++meshIndex) {
      const aiMesh* assimpMesh = scene->mMeshes[meshIndex];
      if (!assimpMesh) {
        continue;
      }

      std::size_t triangleCount = 0;
      for (auto faceIndex = 0u; faceIndex < assimpMesh->mNumFaces;
           ++faceIndex) {
        const aiFace& face = assimpMesh->mFaces[faceIndex];
        if (face.mNumIndices >= 3u) {
          triangleCount += static_cast<std::size_t>(face.mNumIndices - 2u);
        }
      }

      SubMeshRange range;
      range.vertexOffset = vertexOffset;
      range.vertexCount = assimpMesh->mNumVertices;
      range.triangleOffset = triangleOffset;
      range.triangleCount = triangleCount;
      range.materialIndex = assimpMesh->mMaterialIndex;
      subMeshes->push_back(range);

      vertexOffset += assimpMesh->mNumVertices;
      triangleOffset += triangleCount;
    }
  }

  return triMesh;
}

//==============================================================================
std::shared_ptr<math::PolygonMesh<double>> MeshShape::convertAssimpPolygonMesh(
    const aiScene* scene)
{
  if (!scene) {
    return nullptr;
  }

  auto polygonMesh = std::make_shared<math::PolygonMesh<double>>();

  std::size_t totalVertices = 0;
  std::size_t totalFaces = 0;
  for (std::size_t i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh* assimpMesh = scene->mMeshes[i];
    if (!assimpMesh) {
      continue;
    }
    totalVertices += assimpMesh->mNumVertices;
    totalFaces += assimpMesh->mNumFaces;
  }

  polygonMesh->reserveVertices(totalVertices);
  polygonMesh->reserveFaces(totalFaces);
  polygonMesh->reserveVertexNormals(totalVertices);

  for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
    const aiMesh* assimpMesh = scene->mMeshes[meshIndex];
    if (!assimpMesh) {
      continue;
    }

    const std::size_t vertexOffset = polygonMesh->getVertices().size();

    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& vertex = assimpMesh->mVertices[i];
      polygonMesh->addVertex(
          static_cast<double>(vertex.x),
          static_cast<double>(vertex.y),
          static_cast<double>(vertex.z));
    }

    for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
      const aiFace& face = assimpMesh->mFaces[i];
      if (face.mNumIndices < 3) {
        DART_WARN(
            "[MeshShape::convertAssimpPolygonMesh] Face with fewer than 3 "
            "indices detected in mesh {}. Skipping this face.",
            meshIndex);
        continue;
      }

      math::PolygonMesh<double>::Face polygonFace;
      polygonFace.reserve(face.mNumIndices);
      for (auto index = 0u; index < face.mNumIndices; ++index) {
        polygonFace.push_back(face.mIndices[index] + vertexOffset);
      }
      polygonMesh->addFace(std::move(polygonFace));
    }

    if (assimpMesh->mNormals) {
      for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
        const aiVector3D& normal = assimpMesh->mNormals[i];
        polygonMesh->addVertexNormal(
            static_cast<double>(normal.x),
            static_cast<double>(normal.y),
            static_cast<double>(normal.z));
      }
    }
  }

  return polygonMesh;
}

//==============================================================================
std::shared_ptr<math::PolygonMesh<double>>
MeshShape::convertTriMeshToPolygonMesh(const math::TriMesh<double>& mesh)
{
  auto polygonMesh = std::make_shared<math::PolygonMesh<double>>();

  polygonMesh->reserveVertices(mesh.getVertices().size());
  polygonMesh->reserveFaces(mesh.getTriangles().size());

  polygonMesh->getVertices() = mesh.getVertices();
  if (mesh.hasVertexNormals()) {
    polygonMesh->getVertexNormals() = mesh.getVertexNormals();
  }

  for (const auto& triangle : mesh.getTriangles()) {
    polygonMesh->addFace({triangle[0], triangle[1], triangle[2]});
  }

  return polygonMesh;
}

//==============================================================================
std::string MeshShape::getMeshUri() const
{
  return mMeshUri.toString();
}

//==============================================================================
const common::Uri& MeshShape::getMeshUri2() const
{
  return mMeshUri;
}

//==============================================================================
void MeshShape::update()
{
  // Do nothing
}

//==============================================================================
const std::string& MeshShape::getMeshPath() const
{
  return mMeshPath;
}

//==============================================================================
common::ResourceRetrieverPtr MeshShape::getResourceRetriever()
{
  return mResourceRetriever;
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    const std::string& path,
    common::ResourceRetrieverPtr resourceRetriever)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  setMesh(
      mesh,
      MeshOwnership::Imported,
      common::Uri(path),
      std::move(resourceRetriever));
  DART_SUPPRESS_DEPRECATED_END
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  setMesh(mesh, MeshOwnership::Imported, uri, std::move(resourceRetriever));
  DART_SUPPRESS_DEPRECATED_END
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    MeshOwnership ownership,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  if (mesh == mMesh.get() && ownership == mMesh.getOwnership()) {
    // Nothing to do.
    return;
  }

  const bool meshIsCached = mesh && (mesh == mCachedAiScene);
  const MeshOwnership effectiveOwnership
      = meshIsCached ? MeshOwnership::Imported : ownership;

  if (meshIsCached && ownership != MeshOwnership::Imported) {
    DART_WARN(
        "[MeshShape::setMesh] Overriding MeshOwnership ({}) to Imported when "
        "adopting a cached aiScene created by getMesh().",
        static_cast<int>(ownership));
  }

  // Clear cached aiScene to prevent stale data. If the caller is adopting the
  // cached pointer, transfer ownership to mMesh without releasing it here.
  if (mCachedAiScene) {
    if (meshIsCached) {
      mCachedAiScene = nullptr;
    } else {
      aiReleaseImport(mCachedAiScene);
      mCachedAiScene = nullptr;
    }
  }

  releaseMesh();

  mMesh.set(mesh, effectiveOwnership);

  mPolygonMesh.reset();
  mTriMesh = convertAssimpMesh(mMesh.get(), &mSubMeshRanges);
  mMaterials.clear();

  if (!mTriMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    mTextureCoords.clear();
    mTextureCoordComponents = 0;
    mSubMeshRanges.clear();
    mIsBoundingBoxDirty = true;
    mIsVolumeDirty = true;
    return;
  }

  extractTextureCoordsFromScene(mMesh.get());

  mMeshUri = uri;

  if (uri.mScheme.get_value_or("file") == "file" && uri.mPath) {
    mMeshPath = uri.getFilesystemPath();
  } else if (resourceRetriever) {
    DART_SUPPRESS_DEPRECATED_BEGIN
    mMeshPath = resourceRetriever->getFilePath(uri);
    DART_SUPPRESS_DEPRECATED_END
  } else {
    mMeshPath.clear();
  }

  mResourceRetriever = std::move(resourceRetriever);

  // Extract material properties for Assimp-free rendering.
  extractMaterialsFromScene(
      mMesh.get(), mMeshPath.empty() ? uri.getPath() : mMeshPath, mMeshUri);

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
void MeshShape::setMesh(
    std::shared_ptr<const aiScene> mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  if (mesh && mesh.get() == mMesh.get()
      && mMesh.getOwnership() == MeshOwnership::Custom) {
    return;
  }

  if (mCachedAiScene) {
    if (mesh && mesh.get() == mCachedAiScene) {
      mCachedAiScene = nullptr;
    } else {
      aiReleaseImport(mCachedAiScene);
      mCachedAiScene = nullptr;
    }
  }

  releaseMesh();
  mMesh.set(std::move(mesh));

  mPolygonMesh.reset();
  mTriMesh = convertAssimpMesh(mMesh.get(), &mSubMeshRanges);
  mMaterials.clear();

  if (!mTriMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    mTextureCoords.clear();
    mTextureCoordComponents = 0;
    mSubMeshRanges.clear();
    mIsBoundingBoxDirty = true;
    mIsVolumeDirty = true;
    return;
  }

  extractTextureCoordsFromScene(mMesh.get());

  mMeshUri = uri;

  if (uri.mScheme.get_value_or("file") == "file" && uri.mPath) {
    mMeshPath = uri.getFilesystemPath();
  } else if (resourceRetriever) {
    DART_SUPPRESS_DEPRECATED_BEGIN
    mMeshPath = resourceRetriever->getFilePath(uri);
    DART_SUPPRESS_DEPRECATED_END
  } else {
    mMeshPath.clear();
  }

  mResourceRetriever = std::move(resourceRetriever);

  // Extract material properties for Assimp-free rendering.
  extractMaterialsFromScene(
      mMesh.get(), mMeshPath.empty() ? uri.getPath() : mMeshPath, mMeshUri);

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
void MeshShape::setScale(const Eigen::Vector3d& scale)
{
  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
void MeshShape::setScale(const double scale)
{
  setScale(Eigen::Vector3d::Constant(scale));
}

//==============================================================================
const Eigen::Vector3d& MeshShape::getScale() const
{
  return mScale;
}

//==============================================================================
void MeshShape::setColorMode(ColorMode mode)
{
  mColorMode = mode;
}

//==============================================================================
MeshShape::ColorMode MeshShape::getColorMode() const
{
  return mColorMode;
}

//==============================================================================
void MeshShape::setAlphaMode(MeshShape::AlphaMode mode)
{
  mAlphaMode = mode;
}

//==============================================================================
MeshShape::AlphaMode MeshShape::getAlphaMode() const
{
  return mAlphaMode;
}

//==============================================================================
void MeshShape::setColorIndex(int index)
{
  mColorIndex = index;
}

//==============================================================================
int MeshShape::getColorIndex() const
{
  return mColorIndex;
}

//==============================================================================
int MeshShape::getDisplayList() const
{
  return mDisplayList;
}

//==============================================================================
void MeshShape::setDisplayList(int index)
{
  mDisplayList = index;
}

//==============================================================================
std::span<const MeshMaterial> MeshShape::getMaterials() const
{
  return std::span<const MeshMaterial>(mMaterials);
}

//==============================================================================
std::size_t MeshShape::getNumMaterials() const
{
  return mMaterials.size();
}

//==============================================================================
const MeshMaterial* MeshShape::getMaterial(std::size_t index) const
{
  if (index < mMaterials.size()) {
    return &mMaterials[index];
  }
  return nullptr;
}

//==============================================================================
void MeshShape::extractTextureCoordsFromScene(const aiScene* scene)
{
  mTextureCoords.clear();
  mTextureCoordComponents = 0;

  if (!scene || scene->mNumMeshes == 0) {
    return;
  }

  std::size_t totalVertices = 0;
  bool hasTexCoords = false;
  unsigned int maxComponents = 0;
  for (std::size_t i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh* assimpMesh = scene->mMeshes[i];
    if (!assimpMesh) {
      continue;
    }

    totalVertices += assimpMesh->mNumVertices;
    if (assimpMesh->HasTextureCoords(0)) {
      hasTexCoords = true;
      maxComponents = std::max(maxComponents, assimpMesh->mNumUVComponents[0]);
    }
  }

  if (!hasTexCoords || totalVertices == 0) {
    return;
  }

  if (mTriMesh) {
    const auto expectedVertices = mTriMesh->getVertices().size();
    if (expectedVertices != 0 && totalVertices != expectedVertices) {
      return;
    }
  }

  mTextureCoords.reserve(totalVertices);
  for (std::size_t i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh* assimpMesh = scene->mMeshes[i];
    if (!assimpMesh) {
      continue;
    }

    const bool meshHasTexCoords = assimpMesh->HasTextureCoords(0);
    for (auto j = 0u; j < assimpMesh->mNumVertices; ++j) {
      if (meshHasTexCoords) {
        const aiVector3D& texCoord = assimpMesh->mTextureCoords[0][j];
        mTextureCoords.emplace_back(texCoord.x, texCoord.y, texCoord.z);
      } else {
        mTextureCoords.emplace_back(0.0, 0.0, 0.0);
      }
    }
  }

  mTextureCoordComponents = static_cast<int>(std::min(maxComponents, 3u));
}

//==============================================================================
void MeshShape::extractMaterialsFromScene(
    const aiScene* scene,
    const std::string& basePath,
    const common::Uri& meshUri)
{
  if (!scene || scene->mNumMaterials == 0) {
    return;
  }

  mMaterials.clear();
  mMaterials.reserve(scene->mNumMaterials);

  const common::filesystem::path meshPath = basePath;
  std::error_code meshPathEc;
  const bool meshPathExists
      = !basePath.empty() && common::filesystem::exists(meshPath, meshPathEc)
        && !meshPathEc;

  for (std::size_t i = 0; i < scene->mNumMaterials; ++i) {
    aiMaterial* aiMat = scene->mMaterials[i];
    assert(aiMat);

    MeshMaterial material;

    // Extract colors
    aiColor4D c;
    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_AMBIENT, &c) == AI_SUCCESS) {
      material.ambient = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_DIFFUSE, &c) == AI_SUCCESS) {
      material.diffuse = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_SPECULAR, &c) == AI_SUCCESS) {
      material.specular = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_EMISSIVE, &c) == AI_SUCCESS) {
      material.emissive = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    // Extract shininess
    unsigned int maxValue = 1;
    float shininess = 0.0f, strength = 1.0f;
    if (aiGetMaterialFloatArray(
            aiMat, AI_MATKEY_SHININESS, &shininess, &maxValue)
        == AI_SUCCESS) {
      maxValue = 1;
      if (aiGetMaterialFloatArray(
              aiMat, AI_MATKEY_SHININESS_STRENGTH, &strength, &maxValue)
          == AI_SUCCESS) {
        shininess *= strength;
      }
      material.shininess = shininess;
    }

    // Extract texture paths for all texture types
    // Check common texture types and store them
    const aiTextureType textureTypes[]
        = {aiTextureType_DIFFUSE,
           aiTextureType_SPECULAR,
           aiTextureType_NORMALS,
           aiTextureType_AMBIENT,
           aiTextureType_EMISSIVE,
           aiTextureType_HEIGHT,
           aiTextureType_SHININESS,
           aiTextureType_OPACITY,
           aiTextureType_DISPLACEMENT,
           aiTextureType_LIGHTMAP,
           aiTextureType_REFLECTION};

    for (const auto& type : textureTypes) {
      const auto count = aiMat->GetTextureCount(type);
      for (auto j = 0u; j < count; ++j) {
        aiString imagePath;
        if (aiMat->GetTexture(type, j, &imagePath) == AI_SUCCESS) {
          const std::string imagePathString = imagePath.C_Str();
          if (imagePathString.empty()) {
            continue;
          }

          const common::filesystem::path relativeImagePath = imagePathString;
          std::error_code ec;
          bool attemptedCanonicalize = false;
          if (!basePath.empty() || relativeImagePath.is_absolute()) {
            const common::filesystem::path absoluteImagePath
                = common::filesystem::canonical(
                    meshPath.parent_path() / relativeImagePath, ec);
            attemptedCanonicalize = true;
            if (!ec) {
              material.textureImagePaths.emplace_back(
                  absoluteImagePath.string());
              continue;
            }
          }

          bool resolved = false;
          if (meshUri.mPath) {
            common::Uri resolvedUri;
            if (resolvedUri.fromRelativeUri(
                    meshUri, std::string_view{imagePathString})) {
              material.textureImagePaths.emplace_back(resolvedUri.toString());
              resolved = true;
            }
          }

          if (!resolved) {
            material.textureImagePaths.emplace_back(imagePathString);
          }

          if (meshPathExists && attemptedCanonicalize && !resolved) {
            DART_WARN(
                "[MeshShape::extractMaterialsFromScene] Failed to resolve "
                "texture image path from (base: `{}`, relative: '{}').",
                meshPath.parent_path().string(),
                relativeImagePath.string());
          }
        }
      }
    }

    mMaterials.emplace_back(std::move(material));
  }
}

//==============================================================================
Eigen::Matrix3d MeshShape::computeInertia(double _mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), _mass);
}

//==============================================================================
ShapePtr MeshShape::clone() const
{
  std::shared_ptr<math::TriMesh<double>> clonedTriMesh;
  if (const auto triMesh = getTriMesh()) {
    clonedTriMesh = std::make_shared<math::TriMesh<double>>(*triMesh);
  }

  auto new_shape = std::make_shared<MeshShape>(mScale, clonedTriMesh, mMeshUri);
  if (const auto polygonMesh = getPolygonMesh()) {
    new_shape->mPolygonMesh
        = std::make_shared<math::PolygonMesh<double>>(*polygonMesh);
  }
  new_shape->mMeshPath = mMeshPath;
  new_shape->mResourceRetriever = mResourceRetriever;
  new_shape->mDisplayList = mDisplayList;
  new_shape->mColorMode = mColorMode;
  new_shape->mAlphaMode = mAlphaMode;
  new_shape->mColorIndex = mColorIndex;
  new_shape->mMaterials = mMaterials;
  new_shape->mSubMeshRanges = mSubMeshRanges;
  new_shape->mTextureCoords = mTextureCoords;
  new_shape->mTextureCoordComponents = mTextureCoordComponents;

  return new_shape;
}

//==============================================================================
void MeshShape::updateBoundingBox() const
{
  // Use getTriMesh() instead of directly accessing mTriMesh
  // This handles lazy conversion for backward compatibility with
  // CustomMeshShape
  const auto* triMesh = getTriMesh().get();

  if (!triMesh || triMesh->getVertices().empty()) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d minPoint
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maxPoint = -minPoint;

  const auto& vertices = triMesh->getVertices();
  for (const auto& vertex : vertices) {
    const Eigen::Vector3d scaledVertex = vertex.cwiseProduct(mScale);
    minPoint = minPoint.cwiseMin(scaledVertex);
    maxPoint = maxPoint.cwiseMax(scaledVertex);
  }

  mBoundingBox.setMin(minPoint);
  mBoundingBox.setMax(maxPoint);

  mIsBoundingBoxDirty = false;
}

//==============================================================================
void MeshShape::updateVolume() const
{
  const Eigen::Vector3d bounds = getBoundingBox().computeFullExtents();
  mVolume = bounds.x() * bounds.y() * bounds.z();
  mIsVolumeDirty = false;
}

//==============================================================================
aiScene* MeshShape::cloneMesh() const
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = getMesh();
  DART_SUPPRESS_DEPRECATED_END
  if (!scene)
    return nullptr;

  aiScene* new_scene = nullptr;
  aiCopyScene(scene, &new_scene);
  return new_scene;
}

//==============================================================================
namespace {

bool hasColladaExtension(std::string_view path)
{
  const std::size_t extensionIndex = path.find_last_of('.');
  if (extensionIndex == std::string_view::npos)
    return false;

  std::string extension(path.substr(extensionIndex));
  std::transform(
      extension.begin(), extension.end(), extension.begin(), ::tolower);
  return extension == ".dae" || extension == ".zae";
}

bool isColladaResource(
    std::string_view uri, const common::ResourceRetrieverPtr& retriever)
{
  if (hasColladaExtension(uri))
    return true;

  const auto parsedUri = common::Uri::createFromStringOrPath(std::string(uri));
  if (parsedUri.mScheme.get_value_or("file") == "file" && parsedUri.mPath) {
    if (hasColladaExtension(parsedUri.mPath.get()))
      return true;
  }

  if (!retriever)
    return false;

  const auto resource = retriever->retrieve(parsedUri);
  if (!resource)
    return false;

  constexpr std::size_t kMaxProbeSize = 4096;
  const auto sampleSize = std::min(kMaxProbeSize, resource->getSize());
  std::string buffer(sampleSize, '\0');
  const auto read = resource->read(buffer.data(), 1, sampleSize);
  buffer.resize(read);
  resource->seek(0, common::Resource::SEEKTYPE_SET);

  const auto upper = buffer.find("COLLADA");
  const auto lower = buffer.find("collada");
  const auto mixed = buffer.find("Collada");
  return upper != std::string::npos || lower != std::string::npos
         || mixed != std::string::npos;
}

} // namespace

const aiScene* MeshShape::loadMesh(
    const std::string& _uri, const common::ResourceRetrieverPtr& retriever)
{
  const bool isCollada = isColladaResource(_uri, retriever);

  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION
  if (isCollada) {
    // Keep authoring up-axis and allow us to preserve the Collada unit scale.
    aiSetImportPropertyInteger(
        propertyStore, AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
  }
#endif

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API.  Then wrap
  // the IOSystem in an aiFileIO from Assimp's C API. Yes, this API is
  // completely ridiculous...
  aiFileIO* fileIOPtr = nullptr;
  std::optional<AssimpInputResourceRetrieverAdaptor> systemIO;
  std::optional<aiFileIO> fileIO;
  if (retriever) {
    // Suppress deprecation warnings - we need to use this for backward
    // compatibility
    DART_SUPPRESS_DEPRECATED_BEGIN
    systemIO.emplace(retriever);
    fileIO.emplace(createFileIO(&systemIO.value()));
    DART_SUPPRESS_DEPRECATED_END
    fileIOPtr = &fileIO.value();
  }

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
      _uri.c_str(),
      aiProcess_GenNormals | aiProcess_JoinIdenticalVertices
          | aiProcess_SortByPType | aiProcess_OptimizeMeshes,
      fileIOPtr,
      propertyStore);

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if (!scene) {
    DART_WARN("Failed loading mesh '{}'.", _uri);
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

#if !defined(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION)
  if (isCollada && scene->mRootNode)
    scene->mRootNode->mTransformation = aiMatrix4x4();
#endif

  // Finally, pre-transform the vertices. We can't do this as part of the
  // import process, because we may have changed mTransformation above.
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  DART_WARN_IF(!scene, "Failed pre-transforming vertices.");

  aiReleasePropertyStore(propertyStore);

  return scene;
}

//==============================================================================
const aiScene* MeshShape::loadMesh(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = loadMesh(uri.toString(), retriever);
  DART_SUPPRESS_DEPRECATED_END
  return scene;
}

//==============================================================================
const aiScene* MeshShape::loadMesh(const std::string& filePath)
{
  const auto retriever = std::make_shared<common::LocalResourceRetriever>();
  DART_SUPPRESS_DEPRECATED_BEGIN
  const aiScene* scene = loadMesh("file://" + filePath, retriever);
  DART_SUPPRESS_DEPRECATED_END
  return scene;
}

} // namespace dynamics
} // namespace dart
