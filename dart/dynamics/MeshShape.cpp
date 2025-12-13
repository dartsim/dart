/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/dynamics/MeshShape.hpp"

#include "dart/common/Diagnostics.hpp"
#include "dart/common/Filesystem.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Resource.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/MeshMaterial.hpp"
#include "dart/dynamics/detail/AssimpInputResourceAdaptor.hpp"

#include <assimp/Importer.hpp>
#include <assimp/cexport.h>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/postprocess.h>

#include <algorithm>
#include <limits>
#include <string>

namespace dart {
namespace dynamics {

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever,
    MeshOwnership ownership)
  : Shape(MESH),
    mMesh(nullptr),
    mMeshOwnership(MeshOwnership::None),
    mTriMesh(nullptr),
    mCachedAiScene(nullptr),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setMesh(mesh, ownership, uri, std::move(resourceRetriever));
  setScale(scale);
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<math::TriMesh<double>> mesh,
    const common::Uri& uri)
  : Shape(MESH),
    mMesh(nullptr),
    mMeshOwnership(MeshOwnership::None),
    mTriMesh(std::move(mesh)),
    mCachedAiScene(nullptr),
    mMeshUri(uri),
    mMeshPath(uri.getPath()),
    mResourceRetriever(nullptr),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setScale(scale);
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<const aiScene> mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(MESH),
    mMesh(nullptr),
    mMeshOwnership(MeshOwnership::None),
    mTriMesh(nullptr),
    mCachedAiScene(nullptr),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setMesh(std::move(mesh), uri, std::move(resourceRetriever));
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
  mMeshOwnership = MeshOwnership::None;
}

//==============================================================================
const std::string& MeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& MeshShape::getStaticType()
{
  static const std::string type("MeshShape");
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
      const_cast<MeshShape*>(this)->mTriMesh = convertAssimpMesh(scene);
    }
  }
  return mTriMesh;
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
  if (!mTriMesh) {
    return nullptr;
  }

  // Create a new aiScene structure
  aiScene* scene = new aiScene();

  // Set up basic scene structure
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh*[1];
  scene->mMeshes[0] = new aiMesh();

  aiMesh* mesh = scene->mMeshes[0];

  const auto& vertices = mTriMesh->getVertices();
  const auto& triangles = mTriMesh->getTriangles();
  const auto& normals = mTriMesh->getVertexNormals();

  // Set vertex data
  mesh->mNumVertices = vertices.size();
  mesh->mVertices = new aiVector3D[mesh->mNumVertices];
  for (size_t i = 0; i < vertices.size(); ++i) {
    mesh->mVertices[i]
        = aiVector3D(vertices[i].x(), vertices[i].y(), vertices[i].z());
  }

  // Set normal data
  if (!normals.empty()) {
    mesh->mNormals = new aiVector3D[mesh->mNumVertices];
    for (size_t i = 0; i < normals.size(); ++i) {
      mesh->mNormals[i]
          = aiVector3D(normals[i].x(), normals[i].y(), normals[i].z());
    }
  }

  // Set face data
  mesh->mNumFaces = triangles.size();
  mesh->mFaces = new aiFace[mesh->mNumFaces];
  for (size_t i = 0; i < triangles.size(); ++i) {
    aiFace& face = mesh->mFaces[i];
    face.mNumIndices = 3;
    face.mIndices = new unsigned int[3];
    face.mIndices[0] = triangles[i].x();
    face.mIndices[1] = triangles[i].y();
    face.mIndices[2] = triangles[i].z();
  }

  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
  mesh->mMaterialIndex = 0;

  // Set up root node
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1];
  scene->mRootNode->mMeshes[0] = 0;

  // Set up a default material
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial*[1];
  scene->mMaterials[0] = new aiMaterial();

  return scene;
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::convertAssimpMesh(
    const aiScene* scene)
{
  if (!scene) {
    return nullptr;
  }

  auto triMesh = std::make_shared<math::TriMesh<double>>();

  // ALWAYS merge all meshes into a single TriMesh
  // This is necessary because:
  // 1. Files with multiple materials (common in COLLADA/OBJ) have multiple aiMesh objects
  // 2. All meshes in a scene belong to the same shape for collision/rendering
  // 3. CustomMeshShape from gz-physics creates one aiMesh per submesh
  const std::size_t numMeshesToProcess = scene->mNumMeshes;

  // Reserve space for all vertices and faces upfront
  std::size_t totalVertices = 0;
  std::size_t totalFaces = 0;
  for (std::size_t i = 0; i < numMeshesToProcess; ++i) {
    if (scene->mMeshes[i]) {
      totalVertices += scene->mMeshes[i]->mNumVertices;
      totalFaces += scene->mMeshes[i]->mNumFaces;
    }
  }
  triMesh->reserveVertices(totalVertices);
  triMesh->reserveTriangles(totalFaces);
  triMesh->reserveVertexNormals(totalVertices);

  // Process all meshes and merge them into a single TriMesh
  for (std::size_t meshIndex = 0; meshIndex < numMeshesToProcess; ++meshIndex) {
    const aiMesh* assimpMesh = scene->mMeshes[meshIndex];

    if (!assimpMesh) {
      continue;
    }

    // Track the vertex offset for this submesh (for face indices)
    const std::size_t vertexOffset = triMesh->getVertices().size();

    // Parse vertices
    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& vertex = assimpMesh->mVertices[i];
      triMesh->addVertex(
          static_cast<double>(vertex.x),
          static_cast<double>(vertex.y),
          static_cast<double>(vertex.z));
    }

    // Parse faces (triangles), adjusting indices by vertex offset
    for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
      const aiFace& face = assimpMesh->mFaces[i];

      // Skip non-triangular faces
      if (face.mNumIndices != 3) {
        DART_WARN(
            "[MeshShape::convertAssimpMesh] Non-triangular face detected in "
            "mesh {}. Skipping this face.",
            meshIndex);
        continue;
      }

      triMesh->addTriangle(
          face.mIndices[0] + vertexOffset,
          face.mIndices[1] + vertexOffset,
          face.mIndices[2] + vertexOffset);
    }

    // Parse vertex normals
    if (assimpMesh->mNormals) {
      for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
        const aiVector3D& normal = assimpMesh->mNormals[i];
        triMesh->addVertexNormal(
            static_cast<double>(normal.x),
            static_cast<double>(normal.y),
            static_cast<double>(normal.z));
      }
    }
  }

  // Compute vertex normals if they are missing or incomplete.
  if (triMesh->hasTriangles()
      && triMesh->getVertexNormals().size() != triMesh->getVertices().size()) {
    triMesh->computeVertexNormals();
  }

  return triMesh;
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
  setMesh(
      mesh,
      MeshOwnership::Imported,
      common::Uri(path),
      std::move(resourceRetriever));
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  // This overload intentionally does not retain the aiScene pointer. Call the
  // MeshOwnership overload if you need explicit lifetime management.

  // Clear cached aiScene to prevent stale data.
  if (mCachedAiScene) {
    aiReleaseImport(mCachedAiScene);
    mCachedAiScene = nullptr;
  }

  mTriMesh = convertAssimpMesh(mesh);
  mMaterials.clear();

  if (!mTriMesh) {
    releaseMesh();
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    mIsBoundingBoxDirty = true;
    mIsVolumeDirty = true;
    return;
  }

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
      mesh, mMeshPath.empty() ? uri.getPath() : mMeshPath);

  // Clear any previously retained aiScene after conversion/material extraction.
  releaseMesh();

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
namespace {

std::shared_ptr<const aiScene> makeMeshHandle(
    const aiScene* mesh, MeshShape::MeshOwnership ownership)
{
  if (!mesh)
    return nullptr;

  switch (ownership) {
    case MeshShape::MeshOwnership::Imported:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene* scene) { //
        aiReleaseImport(const_cast<aiScene*>(scene));
      });
    case MeshShape::MeshOwnership::Copied:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene* scene) {
        aiFreeScene(const_cast<aiScene*>(scene));
      });
    case MeshShape::MeshOwnership::Manual:
      return std::shared_ptr<const aiScene>(mesh, [](const aiScene* scene) {
        delete const_cast<aiScene*>(scene);
      });
    case MeshShape::MeshOwnership::Custom:
    case MeshShape::MeshOwnership::None:
    default:
      return std::shared_ptr<const aiScene>(
          mesh, [](const aiScene*) { /* no-op */ });
  }
}

} // namespace

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    MeshOwnership ownership,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  if (mesh == mMesh.get() && ownership == mMeshOwnership) {
    // Nothing to do.
    return;
  }

  // Clear cached aiScene to prevent stale data.
  if (mCachedAiScene) {
    aiReleaseImport(mCachedAiScene);
    mCachedAiScene = nullptr;
  }

  releaseMesh();

  mMesh = makeMeshHandle(mesh, ownership);
  mMeshOwnership = mesh ? ownership : MeshOwnership::None;

  mTriMesh = convertAssimpMesh(mMesh.get());
  mMaterials.clear();

  if (!mTriMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    mIsBoundingBoxDirty = true;
    mIsVolumeDirty = true;
    return;
  }

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
  extractMaterialsFromScene(mMesh.get(), mMeshPath.empty() ? uri.getPath() : mMeshPath);

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
  if (mesh == mMesh && mMeshOwnership == MeshOwnership::Custom) {
    return;
  }

  if (mCachedAiScene) {
    aiReleaseImport(mCachedAiScene);
    mCachedAiScene = nullptr;
  }

  releaseMesh();
  mMesh = std::move(mesh);
  mMeshOwnership = mMesh ? MeshOwnership::Custom : MeshOwnership::None;

  mTriMesh = convertAssimpMesh(mMesh.get());
  mMaterials.clear();

  if (!mTriMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    mIsBoundingBoxDirty = true;
    mIsVolumeDirty = true;
    return;
  }

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
  extractMaterialsFromScene(mMesh.get(), mMeshPath.empty() ? uri.getPath() : mMeshPath);

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
const std::vector<MeshMaterial>& MeshShape::getMaterials() const
{
  return mMaterials;
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
void MeshShape::extractMaterialsFromScene(
    const aiScene* scene, const std::string& basePath)
{
  if (!scene || scene->mNumMaterials == 0) {
    return;
  }

  mMaterials.clear();
  mMaterials.reserve(scene->mNumMaterials);

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
          const common::filesystem::path relativeImagePath = imagePath.C_Str();
          const common::filesystem::path meshPath = basePath;
          std::error_code ec;
          const common::filesystem::path absoluteImagePath
              = common::filesystem::canonical(
                  meshPath.parent_path() / relativeImagePath, ec);

          if (ec) {
            DART_WARN(
                "[MeshShape::extractMaterialsFromScene] Failed to resolve "
                "texture image path from (base: `{}`, relative: '{}').",
                meshPath.parent_path().string(),
                relativeImagePath.string());
          } else {
            material.textureImagePaths.emplace_back(absoluteImagePath.string());
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
  new_shape->mMeshPath = mMeshPath;
  new_shape->mResourceRetriever = mResourceRetriever;
  new_shape->mDisplayList = mDisplayList;
  new_shape->mColorMode = mColorMode;
  new_shape->mAlphaMode = mAlphaMode;
  new_shape->mColorIndex = mColorIndex;
  new_shape->mMaterials = mMaterials;

  return new_shape;
}

//==============================================================================
void MeshShape::updateBoundingBox() const
{
  // Use getTriMesh() instead of directly accessing mTriMesh
  // This handles lazy conversion for backward compatibility with CustomMeshShape
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

bool hasColladaExtension(const std::string& path)
{
  const std::size_t extensionIndex = path.find_last_of('.');
  if (extensionIndex == std::string::npos)
    return false;

  std::string extension = path.substr(extensionIndex);
  std::transform(
      extension.begin(), extension.end(), extension.begin(), ::tolower);
  return extension == ".dae" || extension == ".zae";
}

bool isColladaResource(
    const std::string& uri, const common::ResourceRetrieverPtr& retriever)
{
  if (hasColladaExtension(uri))
    return true;

  const auto parsedUri = common::Uri::createFromStringOrPath(uri);
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
  // Suppress deprecation warnings - we need to use this for backward
  // compatibility
  DART_SUPPRESS_DEPRECATED_BEGIN
  AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = createFileIO(&systemIO);
  DART_SUPPRESS_DEPRECATED_END

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
      _uri.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
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
  return loadMesh(uri.toString(), retriever);
}

//==============================================================================
const aiScene* MeshShape::loadMesh(const std::string& filePath)
{
  const auto retriever = std::make_shared<common::LocalResourceRetriever>();
  return loadMesh("file://" + filePath, retriever);
}

} // namespace dynamics
} // namespace dart
