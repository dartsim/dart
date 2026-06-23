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

#include "dart/dynamics/MeshShape.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Deprecated.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/BoxShape.hpp"

#include <assimp/Importer.hpp>
#include <assimp/cexport.h>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/material.h>
#include <assimp/postprocess.h>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <utility>

#if !(ASSIMP_AISCENE_CTOR_DTOR_DEFINED)
// We define our own constructor and destructor for aiScene, because it seems to
// be missing from the standard assimp library (see #451)
aiScene::aiScene()
  : mFlags(0),
    mRootNode(nullptr),
    mNumMeshes(0),
    mMeshes(nullptr),
    mNumMaterials(0),
    mMaterials(nullptr),
    mAnimations(nullptr),
    mNumTextures(0),
    mTextures(nullptr),
    mNumLights(0),
    mLights(nullptr),
    mNumCameras(0),
    mCameras(nullptr)
{
}

aiScene::~aiScene()
{
  delete mRootNode;

  if (mNumMeshes && mMeshes)
    for (std::size_t a = 0; a < mNumMeshes; ++a)
      delete mMeshes[a];
  delete[] mMeshes;

  if (mNumMaterials && mMaterials)
    for (std::size_t a = 0; a < mNumMaterials; ++a)
      delete mMaterials[a];
  delete[] mMaterials;

  if (mNumAnimations && mAnimations)
    for (std::size_t a = 0; a < mNumAnimations; ++a)
      delete mAnimations[a];
  delete[] mAnimations;

  if (mNumTextures && mTextures)
    for (std::size_t a = 0; a < mNumTextures; ++a)
      delete mTextures[a];
  delete[] mTextures;

  if (mNumLights && mLights)
    for (std::size_t a = 0; a < mNumLights; ++a)
      delete mLights[a];
  delete[] mLights;

  if (mNumCameras && mCameras)
    for (std::size_t a = 0; a < mNumCameras; ++a)
      delete mCameras[a];
  delete[] mCameras;
}
#endif // #if !(ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

// We define our own constructor and destructor for aiMaterial, because it seems
// to be missing from the standard assimp library (see #451)
#if !(ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)
aiMaterial::aiMaterial()
{
  mNumProperties = 0;
  mNumAllocated = 5;
  mProperties = new aiMaterialProperty*[5];
  for (std::size_t i = 0; i < 5; ++i)
    mProperties[i] = nullptr;
}

aiMaterial::~aiMaterial()
{
  for (std::size_t i = 0; i < mNumProperties; ++i) {
    auto* prop = mProperties[i];
    if (!prop)
      continue;
    delete[] prop->mData;
    delete prop;
  }

  delete[] mProperties;
}
#endif // #if !(ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

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
    mTriMesh(nullptr),
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
    mCachedAiScene(nullptr),
    mMeshUri(uri),
    mResourceRetriever(std::move(resourceRetriever)),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  if (mResourceRetriever)
    mMeshPath = mResourceRetriever->getFilePath(uri);

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
}

//==============================================================================
void MeshShape::setLegacyMesh(const aiScene* mesh, MeshOwnership ownership)
{
  if (mesh == mMesh.get()) {
    notifyLegacyMeshChanged();
    return;
  }

  const bool meshIsCached = mesh && mesh == mCachedAiScene;
  const MeshOwnership effectiveOwnership
      = meshIsCached ? MeshOwnership::Imported : ownership;

  if (mCachedAiScene) {
    if (meshIsCached)
      mCachedAiScene = nullptr;
    else
      aiReleaseImport(mCachedAiScene);
  }

  releaseMesh();
  mMesh.set(mesh, effectiveOwnership);
  notifyLegacyMeshChanged();
}

//==============================================================================
void MeshShape::notifyLegacyMeshChanged()
{
  mTriMesh = convertAssimpMesh(mMesh.get(), &mSubMeshRanges);
  extractMaterialsFromScene(mMesh.get());

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;
  incrementVersion();
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
  set(mesh, MeshOwnership::None);
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
const aiScene* MeshShape::getMesh() const
{
  if (mMesh)
    return mMesh.get();

  if (!mCachedAiScene)
    mCachedAiScene = convertToAssimpMesh();

  return mCachedAiScene;
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::getTriMesh() const
{
  if (!mTriMesh) {
    const aiScene* scene = nullptr;
    if (mMesh)
      scene = mMesh.get();
    else if (mCachedAiScene)
      scene = mCachedAiScene;

    if (scene) {
      auto* self = const_cast<MeshShape*>(this);
      self->mTriMesh = convertAssimpMesh(scene, &self->mSubMeshRanges);
      self->extractMaterialsFromScene(scene);
    }
  }

  return mTriMesh;
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
  if (!scene)
    return nullptr;

  auto triMesh = std::make_shared<math::TriMesh<double>>();
  if (subMeshes)
    subMeshes->clear();

  std::size_t totalVertices = 0;
  std::size_t totalFaces = 0;
  for (auto i = 0u; i < scene->mNumMeshes; ++i) {
    const aiMesh* mesh = scene->mMeshes[i];
    if (!mesh)
      continue;

    totalVertices += mesh->mNumVertices;
    totalFaces += mesh->mNumFaces;
  }

  triMesh->reserveVertices(totalVertices);
  triMesh->reserveTriangles(totalFaces);
  triMesh->reserveVertexNormals(totalVertices);

  for (auto meshIndex = 0u; meshIndex < scene->mNumMeshes; ++meshIndex) {
    const aiMesh* mesh = scene->mMeshes[meshIndex];
    if (!mesh)
      continue;

    const std::size_t vertexOffset = triMesh->getVertices().size();
    const std::size_t triangleOffset = triMesh->getTriangles().size();
    std::size_t triangleCount = 0;

    for (auto i = 0u; i < mesh->mNumVertices; ++i) {
      const aiVector3D& vertex = mesh->mVertices[i];
      triMesh->addVertex(vertex.x, vertex.y, vertex.z);
    }

    for (auto i = 0u; i < mesh->mNumFaces; ++i) {
      const aiFace& face = mesh->mFaces[i];
      if (face.mNumIndices != 3)
        continue;

      triMesh->addTriangle(
          face.mIndices[0] + vertexOffset,
          face.mIndices[1] + vertexOffset,
          face.mIndices[2] + vertexOffset);
      ++triangleCount;
    }

    if (mesh->mNormals) {
      for (auto i = 0u; i < mesh->mNumVertices; ++i) {
        const aiVector3D& normal = mesh->mNormals[i];
        triMesh->addVertexNormal(normal.x, normal.y, normal.z);
      }
    }

    if (subMeshes) {
      SubMeshRange range;
      range.vertexOffset = vertexOffset;
      range.vertexCount = mesh->mNumVertices;
      range.triangleOffset = triangleOffset;
      range.triangleCount = triangleCount;
      range.materialIndex = mesh->mMaterialIndex;
      subMeshes->push_back(range);
    }
  }

  if (triMesh->hasTriangles()
      && triMesh->getVertexNormals().size() != triMesh->getVertices().size()) {
    triMesh->computeVertexNormals();
  }

  return triMesh;
}

//==============================================================================
const aiScene* MeshShape::convertToAssimpMesh() const
{
  if (!mTriMesh || !mTriMesh->hasTriangles())
    return nullptr;

  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10);

  const auto& vertices = mTriMesh->getVertices();
  for (const auto& vertex : vertices) {
    stream << "v " << vertex.x() << ' ' << vertex.y() << ' ' << vertex.z()
           << '\n';
  }

  const auto& normals = mTriMesh->getVertexNormals();
  const bool hasNormals = normals.size() == vertices.size();
  if (hasNormals) {
    for (const auto& normal : normals) {
      stream << "vn " << normal.x() << ' ' << normal.y() << ' ' << normal.z()
             << '\n';
    }
  }

  for (const auto& triangle : mTriMesh->getTriangles()) {
    const std::size_t i0 = static_cast<std::size_t>(triangle.x()) + 1;
    const std::size_t i1 = static_cast<std::size_t>(triangle.y()) + 1;
    const std::size_t i2 = static_cast<std::size_t>(triangle.z()) + 1;

    if (hasNormals) {
      stream << "f " << i0 << "//" << i0 << ' ' << i1 << "//" << i1 << ' ' << i2
             << "//" << i2 << '\n';
    } else {
      stream << "f " << i0 << ' ' << i1 << ' ' << i2 << '\n';
    }
  }

  const std::string obj = stream.str();
  const unsigned int flags = aiProcess_Triangulate
                             | aiProcess_JoinIdenticalVertices
                             | aiProcess_SortByPType | aiProcess_OptimizeMeshes
                             | aiProcess_GenNormals;

  const aiScene* scene = aiImportFileFromMemory(
      obj.data(), static_cast<unsigned int>(obj.size()), flags, "obj");

  if (!scene) {
    dtwarn << "[MeshShape::convertToAssimpMesh] Failed to import TriMesh via "
              "Assimp: "
           << aiGetErrorString() << "\n";
  }

  return scene;
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
  const bool sameMesh
      = mesh == mMesh.get() && ownership == mMesh.getOwnership();
  const bool sameMetadata = mMeshUri.toString() == uri.toString()
                            && mResourceRetriever == resourceRetriever;
  if (sameMesh && sameMetadata)
    return;

  if (!sameMesh) {
    const bool meshIsCached = mesh && mesh == mCachedAiScene;
    const MeshOwnership effectiveOwnership
        = meshIsCached ? MeshOwnership::Imported : ownership;

    if (mCachedAiScene) {
      if (meshIsCached)
        mCachedAiScene = nullptr;
      else
        aiReleaseImport(mCachedAiScene);
    }

    releaseMesh();
    mMesh.set(mesh, effectiveOwnership);
    mTriMesh = convertAssimpMesh(mMesh.get(), &mSubMeshRanges);
    extractMaterialsFromScene(mMesh.get());
  }

  mMeshUri = uri;
  mMeshPath
      = resourceRetriever ? resourceRetriever->getFilePath(uri) : std::string();
  mResourceRetriever = std::move(resourceRetriever);

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
  const bool sameMesh = mesh && mesh.get() == mMesh.get()
                        && mMesh.getOwnership() == MeshOwnership::Custom;
  const bool sameMetadata = mMeshUri.toString() == uri.toString()
                            && mResourceRetriever == resourceRetriever;
  if (sameMesh && sameMetadata)
    return;

  if (!sameMesh) {
    if (mCachedAiScene) {
      if (mesh && mesh.get() == mCachedAiScene)
        mCachedAiScene = nullptr;
      else
        aiReleaseImport(mCachedAiScene);
    }

    releaseMesh();
    mMesh.set(std::move(mesh));
    mTriMesh = convertAssimpMesh(mMesh.get(), &mSubMeshRanges);
    extractMaterialsFromScene(mMesh.get());
  }

  mMeshUri = uri;
  mMeshPath
      = resourceRetriever ? resourceRetriever->getFilePath(uri) : std::string();
  mResourceRetriever = std::move(resourceRetriever);

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
  if (index < mMaterials.size())
    return &mMaterials[index];

  return nullptr;
}

//==============================================================================
void MeshShape::extractMaterialsFromScene(const aiScene* scene)
{
  mMaterials.clear();
  if (!scene || scene->mNumMaterials == 0)
    return;

  mMaterials.reserve(scene->mNumMaterials);
  for (auto i = 0u; i < scene->mNumMaterials; ++i) {
    aiMaterial* aiMat = scene->mMaterials[i];
    if (!aiMat)
      continue;

    MeshMaterial material;

    aiColor4D color;
    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_AMBIENT, &color)
        == AI_SUCCESS) {
      material.ambient = Eigen::Vector4f(color.r, color.g, color.b, color.a);
    }
    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_DIFFUSE, &color)
        == AI_SUCCESS) {
      material.diffuse = Eigen::Vector4f(color.r, color.g, color.b, color.a);
    }
    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_SPECULAR, &color)
        == AI_SUCCESS) {
      material.specular = Eigen::Vector4f(color.r, color.g, color.b, color.a);
    }
    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_EMISSIVE, &color)
        == AI_SUCCESS) {
      material.emissive = Eigen::Vector4f(color.r, color.g, color.b, color.a);
    }

    unsigned int maxValue = 1;
    float shininess = 0.0f;
    if (aiGetMaterialFloatArray(
            aiMat, AI_MATKEY_SHININESS, &shininess, &maxValue)
        == AI_SUCCESS) {
      material.shininess = shininess;
    }

    aiString texturePath;
    if (aiMat->GetTexture(aiTextureType_DIFFUSE, 0, &texturePath)
        == AI_SUCCESS) {
      material.textureImagePaths.emplace_back(texturePath.C_Str());
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
  if (const auto triMesh = getTriMesh())
    clonedTriMesh = std::make_shared<math::TriMesh<double>>(*triMesh);

  auto new_shape = std::make_shared<MeshShape>(mScale, clonedTriMesh, mMeshUri);
  new_shape->mMeshPath = mMeshPath;
  new_shape->mResourceRetriever = mResourceRetriever;
  new_shape->mDisplayList = mDisplayList;
  new_shape->mColorMode = mColorMode;
  new_shape->mAlphaMode = mAlphaMode;
  new_shape->mColorIndex = mColorIndex;
  new_shape->mMaterials = mMaterials;
  new_shape->mSubMeshRanges = mSubMeshRanges;

  return new_shape;
}

//==============================================================================
void MeshShape::updateBoundingBox() const
{
  const auto triMesh = getTriMesh();
  if (!triMesh || triMesh->getVertices().empty()) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d minPoint
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maxPoint = -minPoint;

  for (const auto& vertex : triMesh->getVertices()) {
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

  if (retriever) {
    const auto parsedUri = common::Uri::createFromStringOrPath(uri);
    if (parsedUri.mPath) {
      const std::string resolvedPath = retriever->getFilePath(parsedUri);
      if (!resolvedPath.empty())
        return hasColladaExtension(resolvedPath);
    }
  }

  return false;
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
  AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = createFileIO(&systemIO);

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
    dtwarn << "[MeshShape::loadMesh] Failed loading mesh '" << _uri << "'.\n";
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
  if (!scene)
    dtwarn << "[MeshShape::loadMesh] Failed pre-transforming vertices.\n";

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
