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
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Resource.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/BoxShape.hpp"

#include <assimp/Importer.hpp>
#include <assimp/cexport.h>
#include <assimp/cimport.h>
#include <assimp/config.h>
#include <assimp/postprocess.h>

#include <algorithm>
#include <limits>
#include <string>

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
    const common::Uri& path,
    common::ResourceRetrieverPtr resourceRetriever,
    MeshOwnership ownership)
  : Shape(MESH),
    mMesh(nullptr),
    mMeshOwnership(MeshOwnership::None),
    mDisplayList(0),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setMesh(mesh, ownership, path, std::move(resourceRetriever));
  setScale(scale);
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<const aiScene> mesh,
    const common::Uri& path,
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(MESH),
    mMesh(nullptr),
    mMeshOwnership(MeshOwnership::None),
    mDisplayList(0),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setMesh(std::move(mesh), path, std::move(resourceRetriever));
  setScale(scale);
}

//==============================================================================
MeshShape::~MeshShape()
{
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
const aiScene* MeshShape::getMesh() const
{
  return mMesh.get();
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
  setMesh(mesh, MeshOwnership::Imported, uri, std::move(resourceRetriever));
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

  releaseMesh();

  mMesh = makeMeshHandle(mesh, ownership);
  mMeshOwnership = mesh ? ownership : MeshOwnership::None;

  if (!mMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
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

  releaseMesh();
  mMesh = std::move(mesh);
  mMeshOwnership = mMesh ? MeshOwnership::Custom : MeshOwnership::None;

  if (!mMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
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
Eigen::Matrix3d MeshShape::computeInertia(double _mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), _mass);
}

//==============================================================================
ShapePtr MeshShape::clone() const
{
  aiScene* new_scene = cloneMesh();

  auto new_shape = std::make_shared<MeshShape>(
      mScale, new_scene, mMeshUri, mResourceRetriever, MeshOwnership::Copied);
  new_shape->mMeshPath = mMeshPath;
  new_shape->mDisplayList = mDisplayList;
  new_shape->mColorMode = mColorMode;
  new_shape->mAlphaMode = mAlphaMode;
  new_shape->mColorIndex = mColorIndex;

  return new_shape;
}

//==============================================================================
void MeshShape::updateBoundingBox() const
{
  if (!mMesh) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d minPoint
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maxPoint = -minPoint;

  for (unsigned i = 0u; i < mMesh->mNumMeshes; i++) {
    for (unsigned j = 0u; j < mMesh->mMeshes[i]->mNumVertices; j++) {
      const auto& vertex = mMesh->mMeshes[i]->mVertices[j];
      const Eigen::Vector3d eigenVertex(vertex.x, vertex.y, vertex.z);
      minPoint = minPoint.cwiseMin(eigenVertex.cwiseProduct(mScale));
      maxPoint = maxPoint.cwiseMax(eigenVertex.cwiseProduct(mScale));
    }
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
  if (!mMesh)
    return nullptr;

  aiScene* new_scene = nullptr;
  aiCopyScene(mMesh.get(), &new_scene);
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
