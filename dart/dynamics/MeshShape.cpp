/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <limits>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/AssimpInputResourceAdaptor.hpp"
#include "dart/dynamics/BoxShape.hpp"

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
  for (std::size_t i = 0; i < mNumProperties; ++i)
    delete mProperties[i];

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
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(MESH),
    mDisplayList(0),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setMesh(mesh, path, std::move(resourceRetriever));
  setScale(scale);
}

//==============================================================================
MeshShape::~MeshShape()
{
  aiReleaseImport(mMesh);
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
  return mMesh;
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
  setMesh(mesh, common::Uri(path), std::move(resourceRetriever));
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  mMesh = mesh;

  if (!mMesh)
  {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    return;
  }

  mMeshUri = uri;

  if (resourceRetriever)
    mMeshPath = resourceRetriever->getFilePath(uri);
  else
    mMeshPath.clear();

  mResourceRetriever = std::move(resourceRetriever);

  incrementVersion();
}

//==============================================================================
void MeshShape::setScale(const Eigen::Vector3d& scale)
{
  assert((scale.array() > 0.0).all());

  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
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
ShapePtr MeshShape::copy() const
{
  aiScene* new_scene = copyMesh();

  auto new_shape = std::make_shared<MeshShape>(mScale, new_scene, mMeshUri, mResourceRetriever);
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
  if (!mMesh)
  {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  double max_X = -std::numeric_limits<double>::infinity();
  double max_Y = -std::numeric_limits<double>::infinity();
  double max_Z = -std::numeric_limits<double>::infinity();
  double min_X = std::numeric_limits<double>::infinity();
  double min_Y = std::numeric_limits<double>::infinity();
  double min_Z = std::numeric_limits<double>::infinity();

  for (unsigned int i = 0; i < mMesh->mNumMeshes; i++)
  {
    for (unsigned int j = 0; j < mMesh->mMeshes[i]->mNumVertices; j++)
    {
      if (mMesh->mMeshes[i]->mVertices[j].x > max_X)
        max_X = mMesh->mMeshes[i]->mVertices[j].x;
      if (mMesh->mMeshes[i]->mVertices[j].x < min_X)
        min_X = mMesh->mMeshes[i]->mVertices[j].x;
      if (mMesh->mMeshes[i]->mVertices[j].y > max_Y)
        max_Y = mMesh->mMeshes[i]->mVertices[j].y;
      if (mMesh->mMeshes[i]->mVertices[j].y < min_Y)
        min_Y = mMesh->mMeshes[i]->mVertices[j].y;
      if (mMesh->mMeshes[i]->mVertices[j].z > max_Z)
        max_Z = mMesh->mMeshes[i]->mVertices[j].z;
      if (mMesh->mMeshes[i]->mVertices[j].z < min_Z)
        min_Z = mMesh->mMeshes[i]->mVertices[j].z;
    }
  }
  mBoundingBox.setMin(
      Eigen::Vector3d(min_X * mScale[0], min_Y * mScale[1], min_Z * mScale[2]));
  mBoundingBox.setMax(
      Eigen::Vector3d(max_X * mScale[0], max_Y * mScale[1], max_Z * mScale[2]));

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
aiScene* MeshShape::copyMesh() const
{
  // Create new assimp mesh
  aiScene* new_scene = new aiScene();
  // Copy basic data
  new_scene->mNumAnimations = 0; // we do not care about animations // new_scene->mNumAnimations = mMesh->mNumAnimations;
  new_scene->mNumCameras = 0; // we do not care about cameras // new_scene->mNumCameras = mMesh->mNumCameras;
  new_scene->mNumLights = 0; // we do not care about lights // new_scene->mNumLights = mMesh->mNumLights;
  new_scene->mNumMaterials = mMesh->mNumMaterials;
  new_scene->mNumMeshes = mMesh->mNumMeshes;
  new_scene->mNumTextures = mMesh->mNumTextures;
  new_scene->mFlags = mMesh->mFlags;
  // Initialize empty structs
  new_scene->mAnimations = nullptr;
  new_scene->mCameras = nullptr;
  new_scene->mLights = nullptr;

  // Copy materials
  new_scene->mMaterials = new aiMaterial*[new_scene->mNumMaterials];
  for (unsigned int i = 0; i < new_scene->mNumMaterials; i++)
  {
    new_scene->mMaterials[i] = new aiMaterial();
    new_scene->mMaterials[i]->mNumAllocated = mMesh->mMaterials[i]->mNumAllocated;
    new_scene->mMaterials[i]->mNumProperties = mMesh->mMaterials[i]->mNumProperties;

    new_scene->mMaterials[i]->mProperties = new aiMaterialProperty*[new_scene->mMaterials[i]->mNumProperties];

    for (unsigned int j = 0; j < new_scene->mMaterials[i]->mNumProperties; j++)
    {
      new_scene->mMaterials[i]->mProperties[j] = new aiMaterialProperty();
      auto& prop = new_scene->mMaterials[i]->mProperties[j];
      auto& other = mMesh->mMaterials[i]->mProperties[j];

      prop->mKey = other->mKey;
      prop->mSemantic = other->mSemantic;
      prop->mIndex = other->mIndex;
      prop->mDataLength = other->mDataLength;
      prop->mType = other->mType;
      prop->mData = new char[prop->mDataLength];
      memcpy(prop->mData, other->mData, prop->mDataLength);
    }
  }

  // Copy textures
  new_scene->mTextures = new aiTexture*[new_scene->mNumTextures];
  for (unsigned int i = 0; i < new_scene->mNumTextures; i++)
  {
    new_scene->mTextures[i] = new aiTexture();
    strcpy(new_scene->mTextures[i]->achFormatHint, mMesh->mTextures[i]->achFormatHint);
    new_scene->mTextures[i]->mHeight = mMesh->mTextures[i]->mHeight;
    new_scene->mTextures[i]->mWidth = mMesh->mTextures[i]->mWidth;
    new_scene->mTextures[i]->mFilename = mMesh->mTextures[i]->mFilename;
    unsigned int size = new_scene->mTextures[i]->mWidth;
    if (new_scene->mTextures[i]->mHeight > 0)
      size *= new_scene->mTextures[i]->mHeight;
    new_scene->mTextures[i]->pcData = new aiTexel[size];
    memcpy(new_scene->mTextures[i]->pcData, mMesh->mTextures[i]->pcData, size * sizeof(aiTexel));
  }

  // Copy meshes
  new_scene->mMeshes = new aiMesh*[new_scene->mNumMeshes];
  for (unsigned int i = 0; i < new_scene->mNumMeshes; i++)
  {
    new_scene->mMeshes[i] = new aiMesh();
    auto& mesh = new_scene->mMeshes[i];
    auto& other = mMesh->mMeshes[i];
    // Empty - we do not care about animation meshes or bones
    mesh->mAnimMeshes = nullptr;
    mesh->mBones = nullptr;
    mesh->mNumAnimMeshes = 0;
    mesh->mNumBones = 0;
    // Basic info
    mesh->mMaterialIndex = other->mMaterialIndex;
    mesh->mName = other->mName;
    mesh->mNumFaces = other->mNumFaces;
    memcpy(&mesh->mNumUVComponents[0], &other->mNumUVComponents[0], AI_MAX_NUMBER_OF_TEXTURECOORDS * sizeof(unsigned int));
    mesh->mNumVertices = other->mNumVertices;
    mesh->mPrimitiveTypes = other->mPrimitiveTypes;

    if (mesh->mNumVertices > 0) {
      // Copy verticies
      mesh->mVertices = new aiVector3D[mesh->mNumVertices];
      memcpy(mesh->mVertices, other->mVertices, mesh->mNumVertices * sizeof(aiVector3D));

      // Copy normals
      mesh->mNormals = new aiVector3D[mesh->mNumVertices];
      memcpy(mesh->mNormals, other->mNormals, mesh->mNumVertices * sizeof(aiVector3D));

      // Copy faces
      mesh->mFaces = new aiFace[mesh->mNumFaces];
      for (unsigned int a = 0; a < mesh->mNumFaces; a++)
      {
        mesh->mFaces[a].mNumIndices = other->mFaces[a].mNumIndices;
        mesh->mFaces[a].mIndices = new unsigned int[mesh->mFaces[a].mNumIndices];
        memcpy(mesh->mFaces[a].mIndices, other->mFaces[a].mIndices, mesh->mFaces[a].mNumIndices * sizeof(unsigned int));
      }

      // Copy tangents
      if (other->mTangents)
      {
        mesh->mTangents = new aiVector3D[mesh->mNumVertices];
        memcpy(mesh->mTangents, other->mTangents, mesh->mNumVertices * sizeof(aiVector3D));
      }
      // Copy bi-tangents
      if (other->mBitangents)
      {
        mesh->mBitangents = new aiVector3D[mesh->mNumVertices];
        memcpy(mesh->mBitangents, other->mBitangents, mesh->mNumVertices * sizeof(aiVector3D));
      }

      // Copy color sets
      for (unsigned int a = 0; a < AI_MAX_NUMBER_OF_COLOR_SETS; a++)
      {
        if (other->mColors[a])
        {
          mesh->mColors[a] = new aiColor4D[mesh->mNumVertices];
          memcpy(mesh->mColors[a], other->mColors[a], mesh->mNumVertices * sizeof(aiColor4D));
        }
      }

      // Copy texture coordinates
      for (unsigned int a = 0; a < AI_MAX_NUMBER_OF_TEXTURECOORDS; a++)
      {
        if (other->mTextureCoords[a])
        {
          mesh->mTextureCoords[a] = new aiVector3D[mesh->mNumVertices];
          memcpy(mesh->mTextureCoords[a], other->mTextureCoords[a], mesh->mNumVertices * sizeof(aiVector3D));
        }
      }
    }
  }

  // Copy nodes
  std::function<void(aiNode*, aiNode*, aiNode*)> aiNodeCopyRecursive = [&aiNodeCopyRecursive](aiNode* dest, aiNode* src, aiNode* parent)
  {
    dest->mNumMeshes = src->mNumMeshes;
    dest->mNumChildren = src->mNumChildren;
    dest->mName = src->mName;
    dest->mTransformation = src->mTransformation;
    dest->mParent = parent;

    dest->mMeshes = new unsigned int[dest->mNumMeshes];
    memcpy(dest->mMeshes, src->mMeshes, dest->mNumMeshes * sizeof(unsigned int));

    dest->mChildren = new aiNode*[dest->mNumChildren];
    for (unsigned int i = 0; i < dest->mNumChildren; i++)
    {
      dest->mChildren[i] = new aiNode();
      aiNodeCopyRecursive(dest->mChildren[i], src->mChildren[i], dest);
    }
  };

  if (mMesh->mRootNode)
  {
    new_scene->mRootNode = new aiNode();
    aiNodeCopyRecursive(new_scene->mRootNode, mMesh->mRootNode, nullptr);
  }

  return new_scene;
}

//==============================================================================
const aiScene* MeshShape::loadMesh(
    const std::string& _uri, const common::ResourceRetrieverPtr& retriever)
{
  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

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
  if (!scene)
  {
    dtwarn << "[MeshShape::loadMesh] Failed loading mesh '" << _uri << "'.\n";
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  std::string extension;
  const std::size_t extensionIndex = _uri.find_last_of('.');
  if (extensionIndex != std::string::npos)
    extension = _uri.substr(extensionIndex);

  std::transform(
      std::begin(extension),
      std::end(extension),
      std::begin(extension),
      ::tolower);

  if (extension == ".dae" || extension == ".zae")
    scene->mRootNode->mTransformation = aiMatrix4x4();

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
