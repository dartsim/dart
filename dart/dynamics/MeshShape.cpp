/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include "dart/config.hpp"
#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
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

  if(mNumMeshes && mMeshes)
    for(std::size_t a=0; a<mNumMeshes; ++a)
      delete mMeshes[a];
  delete[] mMeshes;

  if(mNumMaterials && mMaterials)
    for(std::size_t a=0; a<mNumMaterials; ++a)
      delete mMaterials[a];
  delete[] mMaterials;

  if(mNumAnimations && mAnimations)
    for(std::size_t a=0; a<mNumAnimations; ++a)
      delete mAnimations[a];
  delete[] mAnimations;

  if(mNumTextures && mTextures)
    for(std::size_t a=0; a<mNumTextures; ++a)
      delete mTextures[a];
  delete[] mTextures;

  if(mNumLights && mLights)
    for(std::size_t a=0; a<mNumLights; ++a)
      delete mLights[a];
  delete[] mLights;

  if(mNumCameras && mCameras)
    for(std::size_t a=0; a<mNumCameras; ++a)
      delete mCameras[a];
  delete[] mCameras;
}
#endif  // #if !(ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

// We define our own constructor and destructor for aiMaterial, because it seems
// to be missing from the standard assimp library (see #451)
#if !(ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)
aiMaterial::aiMaterial()
{
  mNumProperties = 0;
  mNumAllocated = 5;
  mProperties = new aiMaterialProperty*[5];
  for(std::size_t i=0; i<5; ++i)
    mProperties[i] = nullptr;
}

aiMaterial::~aiMaterial()
{
  for(std::size_t i=0; i<mNumProperties; ++i)
    delete mProperties[i];

  delete[] mProperties;
}
#endif  // #if !(ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

namespace dart {
namespace dynamics {

MeshShape::MeshShape(const Eigen::Vector3d& _scale, const aiScene* _mesh,
                     const std::string& _path,
                     const common::ResourceRetrieverPtr& _resourceRetriever)
  : Shape(MESH),
    mResourceRetriever(_resourceRetriever),
    mDisplayList(0),
    mColorMode(MATERIAL_COLOR),
    mColorIndex(0)
{
  assert(_scale[0] > 0.0);
  assert(_scale[1] > 0.0);
  assert(_scale[2] > 0.0);

  setMesh(_mesh, _path, _resourceRetriever);
  setScale(_scale);
}

MeshShape::~MeshShape() {
  delete mMesh;
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

const aiScene* MeshShape::getMesh() const {
  return mMesh;
}

const std::string& MeshShape::getMeshUri() const
{
  return mMeshUri;
}

void MeshShape::update()
{
  // Do nothing
}

//==============================================================================
void MeshShape::notifyAlphaUpdate(double alpha)
{
  for(std::size_t i=0; i<mMesh->mNumMeshes; ++i)
  {
    aiMesh* mesh = mMesh->mMeshes[i];
    for(std::size_t j=0; j<mesh->mNumVertices; ++j)
      mesh->mColors[0][j][3] = alpha;
  }
}

const std::string& MeshShape::getMeshPath() const
{
  return mMeshPath;
}

void MeshShape::setMesh(
  const aiScene* _mesh, const std::string& _path,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  mMesh = _mesh;

  if(nullptr == _mesh) {
    mMeshPath = "";
    mMeshUri = "";
    mResourceRetriever = nullptr;
    return;
  }

  common::Uri uri;
  if(uri.fromString(_path))
  {
    mMeshUri = _path;

    if(uri.mScheme.get_value_or("file") == "file")
      mMeshPath = uri.mPath.get_value_or("");
  }
  else
  {
    dtwarn << "[MeshShape::setMesh] Failed parsing URI '" << _path << "'.\n";
    mMeshUri = "";
    mMeshPath = "";
  }

  mResourceRetriever = _resourceRetriever;

  _updateBoundingBoxDim();
  updateVolume();
}

void MeshShape::setScale(const Eigen::Vector3d& _scale) {
  assert(_scale[0] > 0.0);
  assert(_scale[1] > 0.0);
  assert(_scale[2] > 0.0);
  mScale = _scale;
  updateVolume();
  _updateBoundingBoxDim();
}

const Eigen::Vector3d& MeshShape::getScale() const {
  return mScale;
}

void MeshShape::setColorMode(ColorMode _mode)
{
  mColorMode = _mode;
}

MeshShape::ColorMode MeshShape::getColorMode() const
{
  return mColorMode;
}

void MeshShape::setColorIndex(int _index)
{
  mColorIndex = _index;
}

int MeshShape::getColorIndex() const
{
  return mColorIndex;
}

int MeshShape::getDisplayList() const {
  return mDisplayList;
}

void MeshShape::setDisplayList(int _index) {
  mDisplayList = _index;
}

//==============================================================================
Eigen::Matrix3d MeshShape::computeInertia(double _mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(mBoundingBox.computeFullExtents(), _mass);
}

void MeshShape::updateVolume() {
  Eigen::Vector3d bounds = mBoundingBox.computeFullExtents();
  mVolume = bounds.x() * bounds.y() * bounds.z();
}

void MeshShape::_updateBoundingBoxDim() {

  if(!mMesh)
  {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    return;
  }

  double max_X = -std::numeric_limits<double>::infinity();
  double max_Y = -std::numeric_limits<double>::infinity();
  double max_Z = -std::numeric_limits<double>::infinity();
  double min_X = std::numeric_limits<double>::infinity();
  double min_Y = std::numeric_limits<double>::infinity();
  double min_Z = std::numeric_limits<double>::infinity();

  for (unsigned int i = 0; i < mMesh->mNumMeshes; i++) {
    for (unsigned int j = 0; j < mMesh->mMeshes[i]->mNumVertices; j++) {
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
  mBoundingBox.setMin(Eigen::Vector3d(min_X * mScale[0], min_Y * mScale[1], min_Z * mScale[2]));
  mBoundingBox.setMax(Eigen::Vector3d(max_X * mScale[0], max_Y * mScale[1], max_Z * mScale[2]));
}

const aiScene* MeshShape::loadMesh(
  const std::string& _uri, const common::ResourceRetrieverPtr& _retriever)
{
  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(propertyStore,
    AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT
    | aiPrimitiveType_LINE
  );

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API.  Then wrap
  // the IOSystem in an aiFileIO from Assimp's C API. Yes, this API is
  // completely ridiculous...
  AssimpInputResourceRetrieverAdaptor systemIO(_retriever);
  aiFileIO fileIO = createFileIO(&systemIO);

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
    _uri.c_str(), 
      aiProcess_GenNormals
    | aiProcess_Triangulate
    | aiProcess_JoinIdenticalVertices
    | aiProcess_SortByPType
    | aiProcess_OptimizeMeshes,
    &fileIO,
    propertyStore
  );

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if(!scene)
  {
    dtwarn << "[MeshShape::loadMesh] Failed loading mesh '" << _uri << "'.\n";
    return nullptr;
  }

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  std::string extension;
  const std::size_t extensionIndex = _uri.find_last_of('.');
  if(extensionIndex != std::string::npos)
    extension = _uri.substr(extensionIndex);

  std::transform(std::begin(extension), std::end(extension),
                 std::begin(extension), ::tolower);

  if(extension == ".dae" || extension == ".zae")
    scene->mRootNode->mTransformation = aiMatrix4x4();

  // Finally, pre-transform the vertices. We can't do this as part of the
  // import process, because we may have changed mTransformation above.
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  if(!scene)
    dtwarn << "[MeshShape::loadMesh] Failed pre-transforming vertices.\n";

  return scene;
}

const aiScene* MeshShape::loadMesh(const std::string& _fileName)
{
  const auto retriever = std::make_shared<common::LocalResourceRetriever>();
  return loadMesh("file://" + _fileName, retriever);
}

}  // namespace dynamics
}  // namespace dart
