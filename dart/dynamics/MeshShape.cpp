/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s):
 * Date:
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/dynamics/MeshShape.h"
#include "dart/utils/LocalResourceRetriever.h"

#include <iostream>
#include <limits>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include "dart/renderer/RenderInterface.h"
#include "dart/common/Console.h"

// We define our own constructor for aiScene, because it seems to be missing
// from the standard assimp library
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

// We define our own destructor for aiScene, because it seems to be missing
// from the standard assimp library
aiScene::~aiScene()
{
  delete mRootNode;

  if(mNumMeshes && mMeshes)
    for(size_t a=0; a<mNumMeshes; ++a)
      delete mMeshes[a];
  delete[] mMeshes;

  if(mNumMaterials && mMaterials)
    for(size_t a=0; a<mNumMaterials; ++a)
      delete mMaterials[a];
  delete[] mMaterials;

  if(mNumAnimations && mAnimations)
    for(size_t a=0; a<mNumAnimations; ++a)
      delete mAnimations[a];
  delete[] mAnimations;

  if(mNumTextures && mTextures)
    for(size_t a=0; a<mNumTextures; ++a)
      delete mTextures[a];
  delete[] mTextures;

  if(mNumLights && mLights)
    for(size_t a=0; a<mNumLights; ++a)
      delete mLights[a];
  delete[] mLights;

  if(mNumCameras && mCameras)
    for(size_t a=0; a<mNumCameras; ++a)
      delete mCameras[a];
  delete[] mCameras;
}

// We define our own constructor for aiMaterial, because it seems to be missing
// from the standard assimp library
aiMaterial::aiMaterial()
{
  mNumProperties = 0;
  mNumAllocated = 5;
  mProperties = new aiMaterialProperty*[5];
  for(size_t i=0; i<5; ++i)
    mProperties[i] = nullptr;
}

// We define our own destructor for aiMaterial, because it seems to be missing
// from the standard assimp library
aiMaterial::~aiMaterial()
{
  for(size_t i=0; i<mNumProperties; ++i)
    delete mProperties[i];

  delete[] mProperties;
}

namespace dart {
namespace dynamics {

static bool startsWith(const std::string &_target, const std::string &_prefix)
{
  return _target.size() >= _prefix.size()
      && _target.substr(0, _prefix.size()) == _prefix;
}

static bool endsWith(const std::string &_target, const std::string &_suffix)
{
  if( _target.size() >= _suffix.size() )
    std::cout << "Testing: " << _target.substr(_target.size() - _suffix.size()) << std::endl;

  return _target.size() >= _suffix.size()
      && _target.substr(_target.size() - _suffix.size()) == _suffix;
}

static std::string extractPathFromUri(const std::string &_uri)
{
  static const std::string fileSchema = "file://";

  if (startsWith(_uri, fileSchema))
    return _uri.substr(fileSchema.size());
  else
    return "";
}

MeshShape::MeshShape(const Eigen::Vector3d& _scale, const aiScene* _mesh,
                     const std::string &_path, bool _isUri,
                     const utils::ResourceRetrieverPtr& _resourceRetriever)
  : Shape(MESH),
    mResourceRetriever(_resourceRetriever),
    mDisplayList(0)
{
  assert(_scale[0] > 0.0);
  assert(_scale[1] > 0.0);
  assert(_scale[2] > 0.0);

  setMesh(_mesh, _path, _isUri, _resourceRetriever);
  setScale(_scale);
}

MeshShape::~MeshShape() {
  delete mMesh;
}

const aiScene* MeshShape::getMesh() const {
  return mMesh;
}

const std::string &MeshShape::getMeshUri() const
{
  return mMeshUri;
}

const std::string &MeshShape::getMeshPath() const
{
  return mMeshPath;
}

void MeshShape::setMesh(
  const aiScene* _mesh, const std::string &_path, bool _isUri,
  const utils::ResourceRetrieverPtr& _resourceRetriever)
{
  mMesh = _mesh;

  if(nullptr == _mesh) {
    mMeshPath = "";
    mMeshUri = "";
    mResourceRetriever = nullptr;
    return;
  }

  if(_isUri)
  {
    mMeshUri = _path;
    mMeshPath = extractPathFromUri(_path);
  }
  else
  {
    mMeshUri = "file://" + _path;
    mMeshPath = _path;
  }

  mResourceRetriever = _resourceRetriever;

  _updateBoundingBoxDim();
  computeVolume();
}

void MeshShape::setScale(const Eigen::Vector3d& _scale) {
  assert(_scale[0] > 0.0);
  assert(_scale[1] > 0.0);
  assert(_scale[2] > 0.0);
  mScale = _scale;
  computeVolume();
}

const Eigen::Vector3d& MeshShape::getScale() const {
  return mScale;
}

int MeshShape::getDisplayList() const {
  return mDisplayList;
}

void MeshShape::setDisplayList(int _index) {
  mDisplayList = _index;
}

void MeshShape::draw(renderer::RenderInterface* _ri,
                     const Eigen::Vector4d& _color,
                     bool _useDefaultColor) const {
  if (!_ri)
    return;

  if (!_useDefaultColor)
    _ri->setPenColor(_color);
  else
    _ri->setPenColor(mColor);
  _ri->pushMatrix();
  _ri->transform(mTransform);

  _ri->drawMesh(mScale, mMesh);

  _ri->popMatrix();
}

Eigen::Matrix3d MeshShape::computeInertia(double _mass) const {
  // use bounding box to represent the mesh
  double l = mScale[0] * mBoundingBoxDim[0];
  double h = mScale[1] * mBoundingBoxDim[1];
  double w = mScale[2] * mBoundingBoxDim[2];

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  inertia(0, 0) = _mass / 12.0 * (h * h + w * w);
  inertia(1, 1) = _mass / 12.0 * (l * l + w * w);
  inertia(2, 2) = _mass / 12.0 * (l * l + h * h);

  return inertia;
}

void MeshShape::computeVolume() {
  // Use bounding box to represent the mesh
  double l = mScale[0] * mBoundingBoxDim[0];
  double h = mScale[1] * mBoundingBoxDim[1];
  double w = mScale[2] * mBoundingBoxDim[2];

  mVolume = l * h * w;
}

void MeshShape::_updateBoundingBoxDim() {
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
  mBoundingBoxDim[0] = max_X - min_X;
  mBoundingBoxDim[1] = max_Y - min_Y;
  mBoundingBoxDim[2] = max_Z - min_Z;
}

const aiScene* MeshShape::loadMesh(const uint8_t* _data, size_t _size,
                                   const std::string& _uri)
{
  // Extract the file extension.
  std::string extension;
  const size_t extensionIndex = _uri.find_last_of('.');
  if(extensionIndex != std::string::npos)
    extension = _uri.substr(extensionIndex);

  std::transform(std::begin(extension), std::end(extension),
                 std::begin(extension), ::tolower);

  // Use the file extension as a "format hint" for Assimp.
  const char *achFormatHint;
  if(!extension.empty())
    achFormatHint = extension.c_str() + 1; // strip the '.'
  else
    achFormatHint = nullptr;
  
  // Remove points and lines
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(propertyStore,
    AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  const aiScene* scene = aiImportFileFromMemoryWithProperties(
    reinterpret_cast<const char*>(_data), _size,
      aiProcess_GenNormals
    | aiProcess_Triangulate
    | aiProcess_JoinIdenticalVertices
    | aiProcess_SortByPType
    | aiProcess_OptimizeMeshes,
    achFormatHint, propertyStore
  );

  aiReleasePropertyStore(propertyStore);

  if(!scene)
  {
    const char* assimpMessage = aiGetErrorString();
    const char* message
      = (assimpMessage) ? assimpMessage : "An unknown error has occurred.";
    dterr << "[MeshShape::loadMesh] Failed loading mesh: " << message << '\n';
    return nullptr;
  }

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  if(extension == ".dae" || extension == ".zae")
    scene->mRootNode->mTransformation = aiMatrix4x4();

  // Pre-transform the verticies. Assimp states that this post-processing step
  // cannot fail, so we'll assert if it returns nullptr.
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  assert(scene);

  return scene;
}

const aiScene* MeshShape::loadMesh(const utils::MemoryResource& _resource,
                                   const std::string& _uri)
{
  return loadMesh(_resource.getData(), _resource.getSize(), _uri);
}

const aiScene* MeshShape::loadMesh(const std::string& _fileName)
{
  utils::LocalResourceRetriever retriever;
  const std::string uri = "file://" + _fileName;
  return loadMesh(*retriever.retrieve(uri), uri);
}

}  // namespace dynamics
}  // namespace dart
