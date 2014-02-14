/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
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

#include <iostream>
#include <limits>
#include <string>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include "dart/renderer/RenderInterface.h"

namespace dart {
namespace dynamics {

MeshShape::MeshShape(const Eigen::Vector3d& _scale, const aiScene* _mesh)
  : Shape(MESH),
    mScale(_scale),
    mMesh(_mesh),
    mDisplayList(0) {
  assert(_scale[0] > 0.0);
  assert(_scale[1] > 0.0);
  assert(_scale[2] > 0.0);
  _updateBoundingBoxDim();
  computeVolume();
  initMeshes();
}

MeshShape::~MeshShape() {
}

const aiScene* MeshShape::getMesh() const {
  return mMesh;
}

void MeshShape::setMesh(const aiScene* _mesh) {
  assert(_mesh);
  mMesh = _mesh;
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

const aiScene* MeshShape::loadMesh(const std::string& _fileName) {
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  // remove points and lines
  aiSetImportPropertyInteger(propertyStore,
                             AI_CONFIG_PP_SBP_REMOVE,
                             aiPrimitiveType_POINT | aiPrimitiveType_LINE);
  const aiScene* scene =
      aiImportFileExWithProperties(_fileName.c_str(),
                                   aiProcess_GenNormals             |
                                   aiProcess_Triangulate            |
                                   aiProcess_JoinIdenticalVertices  |
                                   aiProcess_SortByPType            |
                                   aiProcess_OptimizeMeshes,
                                   NULL, propertyStore);
  aiReleasePropertyStore(propertyStore);

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  if (_fileName.length() >= 4
     && _fileName.substr(_fileName.length() - 4, 4) == ".dae") {
    scene->mRootNode->mTransformation = aiMatrix4x4();
  }
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);

  return scene;
}

}  // namespace dynamics
}  // namespace dart
