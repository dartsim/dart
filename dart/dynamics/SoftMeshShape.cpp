/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/SoftMeshShape.hpp"

#include "dart/common/Console.hpp"

#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

namespace dart {
namespace dynamics {

SoftMeshShape::SoftMeshShape(SoftBodyNode* _softBodyNode)
  : Shape(SOFT_MESH),
    mSoftBodyNode(_softBodyNode),
    mAssimpMesh(nullptr)
{
  assert(_softBodyNode != nullptr);
  // Build mesh here using soft body node
  // TODO(JS): Not implemented.
  _buildMesh();
  mVariance = DYNAMIC_VERTICES;
}

SoftMeshShape::~SoftMeshShape()
{
  // Do nothing
}

//==============================================================================
const std::string& SoftMeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& SoftMeshShape::getStaticType()
{
  static const std::string type("SoftMeshShape");
  return type;
}

const aiMesh* SoftMeshShape::getAssimpMesh() const
{
  return mAssimpMesh.get();
}

const SoftBodyNode* SoftMeshShape::getSoftBodyNode() const
{
  return mSoftBodyNode;
}

Eigen::Matrix3d SoftMeshShape::computeInertia(double /*mass*/) const
{
  dtwarn << "[SoftMeshShape::computeInertia] Not implemented yet.\n";
  // TODO(JS): Not implemented.

  return Eigen::Matrix3d::Zero();
}

void SoftMeshShape::updateVolume()
{
  // TODO(JS): Not implemented.
}

void SoftMeshShape::_buildMesh()
{
  // Get number of vertices and faces from soft body node
  int nVertices = mSoftBodyNode->getNumPointMasses();
  int nFaces    = mSoftBodyNode->getNumFaces();

  // Create new aiMesh
  mAssimpMesh = common::make_unique<aiMesh>();

  // Set vertices and normals
  mAssimpMesh->mNumVertices = nVertices;
  mAssimpMesh->mVertices    = new aiVector3D[nVertices];
  mAssimpMesh->mNormals     = new aiVector3D[nVertices];
  aiVector3D itAIVector3d;
  for (int i = 0; i < nVertices; ++i)
  {
    PointMass* itPointMass        = mSoftBodyNode->getPointMass(i);
    const Eigen::Vector3d& vertex = itPointMass->getRestingPosition();
    itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
    mAssimpMesh->mVertices[i] = itAIVector3d;
    mAssimpMesh->mNormals[i]  = itAIVector3d;
  }

  // Set faces
  mAssimpMesh->mNumFaces = nFaces;
  mAssimpMesh->mFaces = new aiFace[nFaces];
  for (int i = 0; i < nFaces; ++i)
  {
    Eigen::Vector3i itFace = mSoftBodyNode->getFace(i);
    aiFace* itAIFace = &mAssimpMesh->mFaces[i];
    itAIFace->mNumIndices = 3;
    itAIFace->mIndices    = new unsigned int[3];
    itAIFace->mIndices[0] = itFace[0];
    itAIFace->mIndices[1] = itFace[1];
    itAIFace->mIndices[2] = itFace[2];
  }
}

void SoftMeshShape::update()
{
  std::size_t nVertices = mSoftBodyNode->getNumPointMasses();

  aiVector3D itAIVector3d;
  for (std::size_t i = 0; i < nVertices; ++i)
  {
    PointMass* itPointMass        = mSoftBodyNode->getPointMass(i);
    const Eigen::Vector3d& vertex = itPointMass->getLocalPosition();
    itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
    mAssimpMesh->mVertices[i] = itAIVector3d;
  }
}

}  // namespace dynamics
}  // namespace dart
