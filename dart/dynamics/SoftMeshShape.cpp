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

#include "dart/dynamics/SoftMeshShape.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

#include <limits>

namespace dart {
namespace dynamics {

SoftMeshShape::SoftMeshShape(SoftBodyNode* _softBodyNode)
  : Shape(SOFT_MESH), mSoftBodyNode(_softBodyNode), mAssimpMesh(nullptr)
{
  DART_ASSERT(_softBodyNode != nullptr);
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

//==============================================================================
const aiMesh* SoftMeshShape::getAssimpMesh() const
{
  return mAssimpMesh.get();
}

//==============================================================================
const SoftBodyNode* SoftMeshShape::getSoftBodyNode() const
{
  return mSoftBodyNode;
}

//==============================================================================
Eigen::Matrix3d SoftMeshShape::computeInertia(double /*mass*/) const
{
  dtwarn << "[SoftMeshShape::computeInertia] Not implemented yet.\n";
  // TODO(JS): Not implemented.

  return Eigen::Matrix3d::Zero();
}

//==============================================================================
ShapePtr SoftMeshShape::clone() const
{
  dtwarn << "[SoftMeshShape::clone] This should never be called.\n";
  return nullptr;
}

//==============================================================================
void SoftMeshShape::updateBoundingBox() const
{
  const auto numPointMasses = mSoftBodyNode->getNumPointMasses();
  if (numPointMasses == 0u) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d min
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  const auto& pointStates = mSoftBodyNode->mAspectState.mPointStates;
  const auto& pointProperties = mSoftBodyNode->mAspectProperties.mPointProps;
  if (pointStates.size() == numPointMasses
      && pointProperties.size() == numPointMasses) {
    for (std::size_t i = 0u; i < numPointMasses; ++i) {
      const Eigen::Vector3d vertex
          = pointStates[i].mPositions + pointProperties[i].mX0;
      min = min.cwiseMin(vertex);
      max = max.cwiseMax(vertex);
    }
  } else {
    const auto& pointMasses = mSoftBodyNode->getPointMasses();
    for (const auto* pointMass : pointMasses) {
      const Eigen::Vector3d& vertex = pointMass->getLocalPosition();
      min = min.cwiseMin(vertex);
      max = max.cwiseMax(vertex);
    }
  }

  mBoundingBox.setMin(min);
  mBoundingBox.setMax(max);
  mIsBoundingBoxDirty = false;
}

//==============================================================================
void SoftMeshShape::updateVolume() const
{
  // TODO(JS): Not implemented.
  mIsVolumeDirty = false;
}

void SoftMeshShape::_buildMesh()
{
  // Get number of vertices and faces from soft body node
  const int nVertices = static_cast<int>(mSoftBodyNode->getNumPointMasses());
  const int nFaces = static_cast<int>(mSoftBodyNode->getNumFaces());

  // Create new aiMesh
  mAssimpMesh = std::make_unique<aiMesh>();

  // Set vertices and normals
  mAssimpMesh->mNumVertices = nVertices;
  mAssimpMesh->mVertices = new aiVector3D[nVertices];
  mAssimpMesh->mNormals = new aiVector3D[nVertices];
  aiVector3D itAIVector3d;
  const auto numPointMasses = static_cast<std::size_t>(nVertices);
  const auto& pointProperties = mSoftBodyNode->mAspectProperties.mPointProps;
  if (pointProperties.size() == numPointMasses) {
    for (int i = 0; i < nVertices; ++i) {
      const Eigen::Vector3d& vertex
          = pointProperties[static_cast<std::size_t>(i)].mX0;
      itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
      mAssimpMesh->mVertices[i] = itAIVector3d;
      mAssimpMesh->mNormals[i] = itAIVector3d;
    }
  } else { // LCOV_EXCL_START: PointMass count follows point properties.
    for (int i = 0; i < nVertices; ++i) {
      PointMass* itPointMass = mSoftBodyNode->getPointMass(i);
      const Eigen::Vector3d& vertex = itPointMass->getRestingPosition();
      itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
      mAssimpMesh->mVertices[i] = itAIVector3d;
      mAssimpMesh->mNormals[i] = itAIVector3d;
    }
  } // LCOV_EXCL_STOP

  // Set faces
  mAssimpMesh->mNumFaces = nFaces;
  mAssimpMesh->mFaces = new aiFace[nFaces];
  const auto& faces = mSoftBodyNode->mAspectProperties.mFaces;
  for (int i = 0; i < nFaces; ++i) {
    const Eigen::Vector3i itFace
        = faces.size() == static_cast<std::size_t>(nFaces)
              ? faces[static_cast<std::size_t>(i)]
              : mSoftBodyNode->getFace(static_cast<std::size_t>(i));
    aiFace* itAIFace = &mAssimpMesh->mFaces[i];
    itAIFace->mNumIndices = 3;
    itAIFace->mIndices = new unsigned int[3];
    itAIFace->mIndices[0] = itFace[0];
    itAIFace->mIndices[1] = itFace[1];
    itAIFace->mIndices[2] = itFace[2];
  }
}

void SoftMeshShape::update()
{
  aiVector3D itAIVector3d;
  const auto numPointMasses = mSoftBodyNode->getNumPointMasses();
  const auto& pointStates = mSoftBodyNode->mAspectState.mPointStates;
  const auto& pointProperties = mSoftBodyNode->mAspectProperties.mPointProps;
  if (pointStates.size() == numPointMasses
      && pointProperties.size() == numPointMasses) {
    for (std::size_t i = 0u; i < numPointMasses; ++i) {
      const Eigen::Vector3d vertex
          = pointStates[i].mPositions + pointProperties[i].mX0;
      itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
      mAssimpMesh->mVertices[i] = itAIVector3d;
    }
  } else {
    const auto& pointMasses = mSoftBodyNode->getPointMasses();
    for (std::size_t i = 0; i < pointMasses.size(); ++i) {
      PointMass* itPointMass = pointMasses[i];
      const Eigen::Vector3d& vertex = itPointMass->getLocalPosition();
      itAIVector3d.Set(vertex[0], vertex[1], vertex[2]);
      mAssimpMesh->mVertices[i] = itAIVector3d;
    }
  }

  mIsBoundingBoxDirty = true;
}

//==============================================================================
void SoftMeshShape::refreshData()
{
  update();
}

} // namespace dynamics
} // namespace dart
