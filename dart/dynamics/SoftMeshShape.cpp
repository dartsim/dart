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

#include "dart/dynamics/SoftMeshShape.hpp"

#include "dart/common/Console.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

namespace dart {
namespace dynamics {

SoftMeshShape::SoftMeshShape(SoftBodyNode* _softBodyNode)
  : Shape(SOFT_MESH),
    mSoftBodyNode(_softBodyNode),
    mTriMesh(std::make_shared<math::TriMesh<double>>())
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

//==============================================================================
std::shared_ptr<math::TriMesh<double>> SoftMeshShape::getTriMesh() const
{
  return mTriMesh;
}

//==============================================================================
const aiMesh* SoftMeshShape::getAssimpMesh() const
{
  // Convert TriMesh to aiMesh for backward compatibility
  // WARNING: This is expensive and creates a temporary mesh
  dtwarn
      << "[SoftMeshShape::getAssimpMesh] Deprecated method called. "
      << "Use getTriMesh() instead. This performs an expensive conversion.\n";

  auto* aiMesh = new ::aiMesh();
  const auto& vertices = mTriMesh->getVertices();
  const auto& triangles = mTriMesh->getTriangles();

  // Set vertices
  aiMesh->mNumVertices = vertices.size();
  aiMesh->mVertices = new aiVector3D[vertices.size()];
  for (size_t i = 0; i < vertices.size(); ++i) {
    aiMesh->mVertices[i]
        = aiVector3D(vertices[i][0], vertices[i][1], vertices[i][2]);
  }

  // Set faces
  aiMesh->mNumFaces = triangles.size();
  aiMesh->mFaces = new aiFace[triangles.size()];
  for (size_t i = 0; i < triangles.size(); ++i) {
    aiFace* face = &aiMesh->mFaces[i];
    face->mNumIndices = 3;
    face->mIndices = new unsigned int[3];
    face->mIndices[0] = triangles[i][0];
    face->mIndices[1] = triangles[i][1];
    face->mIndices[2] = triangles[i][2];
  }

  return aiMesh;
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
  // TODO(JS): Not implemented.
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
  int nVertices = mSoftBodyNode->getNumPointMasses();
  int nFaces = mSoftBodyNode->getNumFaces();

  // Reserve space for vertices and triangles
  mTriMesh->reserveVertices(nVertices);
  mTriMesh->reserveTriangles(nFaces);

  // Add vertices (using resting positions)
  for (int i = 0; i < nVertices; ++i) {
    PointMass* itPointMass = mSoftBodyNode->getPointMass(i);
    const Eigen::Vector3d& vertex = itPointMass->getRestingPosition();
    mTriMesh->addVertex(vertex);
  }

  // Add faces (triangles)
  for (int i = 0; i < nFaces; ++i) {
    Eigen::Vector3i itFace = mSoftBodyNode->getFace(i);
    mTriMesh->addTriangle(
        math::TriMesh<double>::Triangle(itFace[0], itFace[1], itFace[2]));
  }
}

void SoftMeshShape::update()
{
  // Update vertex positions from soft body node
  std::size_t nVertices = mSoftBodyNode->getNumPointMasses();

  // Get non-const access to vertices for in-place update
  auto& vertices
      = const_cast<std::vector<Eigen::Vector3d>&>(mTriMesh->getVertices());

  for (std::size_t i = 0; i < nVertices; ++i) {
    PointMass* itPointMass = mSoftBodyNode->getPointMass(i);
    vertices[i] = itPointMass->getLocalPosition();
  }
}

} // namespace dynamics
} // namespace dart
