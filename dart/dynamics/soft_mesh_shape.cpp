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

#include "dart/dynamics/soft_mesh_shape.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/dynamics/point_mass.hpp"
#include "dart/dynamics/soft_body_node.hpp"

namespace dart {
namespace dynamics {

SoftMeshShape::SoftMeshShape(SoftBodyNode* _softBodyNode)
  : Shape(SOFT_MESH),
    mSoftBodyNode(_softBodyNode),
    mTriMesh(std::make_shared<math::TriMesh<double>>()),
    mAssimpMesh(nullptr)
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
std::string_view SoftMeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
std::string_view SoftMeshShape::getStaticType()
{
  static constexpr std::string_view type = "SoftMeshShape";
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
  DART_WARN("Not implemented yet.");
  // TODO(JS): Not implemented.

  return Eigen::Matrix3d::Zero();
}

//==============================================================================
ShapePtr SoftMeshShape::clone() const
{
  DART_WARN("This should never be called.");
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

  mAssimpMesh = std::make_unique<aiMesh>();

  // Reserve space for vertices and triangles
  mTriMesh->reserveVertices(nVertices);
  mTriMesh->reserveTriangles(nFaces);

  // Add vertices (using resting positions)
  mAssimpMesh->mNumVertices = nVertices;
  mAssimpMesh->mVertices = new aiVector3D[nVertices];
  mAssimpMesh->mNormals = new aiVector3D[nVertices];

  for (int i = 0; i < nVertices; ++i) {
    PointMass* itPointMass = mSoftBodyNode->getPointMass(i);
    const Eigen::Vector3d& vertex = itPointMass->getRestingPosition();
    mTriMesh->addVertex(vertex);
    aiVector3D aiVertex;
    aiVertex.Set(vertex[0], vertex[1], vertex[2]);
    mAssimpMesh->mVertices[i] = aiVertex;
    mAssimpMesh->mNormals[i] = aiVertex;
  }

  // Add faces (triangles)
  mAssimpMesh->mNumFaces = nFaces;
  mAssimpMesh->mFaces = new aiFace[nFaces];

  for (int i = 0; i < nFaces; ++i) {
    Eigen::Vector3i itFace = mSoftBodyNode->getFace(i);
    mTriMesh->addTriangle(
        math::TriMesh<double>::Triangle(itFace[0], itFace[1], itFace[2]));

    aiFace* aiFacePtr = &mAssimpMesh->mFaces[i];
    aiFacePtr->mNumIndices = 3;
    aiFacePtr->mIndices = new unsigned int[3];
    aiFacePtr->mIndices[0] = itFace[0];
    aiFacePtr->mIndices[1] = itFace[1];
    aiFacePtr->mIndices[2] = itFace[2];
  }
}

void SoftMeshShape::update()
{
  // Update vertex positions from soft body node
  std::size_t nVertices = mSoftBodyNode->getNumPointMasses();

  // Get non-const access to vertices for in-place update
  auto& vertices = mTriMesh->getVertices();

  for (std::size_t i = 0; i < nVertices; ++i) {
    PointMass* itPointMass = mSoftBodyNode->getPointMass(i);
    vertices[i] = itPointMass->getLocalPosition();

    if (mAssimpMesh && mAssimpMesh->mVertices) {
      const auto& vertex = vertices[i];
      mAssimpMesh->mVertices[i].Set(vertex[0], vertex[1], vertex[2]);
    }
  }
}

} // namespace dynamics
} // namespace dart
