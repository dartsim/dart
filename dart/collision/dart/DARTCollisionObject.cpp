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

#include "dart/collision/dart/DARTCollisionObject.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CapsuleShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/math/Geometry.hpp"

#include <algorithm>

#include <cmath>

namespace dart {
namespace collision {

namespace {

constexpr double kSoftFaceCacheEps = 1e-6;

//==============================================================================
void updateCachedBounds(
    const Eigen::Vector3d& boundsMin,
    const Eigen::Vector3d& boundsMax,
    Eigen::Vector3d& cachedBoundsMin,
    Eigen::Vector3d& cachedBoundsMax,
    Eigen::Vector3d& cachedBoundsCenter,
    Eigen::Vector3d& cachedBoundsHalfExtents,
    bool& hasFiniteCachedBounds)
{
  cachedBoundsMin = boundsMin;
  cachedBoundsMax = boundsMax;
  cachedBoundsCenter = 0.5 * (cachedBoundsMin + cachedBoundsMax);
  cachedBoundsHalfExtents = 0.5 * (cachedBoundsMax - cachedBoundsMin);
  hasFiniteCachedBounds
      = cachedBoundsMin.allFinite() && cachedBoundsMax.allFinite();
}

} // namespace

//==============================================================================
DARTCollisionObject::DARTCollisionObject(
    CollisionDetector* collisionDetector,
    const dynamics::ShapeFrame* shapeFrame)
  : CollisionObject(collisionDetector, shapeFrame)
{
  refreshShapeCache();
}

//==============================================================================
const dynamics::Shape* DARTCollisionObject::getCachedShape() const
{
  return mCachedShape.get();
}

//==============================================================================
const std::string& DARTCollisionObject::getCachedShapeType() const
{
  static const std::string empty;
  return mCachedShapeType ? *mCachedShapeType : empty;
}

//==============================================================================
DARTCollisionObject::CachedShapeKind DARTCollisionObject::getCachedShapeKind()
    const
{
  return mCachedShapeKind;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsMin() const
{
  return mCachedLocalBoundsMin;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsMax() const
{
  return mCachedLocalBoundsMax;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsCenter() const
{
  return mCachedLocalBoundsCenter;
}

//==============================================================================
const Eigen::Vector3d& DARTCollisionObject::getCachedLocalBoundsHalfExtents()
    const
{
  return mCachedLocalBoundsHalfExtents;
}

//==============================================================================
bool DARTCollisionObject::hasFiniteCachedLocalBounds() const
{
  return mHasFiniteCachedLocalBounds;
}

//==============================================================================
bool DARTCollisionObject::isCachedPlaneShape() const
{
  return mIsCachedPlaneShape;
}

//==============================================================================
const Eigen::Isometry3d& DARTCollisionObject::getWorldTransformForCollision()
    const
{
  return mUseBodyNodeWorldTransform ? mBodyNode->getWorldTransform()
                                    : getTransform();
}

//==============================================================================
const std::vector<Eigen::Vector3d>&
DARTCollisionObject::getCachedSoftLocalVertices() const
{
  return mCachedSoftLocalVertices;
}

//==============================================================================
const std::vector<int>& DARTCollisionObject::getCachedSoftFirstFaceByPointMass()
    const
{
  return mCachedSoftFirstFaceByPointMass;
}

//==============================================================================
const std::vector<DARTCollisionObject::CachedSoftFace>&
DARTCollisionObject::getCachedSoftFaces() const
{
  const_cast<DARTCollisionObject*>(this)->refreshSoftFaceCacheIfNeeded();
  return mCachedSoftFaces;
}

//==============================================================================
const std::vector<DARTCollisionObject::CachedSoftFaceBvhNode>&
DARTCollisionObject::getCachedSoftFaceBvhNodes() const
{
  const_cast<DARTCollisionObject*>(this)->refreshSoftFaceCacheIfNeeded();
  return mCachedSoftFaceBvhNodes;
}

//==============================================================================
const std::vector<int>& DARTCollisionObject::getCachedSoftFaceBvhIndices() const
{
  const_cast<DARTCollisionObject*>(this)->refreshSoftFaceCacheIfNeeded();
  return mCachedSoftFaceBvhIndices;
}

//==============================================================================
void DARTCollisionObject::refreshShapeCache()
{
  const auto shapeFrameVersion = mShapeFrame ? mShapeFrame->getVersion() : 0u;
  const auto currentShape = mShapeFrame ? mShapeFrame->getShape() : nullptr;
  const bool hasSameShape = mCachedShape == currentShape;

  if (hasSameShape && mCachedShapeFrameVersion == shapeFrameVersion
      && mCachedShapeKind != CachedShapeKind::SoftMesh) {
    return;
  }

  if (hasSameShape && mCachedShapeFrameVersion == shapeFrameVersion
      && mCachedShapeKind == CachedShapeKind::SoftMesh) {
    refreshSoftMeshCache();
    return;
  }

  mCachedShapeFrameVersion = shapeFrameVersion;
  mCachedShape = currentShape;
  mCachedShapeType = mCachedShape ? &mCachedShape->getType() : nullptr;
  mCachedShapeKind = CachedShapeKind::Unknown;
  mCachedLocalBoundsMin.setZero();
  mCachedLocalBoundsMax.setZero();
  mCachedLocalBoundsCenter.setZero();
  mCachedLocalBoundsHalfExtents.setZero();
  mHasFiniteCachedLocalBounds = false;
  mIsCachedPlaneShape = false;
  mUseBodyNodeWorldTransform = false;

  if (mShapeNode != nullptr && mBodyNode != nullptr) {
    mUseBodyNodeWorldTransform = mShapeNode->getRelativeTransform().matrix()
                                 == Eigen::Isometry3d::Identity().matrix();
  }

  if (!mCachedShape)
    return;

  const auto& shapeType = *mCachedShapeType;
  if (shapeType == dynamics::PlaneShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Plane;
    mIsCachedPlaneShape = true;
    return;
  }

  if (shapeType == dynamics::SphereShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Sphere;
  } else if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto* ellipsoid
        = static_cast<const dynamics::EllipsoidShape*>(mCachedShape.get());
    if (ellipsoid->isSphere())
      mCachedShapeKind = CachedShapeKind::SphereEllipsoid;
  } else if (shapeType == dynamics::BoxShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Box;
  } else if (shapeType == dynamics::CylinderShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Cylinder;
  } else if (shapeType == dynamics::CapsuleShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::Capsule;
  } else if (shapeType == dynamics::SoftMeshShape::getStaticType()) {
    mCachedShapeKind = CachedShapeKind::SoftMesh;
  }

  if (mCachedShapeKind == CachedShapeKind::SoftMesh) {
    refreshSoftMeshCache();
    return;
  }

  const auto& localBox = mCachedShape->getBoundingBox();
  updateCachedBounds(
      localBox.getMin(),
      localBox.getMax(),
      mCachedLocalBoundsMin,
      mCachedLocalBoundsMax,
      mCachedLocalBoundsCenter,
      mCachedLocalBoundsHalfExtents,
      mHasFiniteCachedLocalBounds);

  mCachedSoftLocalVertices.clear();
  mCachedSoftFirstFaceByPointMass.clear();
  mCachedSoftFaces.clear();
  mCachedSoftFaceBvhNodes.clear();
  mCachedSoftFaceBvhIndices.clear();
  mCachedSoftBodyNodeVersion = std::numeric_limits<std::size_t>::max();
  mCachedSoftFacesDirty = false;
  mCachedSoftFaceTopologyDirty = false;
}

//==============================================================================
void DARTCollisionObject::refreshSoftMeshCache()
{
  const auto* softMesh
      = static_cast<const dynamics::SoftMeshShape*>(mCachedShape.get());
  const auto* softBodyNode
      = softMesh != nullptr ? softMesh->getSoftBodyNode() : nullptr;
  if (softBodyNode == nullptr) {
    mCachedLocalBoundsMin.setZero();
    mCachedLocalBoundsMax.setZero();
    mCachedLocalBoundsCenter.setZero();
    mCachedLocalBoundsHalfExtents.setZero();
    mHasFiniteCachedLocalBounds = false;
    mCachedSoftLocalVertices.clear();
    mCachedSoftFirstFaceByPointMass.clear();
    mCachedSoftFaces.clear();
    mCachedSoftFaceBvhNodes.clear();
    mCachedSoftFaceBvhIndices.clear();
    mCachedSoftBodyNodeVersion = std::numeric_limits<std::size_t>::max();
    mCachedSoftFacesDirty = false;
    mCachedSoftFaceTopologyDirty = false;
    return;
  }

  const auto softBodyNodeVersion = softBodyNode->getVersion();
  const auto numPointMasses = softBodyNode->getNumPointMasses();
  const bool pointMassCountChanged
      = mCachedSoftLocalVertices.size() != numPointMasses;
  bool softGeometryChanged = pointMassCountChanged;
  Eigen::Vector3d boundsMin
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d boundsMax
      = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  if (numPointMasses == 0u) {
    boundsMin.setZero();
    boundsMax.setZero();
  } else {
    if (pointMassCountChanged)
      mCachedSoftLocalVertices.resize(numPointMasses);

    const auto& pointStates = softBodyNode->mAspectState.mPointStates;
    const auto& pointProperties = softBodyNode->mAspectProperties.mPointProps;
    if (pointStates.size() == numPointMasses
        && pointProperties.size() == numPointMasses) {
      for (std::size_t i = 0u; i < numPointMasses; ++i) {
        const auto& position = pointStates[i].mPositions;
        const auto& restingPosition = pointProperties[i].mX0;
        const double x = position[0] + restingPosition[0];
        const double y = position[1] + restingPosition[1];
        const double z = position[2] + restingPosition[2];

        boundsMin[0] = std::min(boundsMin[0], x);
        boundsMin[1] = std::min(boundsMin[1], y);
        boundsMin[2] = std::min(boundsMin[2], z);
        boundsMax[0] = std::max(boundsMax[0], x);
        boundsMax[1] = std::max(boundsMax[1], y);
        boundsMax[2] = std::max(boundsMax[2], z);

        auto& cachedVertex = mCachedSoftLocalVertices[i];
        if (!softGeometryChanged
            && (cachedVertex[0] != x || cachedVertex[1] != y
                || cachedVertex[2] != z)) {
          softGeometryChanged = true;
        }
        cachedVertex[0] = x;
        cachedVertex[1] = y;
        cachedVertex[2] = z;
      }
    } else {
      for (std::size_t i = 0u; i < numPointMasses; ++i) {
        const auto* pointMass = softBodyNode->getPointMass(i);
        const Eigen::Vector3d localPosition
            = pointMass->getPositions() + pointMass->getRestingPosition();
        boundsMin = boundsMin.cwiseMin(localPosition);
        boundsMax = boundsMax.cwiseMax(localPosition);
        if (!softGeometryChanged
            && !mCachedSoftLocalVertices[i].cwiseEqual(localPosition).all()) {
          softGeometryChanged = true;
        }
        mCachedSoftLocalVertices[i] = localPosition;
      }
    }
  }

  const auto& faces = softBodyNode->mAspectProperties.mFaces;
  const auto numFaces = faces.size();
  bool softTopologyChanged
      = pointMassCountChanged || mCachedSoftFaces.size() != numFaces
        || mCachedSoftFirstFaceByPointMass.size() != numPointMasses
        || mCachedSoftBodyNodeVersion != softBodyNodeVersion;

  updateCachedBounds(
      boundsMin,
      boundsMax,
      mCachedLocalBoundsMin,
      mCachedLocalBoundsMax,
      mCachedLocalBoundsCenter,
      mCachedLocalBoundsHalfExtents,
      mHasFiniteCachedLocalBounds);

  if (!softGeometryChanged && !softTopologyChanged)
    return;

  if (softTopologyChanged) {
    mCachedSoftFirstFaceByPointMass.assign(numPointMasses, -1);
    mCachedSoftFaces.resize(numFaces);

    for (std::size_t faceIndex = 0u; faceIndex < numFaces; ++faceIndex) {
      auto& cachedFace = mCachedSoftFaces[faceIndex];
      cachedFace = CachedSoftFace{};
      cachedFace.indices = faces[faceIndex];

      if ((cachedFace.indices.array() < 0).any())
        continue;

      const auto index0 = static_cast<std::size_t>(cachedFace.indices[0]);
      const auto index1 = static_cast<std::size_t>(cachedFace.indices[1]);
      const auto index2 = static_cast<std::size_t>(cachedFace.indices[2]);
      if (index0 >= numPointMasses || index1 >= numPointMasses
          || index2 >= numPointMasses) {
        continue;
      }

      if (mCachedSoftFirstFaceByPointMass[index0] < 0)
        mCachedSoftFirstFaceByPointMass[index0] = static_cast<int>(faceIndex);
      if (mCachedSoftFirstFaceByPointMass[index1] < 0)
        mCachedSoftFirstFaceByPointMass[index1] = static_cast<int>(faceIndex);
      if (mCachedSoftFirstFaceByPointMass[index2] < 0)
        mCachedSoftFirstFaceByPointMass[index2] = static_cast<int>(faceIndex);
    }
  }

  if (softGeometryChanged || softTopologyChanged) {
    mCachedSoftFacesDirty = true;
    mCachedSoftFaceTopologyDirty
        = mCachedSoftFaceTopologyDirty || softTopologyChanged;
  }

  mCachedSoftBodyNodeVersion = softBodyNodeVersion;
}

//==============================================================================
void DARTCollisionObject::refreshSoftFaceCacheIfNeeded()
{
  if (!mCachedSoftFacesDirty)
    return;

  bool softFaceValidityChanged = false;
  for (std::size_t faceIndex = 0u; faceIndex < mCachedSoftFaces.size();
       ++faceIndex) {
    auto& cachedFace = mCachedSoftFaces[faceIndex];
    const bool wasValid = cachedFace.valid;
    cachedFace.valid = false;

    if ((cachedFace.indices.array() < 0).any()) {
      softFaceValidityChanged = softFaceValidityChanged || wasValid;
      continue;
    }

    const auto index0 = static_cast<std::size_t>(cachedFace.indices[0]);
    const auto index1 = static_cast<std::size_t>(cachedFace.indices[1]);
    const auto index2 = static_cast<std::size_t>(cachedFace.indices[2]);
    if (index0 >= mCachedSoftLocalVertices.size()
        || index1 >= mCachedSoftLocalVertices.size()
        || index2 >= mCachedSoftLocalVertices.size()) {
      softFaceValidityChanged = softFaceValidityChanged || wasValid;
      continue;
    }

    cachedFace.a = mCachedSoftLocalVertices[index0];
    const Eigen::Vector3d& b = mCachedSoftLocalVertices[index1];
    const Eigen::Vector3d& c = mCachedSoftLocalVertices[index2];
    cachedFace.edge0 = b - cachedFace.a;
    cachedFace.edge1 = c - cachedFace.a;

    Eigen::Vector3d normal = cachedFace.edge0.cross(cachedFace.edge1);
    const double normalNorm = normal.norm();
    if (normalNorm <= kSoftFaceCacheEps) {
      softFaceValidityChanged = softFaceValidityChanged || wasValid;
      continue;
    }

    cachedFace.normal = normal / normalNorm;
    cachedFace.d00 = cachedFace.edge0.dot(cachedFace.edge0);
    cachedFace.d01 = cachedFace.edge0.dot(cachedFace.edge1);
    cachedFace.d11 = cachedFace.edge1.dot(cachedFace.edge1);
    cachedFace.denom
        = cachedFace.d00 * cachedFace.d11 - cachedFace.d01 * cachedFace.d01;
    if (std::abs(cachedFace.denom) <= kSoftFaceCacheEps) {
      softFaceValidityChanged = softFaceValidityChanged || wasValid;
      continue;
    }

    cachedFace.planeOffset = cachedFace.normal.dot(cachedFace.a);
    cachedFace.centroid = (cachedFace.a + b + c) / 3.0;
    cachedFace.boundsMin = cachedFace.a.cwiseMin(b).cwiseMin(c);
    cachedFace.boundsMax = cachedFace.a.cwiseMax(b).cwiseMax(c);
    cachedFace.valid = true;
    softFaceValidityChanged = softFaceValidityChanged || !wasValid;
  }

  if (mCachedSoftFaceTopologyDirty || softFaceValidityChanged
      || mCachedSoftFaceBvhNodes.empty()) {
    refreshSoftFaceBvhCache();
  } else {
    refreshSoftFaceBvhBounds();
  }

  mCachedSoftFacesDirty = false;
  mCachedSoftFaceTopologyDirty = false;
}

//==============================================================================
void DARTCollisionObject::refreshSoftFaceBvhCache()
{
  mCachedSoftFaceBvhNodes.clear();

  mCachedSoftFaceBvhIndices.resize(mCachedSoftFaces.size());
  int numValidFaces = 0;
  for (std::size_t i = 0u; i < mCachedSoftFaces.size(); ++i) {
    if (mCachedSoftFaces[i].valid)
      mCachedSoftFaceBvhIndices[static_cast<std::size_t>(numValidFaces++)]
          = static_cast<int>(i);
  }
  mCachedSoftFaceBvhIndices.resize(static_cast<std::size_t>(numValidFaces));

  if (numValidFaces <= 0)
    return;

  constexpr int kLeafSize = 4;
  const auto buildNode = [&](auto&& self, int first, int count) -> int {
    const int nodeIndex = static_cast<int>(mCachedSoftFaceBvhNodes.size());
    mCachedSoftFaceBvhNodes.push_back(CachedSoftFaceBvhNode{});

    const int firstFaceIndex
        = mCachedSoftFaceBvhIndices[static_cast<std::size_t>(first)];
    const auto& firstFace
        = mCachedSoftFaces[static_cast<std::size_t>(firstFaceIndex)];
    Eigen::Vector3d boundsMin = firstFace.boundsMin;
    Eigen::Vector3d boundsMax = firstFace.boundsMax;
    Eigen::Vector3d centroidMin = firstFace.centroid;
    Eigen::Vector3d centroidMax = firstFace.centroid;

    for (int i = 1; i < count; ++i) {
      const int faceIndex
          = mCachedSoftFaceBvhIndices[static_cast<std::size_t>(first + i)];
      const auto& face = mCachedSoftFaces[static_cast<std::size_t>(faceIndex)];
      boundsMin = boundsMin.cwiseMin(face.boundsMin);
      boundsMax = boundsMax.cwiseMax(face.boundsMax);
      centroidMin = centroidMin.cwiseMin(face.centroid);
      centroidMax = centroidMax.cwiseMax(face.centroid);
    }

    mCachedSoftFaceBvhNodes[static_cast<std::size_t>(nodeIndex)].boundsMin
        = boundsMin;
    mCachedSoftFaceBvhNodes[static_cast<std::size_t>(nodeIndex)].boundsMax
        = boundsMax;

    const Eigen::Vector3d centroidExtent = centroidMax - centroidMin;
    const bool smallCentroidSpan = centroidExtent.maxCoeff() <= 1e-12;
    if (count <= kLeafSize || smallCentroidSpan) {
      auto& leaf = mCachedSoftFaceBvhNodes[static_cast<std::size_t>(nodeIndex)];
      leaf.left = -1;
      leaf.right = -1;
      leaf.first = first;
      leaf.count = count;
      return nodeIndex;
    }

    int axis = 0;
    if (centroidExtent[1] > centroidExtent[axis])
      axis = 1;
    if (centroidExtent[2] > centroidExtent[axis])
      axis = 2;

    const int mid = first + count / 2;
    std::nth_element(
        mCachedSoftFaceBvhIndices.begin() + first,
        mCachedSoftFaceBvhIndices.begin() + mid,
        mCachedSoftFaceBvhIndices.begin() + first + count,
        [this, axis](int lhsFaceIndex, int rhsFaceIndex) {
          return mCachedSoftFaces[static_cast<std::size_t>(lhsFaceIndex)]
                     .centroid[axis]
                 < mCachedSoftFaces[static_cast<std::size_t>(rhsFaceIndex)]
                       .centroid[axis];
        });

    const int left = self(self, first, mid - first);
    const int right = self(self, mid, first + count - mid);

    auto& internal
        = mCachedSoftFaceBvhNodes[static_cast<std::size_t>(nodeIndex)];
    internal.left = left;
    internal.right = right;
    internal.first = 0;
    internal.count = 0;
    return nodeIndex;
  };

  buildNode(buildNode, 0, numValidFaces);
}

//==============================================================================
void DARTCollisionObject::refreshSoftFaceBvhBounds()
{
  if (mCachedSoftFaceBvhNodes.empty() || mCachedSoftFaceBvhIndices.empty())
    return;

  const auto refreshNode = [&](auto&& self, int nodeIndex) -> bool {
    if (nodeIndex < 0
        || static_cast<std::size_t>(nodeIndex)
               >= mCachedSoftFaceBvhNodes.size()) {
      return false;
    }

    auto& node = mCachedSoftFaceBvhNodes[static_cast<std::size_t>(nodeIndex)];
    Eigen::Vector3d boundsMin
        = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d boundsMax
        = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    bool hasBounds = false;

    if (node.left < 0 && node.right < 0) {
      for (int i = 0; i < node.count; ++i) {
        const auto faceCursor = static_cast<std::size_t>(node.first + i);
        if (faceCursor >= mCachedSoftFaceBvhIndices.size())
          continue;

        const int faceIndex = mCachedSoftFaceBvhIndices[faceCursor];
        if (faceIndex < 0
            || static_cast<std::size_t>(faceIndex) >= mCachedSoftFaces.size()) {
          continue;
        }

        const auto& face
            = mCachedSoftFaces[static_cast<std::size_t>(faceIndex)];
        if (!face.valid)
          continue;

        boundsMin = boundsMin.cwiseMin(face.boundsMin);
        boundsMax = boundsMax.cwiseMax(face.boundsMax);
        hasBounds = true;
      }
    } else {
      if (self(self, node.left)) {
        const auto& left
            = mCachedSoftFaceBvhNodes[static_cast<std::size_t>(node.left)];
        boundsMin = boundsMin.cwiseMin(left.boundsMin);
        boundsMax = boundsMax.cwiseMax(left.boundsMax);
        hasBounds = true;
      }
      if (self(self, node.right)) {
        const auto& right
            = mCachedSoftFaceBvhNodes[static_cast<std::size_t>(node.right)];
        boundsMin = boundsMin.cwiseMin(right.boundsMin);
        boundsMax = boundsMax.cwiseMax(right.boundsMax);
        hasBounds = true;
      }
    }

    if (hasBounds) {
      node.boundsMin = boundsMin;
      node.boundsMax = boundsMax;
    } else {
      node.boundsMin.setZero();
      node.boundsMax.setZero();
    }

    return hasBounds;
  };

  refreshNode(refreshNode, 0);
}

//==============================================================================
void DARTCollisionObject::updateEngineData()
{
  refreshShapeCache();
}

} // namespace collision
} // namespace dart
