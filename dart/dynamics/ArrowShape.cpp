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

#include "dart/dynamics/ArrowShape.hpp"

#include "dart/math/Constants.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
ArrowShape::Properties::Properties(
    double _radius,
    double _headRadiusScale,
    double _headLengthScale,
    double _minHeadLength,
    double _maxHeadLength,
    bool _doubleArrow)
  : mRadius(_radius),
    mHeadRadiusScale(_headRadiusScale),
    mHeadLengthScale(_headLengthScale),
    mMinHeadLength(_minHeadLength),
    mMaxHeadLength(_maxHeadLength),
    mDoubleArrow(_doubleArrow)
{
}

//==============================================================================
ArrowShape::ArrowShape()
  : MeshShape(
      Eigen::Vector3d::Ones(), std::make_shared<math::TriMesh<double>>()),
    mResolution(10)
{
}

//==============================================================================
ArrowShape::ArrowShape(
    const Eigen::Vector3d& _tail,
    const Eigen::Vector3d& _head,
    const Properties& _properties,
    const Eigen::Vector4d& _color,
    std::size_t _resolution)
  : MeshShape(
      Eigen::Vector3d::Ones(), std::make_shared<math::TriMesh<double>>()),
    mTail(_tail),
    mHead(_head),
    mProperties(_properties),
    mResolution(_resolution)
{
  instantiate(_resolution);
  configureArrow(mTail, mHead, mProperties);
  setColorMode(MeshShape::COLOR_INDEX);
  notifyColorUpdated(_color);
}

//==============================================================================
ArrowShape::~ArrowShape()
{
  // No manual cleanup needed - TriMesh is managed by shared_ptr
}

//==============================================================================
void ArrowShape::setPositions(
    const Eigen::Vector3d& _tail, const Eigen::Vector3d& _head)
{
  configureArrow(_tail, _head, mProperties);
}

//==============================================================================
const Eigen::Vector3d& ArrowShape::getTail() const
{
  return mTail;
}

//==============================================================================
const Eigen::Vector3d& ArrowShape::getHead() const
{
  return mHead;
}

//==============================================================================
void ArrowShape::setProperties(const Properties& _properties)
{
  configureArrow(mTail, mHead, _properties);
}

//==============================================================================
void ArrowShape::notifyColorUpdated(const Eigen::Vector4d& _color)
{
  // Store color for future use - TriMesh doesn't directly store colors
  // Colors are typically handled by the renderer
  (void)_color; // Suppress unused parameter warning
}

//==============================================================================
const ArrowShape::Properties& ArrowShape::getProperties() const
{
  return mProperties;
}

//==============================================================================
void ArrowShape::configureArrow(
    const Eigen::Vector3d& _tail,
    const Eigen::Vector3d& _head,
    const Properties& _properties)
{
  mTail = _tail;
  mHead = _head;
  mProperties = _properties;

  mProperties.mHeadLengthScale
      = std::max(0.0, std::min(1.0, mProperties.mHeadLengthScale));
  mProperties.mMinHeadLength = std::max(0.0, mProperties.mMinHeadLength);
  mProperties.mMaxHeadLength = std::max(0.0, mProperties.mMaxHeadLength);
  mProperties.mHeadRadiusScale = std::max(1.0, mProperties.mHeadRadiusScale);

  double length = (mTail - mHead).norm();

  double minHeadLength = std::min(
      mProperties.mMinHeadLength,
      mProperties.mDoubleArrow ? length / 2.0 : length);
  double maxHeadLength = std::min(
      mProperties.mMaxHeadLength,
      mProperties.mDoubleArrow ? length / 2.0 : length);

  double headLength = mProperties.mHeadLengthScale * length;
  headLength = std::min(maxHeadLength, std::max(minHeadLength, headLength));

  // Regenerate the entire mesh with new parameters
  mTriMesh = std::make_shared<math::TriMesh<double>>();

  const double pi = math::constantsd::pi();
  const std::size_t resolution = mResolution;

  // Calculate z-coordinates for different parts
  double tailTipZ = 0.0;
  double tailBaseZ = mProperties.mDoubleArrow ? headLength : 0.0;
  double bodyStartZ = tailBaseZ;
  double bodyEndZ = length - headLength;
  double headBaseZ = bodyEndZ;
  double headTipZ = length;

  // Calculate total number of vertices and triangles
  std::size_t tailVertices
      = mProperties.mDoubleArrow ? (2 * resolution + 1) : 0;
  std::size_t bodyVertices = 2 * resolution;
  std::size_t headVertices = 2 * resolution + 1;
  std::size_t totalVertices = tailVertices + bodyVertices + headVertices;

  std::size_t tailTriangles = mProperties.mDoubleArrow ? (3 * resolution) : 0;
  std::size_t bodyTriangles = 2 * resolution;
  std::size_t headTriangles = 3 * resolution;
  std::size_t totalTriangles = tailTriangles + bodyTriangles + headTriangles;

  mTriMesh->reserveVertices(totalVertices);
  mTriMesh->reserveTriangles(totalTriangles);

  std::size_t vertexOffset = 0;

  // Generate tail (if double arrow)
  if (mProperties.mDoubleArrow) {
    // Tail cone vertices
    for (std::size_t i = 0; i < resolution; ++i) {
      double theta = (double)(i) / (double)(resolution)*2.0 * pi;
      double x = mProperties.mRadius * cos(theta);
      double y = mProperties.mRadius * sin(theta);

      // Back of tail cone (at tip)
      mTriMesh->addVertex(Eigen::Vector3d(x, y, tailTipZ));
      // Base of tail cone
      mTriMesh->addVertex(Eigen::Vector3d(
          x * mProperties.mHeadRadiusScale,
          y * mProperties.mHeadRadiusScale,
          tailBaseZ));
    }
    // Tail tip vertex
    mTriMesh->addVertex(Eigen::Vector3d(0, 0, tailTipZ));

    // Tail triangles
    for (std::size_t i = 0; i < resolution; ++i) {
      std::size_t next = (i + 1) % resolution;

      // Back face triangles
      mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
          vertexOffset + 2 * i,
          vertexOffset + 2 * i + 1,
          vertexOffset + 2 * next + 1));
      mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
          vertexOffset + 2 * i,
          vertexOffset + 2 * next + 1,
          vertexOffset + 2 * next));

      // Cone tip triangles
      mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
          vertexOffset + 2 * i + 1,
          vertexOffset + 2 * resolution,
          vertexOffset + 2 * next + 1));
    }

    vertexOffset += tailVertices;
  }

  // Generate body (cylinder)
  for (std::size_t i = 0; i < resolution; ++i) {
    double theta = (double)(i) / (double)(resolution)*2.0 * pi;
    double x = mProperties.mRadius * cos(theta);
    double y = mProperties.mRadius * sin(theta);

    mTriMesh->addVertex(Eigen::Vector3d(x, y, bodyStartZ));
    mTriMesh->addVertex(Eigen::Vector3d(x, y, bodyEndZ));
  }

  // Body triangles
  for (std::size_t i = 0; i < resolution; ++i) {
    std::size_t next = (i + 1) % resolution;

    mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
        vertexOffset + 2 * i,
        vertexOffset + 2 * next + 1,
        vertexOffset + 2 * i + 1));
    mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
        vertexOffset + 2 * i,
        vertexOffset + 2 * next,
        vertexOffset + 2 * next + 1));
  }

  vertexOffset += bodyVertices;

  // Generate head (cone)
  for (std::size_t i = 0; i < resolution; ++i) {
    double theta = (double)(i) / (double)(resolution)*2.0 * pi;
    double x = mProperties.mRadius * cos(theta);
    double y = mProperties.mRadius * sin(theta);

    // Back of head cone
    mTriMesh->addVertex(Eigen::Vector3d(x, y, headBaseZ));
    // Head widened base
    mTriMesh->addVertex(Eigen::Vector3d(
        x * mProperties.mHeadRadiusScale,
        y * mProperties.mHeadRadiusScale,
        headBaseZ));
  }
  // Head tip vertex
  mTriMesh->addVertex(Eigen::Vector3d(0, 0, headTipZ));

  // Head triangles
  for (std::size_t i = 0; i < resolution; ++i) {
    std::size_t next = (i + 1) % resolution;

    // Back face triangles
    mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
        vertexOffset + 2 * i,
        vertexOffset + 2 * next + 1,
        vertexOffset + 2 * i + 1));
    mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
        vertexOffset + 2 * i,
        vertexOffset + 2 * next,
        vertexOffset + 2 * next + 1));

    // Cone tip triangles
    mTriMesh->addTriangle(math::TriMesh<double>::Triangle(
        vertexOffset + 2 * i + 1,
        vertexOffset + 2 * next + 1,
        vertexOffset + 2 * resolution));
  }

  // Apply transformation to orient arrow from tail to head
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = mTail;
  Eigen::Vector3d v = mHead - mTail;
  Eigen::Vector3d z = Eigen::Vector3d::UnitZ();

  if (v.norm() > 0) {
    v.normalize();
    Eigen::Vector3d axis = z.cross(v);
    if (axis.norm() > 0) {
      axis.normalize();
      tf.rotate(Eigen::AngleAxisd(acos(z.dot(v)), axis));
    } else {
      // v is parallel or antiparallel to z
      if (z.dot(v) < 0) {
        // v is antiparallel to z, rotate 180 degrees around any perpendicular
        // axis
        tf.rotate(Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitY()));
      }
      // If v is parallel to z, no rotation needed (identity)
    }
  }

  // Transform all vertices - need to get non-const access
  auto& vertices
      = const_cast<std::vector<Eigen::Vector3d>&>(mTriMesh->getVertices());
  for (auto& vertex : vertices) {
    vertex = tf * vertex;
  }

  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
ShapePtr ArrowShape::clone() const
{
  auto new_shape = std::make_shared<ArrowShape>();

  new_shape->mTail = mTail;
  new_shape->mHead = mHead;
  new_shape->mProperties = mProperties;
  new_shape->mResolution = mResolution;

  // Clone the TriMesh
  new_shape->mTriMesh = std::make_shared<math::TriMesh<double>>(*mTriMesh);

  new_shape->mMeshUri = mMeshUri;
  new_shape->mMeshPath = mMeshPath;
  new_shape->mResourceRetriever = mResourceRetriever;
  new_shape->mDisplayList = mDisplayList;
  new_shape->mScale = mScale;
  new_shape->mColorMode = mColorMode;
  new_shape->mAlphaMode = mAlphaMode;
  new_shape->mColorIndex = mColorIndex;

  return new_shape;
}

//==============================================================================
void ArrowShape::instantiate(std::size_t resolution)
{
  mResolution = resolution;
  // Initial mesh will be generated by configureArrow
}

} // namespace dynamics
} // namespace dart
