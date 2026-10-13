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

#include "PointCloudSensor.hpp"

#if HAVE_OCTOMAP

  #include <dart/dynamics/BodyNode.hpp>
  #include <dart/dynamics/MeshShape.hpp>

  #include <dart/math/Helpers.hpp>
  #include <dart/math/Random.hpp>

  #include <dart/common/Deprecated.hpp>

  #include <algorithm>
  #include <limits>
  #include <stdexcept>
  #include <utility>

  #include <cmath>

namespace dart_demos {
namespace {

using dart::dynamics::SkeletonPtr;

//==============================================================================
void validateSensorCloud(
    const dart::dynamics::VoxelGridShape& grid,
    const octomap::Pointcloud& cloud,
    const Eigen::Vector3d& origin)
{
  const auto octree = grid.getOctree();
  const double factor = 1.0 / octree->getResolution();
  const double maxKey = std::ldexp(1.0, octree->getTreeDepth() - 1);
  const auto keys = [factor, maxKey](const Eigen::Vector3d& point) {
    const Eigen::Vector3d used = point.cast<float>().cast<double>();
    if (!used.allFinite())
      throw std::invalid_argument("sensor point must be finite");
    Eigen::Vector3d result;
    for (int i = 0; i < 3; ++i) {
      result[i] = std::floor(factor * used[i]);
      if (!(result[i] >= -maxKey && result[i] < maxKey))
        throw std::invalid_argument("sensor point lies outside the voxel grid");
    }
    return result;
  };
  const Eigen::Vector3d originKeys = keys(origin);
  static const auto maxKeys = octomap::KeyRay().sizeMax();
  for (const auto& point : cloud) {
    const Eigen::Vector3d endpoint(point.x(), point.y(), point.z());
    const double cells = (keys(endpoint) - originKeys).cwiseAbs().sum();
    // OctoMap traces rays into a fixed key buffer and normalizes in float.
    if (cells + 4.0 > static_cast<double>(maxKeys))
      throw std::invalid_argument(
          "sensor ray crosses too many voxels for OctoMap");
    if (cells > 0.0
        && !((endpoint.cast<float>() - origin.cast<float>()).norm() > 0.0f))
      throw std::invalid_argument(
          "sensor ray is too short for OctoMap float arithmetic");
  }
}

//==============================================================================
/// Rejection-samples visual-mesh vertices with the scene's bounded retry cap;
/// a robot with no visual meshes falls back to box samples instead of hanging.
octomap::Pointcloud generatePointCloudOnRobot(
    const SkeletonPtr& robot, std::size_t numPoints)
{
  octomap::Pointcloud pointCloud;
  pointCloud.reserve(numPoints);

  const auto numBodies = robot->getNumBodyNodes();
  if (numBodies == 0)
    return pointCloud;

  constexpr std::size_t kMaxAttempts = 200000;
  std::size_t attempts = 0;

  while (pointCloud.size() < numPoints && attempts < kMaxAttempts) {
    ++attempts;

    const auto bodyIndex
        = dart::math::Random::uniform<std::size_t>(0, numBodies - 1);
    auto* body = robot->getBodyNode(bodyIndex);
    const auto numShapeNodes
        = body->getNumShapeNodesWith<dart::dynamics::VisualAspect>();
    if (numShapeNodes == 0)
      continue;

    const auto shapeIndex
        = dart::math::Random::uniform<std::size_t>(0, numShapeNodes - 1);
    auto* shapeNode
        = body->getShapeNodeWith<dart::dynamics::VisualAspect>(shapeIndex);
    auto shape = shapeNode->getShape();
    if (!shape || !shape->is<dart::dynamics::MeshShape>())
      continue;
    auto mesh = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);

    DART_SUPPRESS_DEPRECATED_BEGIN
    auto* assimpScene = mesh->getMesh();
    DART_SUPPRESS_DEPRECATED_END
    if (!assimpScene || assimpScene->mNumMeshes < 1)
      continue;

    const auto meshIndex = dart::math::Random::uniform<unsigned int>(
        0, assimpScene->mNumMeshes - 1);
    auto* assimpMesh = assimpScene->mMeshes[meshIndex];
    const auto numVertices = assimpMesh->mNumVertices;
    if (numVertices == 0)
      continue;

    const auto vertexIndex
        = dart::math::Random::uniform<unsigned int>(0, numVertices - 1);
    const auto& vertex = assimpMesh->mVertices[vertexIndex];

    const Eigen::Isometry3d tf = shapeNode->getWorldTransform();
    Eigen::Vector3d eigenVertex
        = tf * Eigen::Vector3f(vertex.x, vertex.y, vertex.z).cast<double>();

    pointCloud.push_back(
        static_cast<float>(eigenVertex.x()),
        static_cast<float>(eigenVertex.y()),
        static_cast<float>(eigenVertex.z()));
  }

  // If the robot has no mesh-shaped visual shape nodes and the attempt cap is
  // exhausted, fill the remainder with uniform in-box samples rather than
  // returning a short cloud.
  const Eigen::Vector3d boxMin = Eigen::Vector3d::Constant(-0.5);
  const Eigen::Vector3d boxMax = Eigen::Vector3d::Constant(0.5);
  while (pointCloud.size() < numPoints) {
    const Eigen::Vector3d point = dart::math::Random::uniform(boxMin, boxMax);
    pointCloud.push_back(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z()));
  }

  return pointCloud;
}

//==============================================================================
octomap::Pointcloud generatePointCloudInBox(
    std::size_t numPoints,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max)
{
  octomap::Pointcloud pointCloud;
  pointCloud.reserve(numPoints);

  for (std::size_t i = 0; i < numPoints; ++i) {
    const Eigen::Vector3d point = dart::math::Random::uniform(min, max);
    pointCloud.push_back(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z()));
  }

  return pointCloud;
}

//==============================================================================
/// Z-height red-blue gradient, alpha 0.75, clipped to [0.1, 0.9]. `maxZ`'s
/// std::numeric_limits<double>::min() start (a tiny positive value rather
/// than the most negative double) is a pre-existing quirk kept byte-for-byte
/// from the original.
std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
generatePointCloudColors(
    const dart::dynamics::PointCloudShape& pointCloudShape,
    const octomap::Pointcloud& pointCloud)
{
  const auto& points = pointCloudShape.getPoints();
  double minZ = std::numeric_limits<double>::max();
  double maxZ = std::numeric_limits<double>::min();
  for (const auto& point : points) {
    minZ = std::min(minZ, point.z());
    maxZ = std::max(maxZ, point.z());
  }
  const double diffZ
      = std::max(std::abs(maxZ - minZ), std::numeric_limits<double>::min());

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      colors;
  colors.reserve(pointCloud.size());
  for (const auto& point : pointCloud) {
    float r
        = (point.z() - static_cast<float>(minZ)) / static_cast<float>(diffZ);
    float g = 0.0f;
    float b = 1.f - r;
    r = dart::math::clip(r, 0.1f, 0.9f);
    g = dart::math::clip(g, 0.1f, 0.9f);
    b = dart::math::clip(b, 0.1f, 0.9f);
    colors.emplace_back(Eigen::Vector4f(r, g, b, 0.75f).cast<double>());
  }

  return colors;
}

} // namespace

//==============================================================================
PointCloudSensor::PointCloudSensor(
    dart::dynamics::SkeletonPtr robot,
    dart::dynamics::SimpleFramePtr sensorFrame,
    std::shared_ptr<dart::dynamics::PointCloudShape> pointCloud,
    std::shared_ptr<dart::dynamics::VoxelGridShape> voxelGrid)
  : mRobot(std::move(robot)),
    mSensorFrame(std::move(sensorFrame)),
    mPointCloud(std::move(pointCloud)),
    mVoxelGrid(std::move(voxelGrid))
{
  if (!mRobot || !mSensorFrame || !mPointCloud || !mVoxelGrid)
    throw std::invalid_argument(
        "robot, sensor frame, point cloud and voxel grid must not be null");
  const double resolution = mVoxelGrid->getOctree()->getResolution();
  if (!(resolution >= 1e-6 && resolution <= 1e6))
    throw std::invalid_argument("voxel resolution must be within [1e-6, 1e6]");
}

//==============================================================================
void PointCloudSensor::update(double timeStep)
{
  if (!std::isfinite(timeStep) || timeStep <= 0.0)
    throw std::invalid_argument("time step must be positive and finite");
  if (!updating)
    return;

  Eigen::VectorXd pos = mRobot->getPositions();
  // Deviation: joint motion uses DART's seeded generator instead of std::rand.
  pos += 0.01
         * dart::math::Random::uniform<Eigen::VectorXd>(pos.size(), -1.0, 1.0);
  mRobot->setPositions(pos);

  constexpr std::size_t kNumPoints = 500;
  octomap::Pointcloud pointCloud;
  switch (samplingMode) {
    case SamplingMode::SampleOnRobot:
      pointCloud = generatePointCloudOnRobot(mRobot, kNumPoints);
      break;
    case SamplingMode::SampleInBox:
      pointCloud = generatePointCloudInBox(
          kNumPoints,
          Eigen::Vector3d::Constant(-0.5),
          Eigen::Vector3d::Constant(0.5));
      break;
  }

  constexpr double radius = 1.0;
  const Eigen::Vector3d center(0.0, 0.1, 0.0);
  Eigen::Vector3d sensorPos = center;
  sensorPos[0] = radius * std::sin(mOrbitTime);
  sensorPos[1] = radius * std::cos(mOrbitTime);
  sensorPos[2] = 0.5 + 0.25 * std::sin(mOrbitTime * 2.0);
  validateSensorCloud(*mVoxelGrid, pointCloud, sensorPos);
  // The orbit advances with the live timestep, as the C++ demo host does.
  mOrbitTime += timeStep;
  mSensorFrame->setTranslation(sensorPos);

  mPointCloud->setPoints(pointCloud);
  mPointCloud->setColors(generatePointCloudColors(*mPointCloud, pointCloud));
  mVoxelGrid->updateOccupancy(pointCloud, sensorPos);
}

//==============================================================================
const dart::dynamics::SkeletonPtr& PointCloudSensor::getRobot() const
{
  return mRobot;
}

//==============================================================================
const dart::dynamics::SimpleFramePtr& PointCloudSensor::getSensorFrame() const
{
  return mSensorFrame;
}

//==============================================================================
const std::shared_ptr<dart::dynamics::PointCloudShape>&
PointCloudSensor::getPointCloud() const
{
  return mPointCloud;
}

//==============================================================================
const std::shared_ptr<dart::dynamics::VoxelGridShape>&
PointCloudSensor::getVoxelGrid() const
{
  return mVoxelGrid;
}

} // namespace dart_demos

#endif // HAVE_OCTOMAP
