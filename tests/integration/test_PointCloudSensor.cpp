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

#include "demo_models/PointCloudSensor.hpp"

#if HAVE_OCTOMAP

  #include <dart/dynamics/BodyNode.hpp>
  #include <dart/dynamics/BoxShape.hpp>
  #include <dart/dynamics/MeshShape.hpp>
  #include <dart/dynamics/RevoluteJoint.hpp>

  #include <dart/math/Random.hpp>
  #include <dart/math/TriMesh.hpp>

  #include <gtest/gtest.h>

  #include <limits>
  #include <stdexcept>

  #include <cmath>
  #include <cstdlib>

namespace {

struct Model
{
  dart::dynamics::SkeletonPtr robot = dart::dynamics::Skeleton::create();
  dart::dynamics::SimpleFramePtr frame
      = dart::dynamics::SimpleFrame::createShared(
          dart::dynamics::Frame::World());
  std::shared_ptr<dart::dynamics::PointCloudShape> points
      = std::make_shared<dart::dynamics::PointCloudShape>();
  std::shared_ptr<dart::dynamics::VoxelGridShape> voxels
      = std::make_shared<dart::dynamics::VoxelGridShape>(0.05);
  dart_demos::PointCloudSensor sensor{robot, frame, points, voxels};
  dart::dynamics::ShapeNode* shape = nullptr;

  explicit Model(bool mesh = true)
  {
    auto* body
        = robot->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>()
              .second;
    if (mesh) {
      auto triangle = std::make_shared<dart::math::TriMesh<double>>();
      triangle->addVertex(0.117, 0.123, 0.147);
      triangle->addVertex(0.237, 0.231, 0.253);
      triangle->addVertex(0.347, 0.343, 0.359);
      triangle->addTriangle(0, 1, 2);
      shape = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
          std::make_shared<dart::dynamics::MeshShape>(
              Eigen::Vector3d::Ones(), triangle));
    } else {
      shape = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
          std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones()));
    }
    shape->setRelativeTranslation(Eigen::Vector3d(0.414, 0.028, 0.731));
  }
};

void seed()
{
  std::srand(42);
  dart::math::Random::setSeed(42);
}

} // namespace

TEST(PointCloudSensorTest, SamplesMeshVerticesAndUpdatesColorsAndOccupancy)
{
  seed();
  Model model;
  model.sensor.update(0.02);
  ASSERT_EQ(model.points->getNumPoints(), 500);
  ASSERT_EQ(model.points->getColors().size(), 500);
  EXPECT_NE(model.robot->getPosition(0), 0.0);
  EXPECT_TRUE(model.frame->getWorldTransform().translation().isApprox(
      Eigen::Vector3d(0, 1, 0.5)));
  const Eigen::Vector3d vertices[]
      = {Eigen::Vector3d(0.117, 0.123, 0.147),
         Eigen::Vector3d(0.237, 0.231, 0.253),
         Eigen::Vector3d(0.347, 0.343, 0.359)};
  bool hasOccupiedEndpoint = false;
  for (const auto& point : model.points->getPoints()) {
    bool matched = false;
    for (const auto& vertex : vertices)
      matched
          |= (point - model.shape->getWorldTransform() * vertex).norm() < 1e-6;
    EXPECT_TRUE(matched);
    hasOccupiedEndpoint |= model.voxels->getOccupancy(point) > 0.5;
  }
  EXPECT_TRUE(hasOccupiedEndpoint);
  for (const auto& color : model.points->getColors()) {
    EXPECT_TRUE(color.allFinite());
    EXPECT_GE(color.head<3>().minCoeff(), 0.1 - 1e-7);
    EXPECT_LE(color.head<3>().maxCoeff(), 0.9 + 1e-7);
    EXPECT_DOUBLE_EQ(color[3], 0.75);
  }
  model.sensor.update(0.02);
  EXPECT_TRUE(
      model.frame->getWorldTransform().translation().isApprox(Eigen::Vector3d(
          std::sin(0.02), std::cos(0.02), 0.5 + 0.25 * std::sin(0.04))));
}

TEST(PointCloudSensorTest, PausingPreservesPointsPoseAndOrbit)
{
  Model model;
  model.sensor.update(0.02);
  const auto points = model.points->getPoints();
  const auto positions = model.robot->getPositions();
  const auto origin = model.frame->getWorldTransform().translation().eval();
  const auto size = model.voxels->getOctree()->size();
  model.sensor.updating = false;
  model.sensor.update(0.4);
  EXPECT_EQ(model.points->getPoints(), points);
  EXPECT_TRUE(model.robot->getPositions().isApprox(positions));
  EXPECT_TRUE(model.frame->getWorldTransform().translation().isApprox(origin));
  EXPECT_EQ(model.voxels->getOctree()->size(), size);
  model.sensor.updating = true;
  model.sensor.update(0.02);
  EXPECT_NEAR(
      model.frame->getWorldTransform().translation()[0], std::sin(0.02), 1e-15);
}

TEST(PointCloudSensorTest, SeededSamplingAndJointMotionRepeat)
{
  for (auto mode :
       {dart_demos::PointCloudSensor::SamplingMode::SampleOnRobot,
        dart_demos::PointCloudSensor::SamplingMode::SampleInBox}) {
    seed();
    Model first;
    first.sensor.samplingMode = mode;
    first.sensor.update(0.001);
    seed();
    Model second;
    second.sensor.samplingMode = mode;
    second.sensor.update(0.001);
    EXPECT_TRUE(first.robot->getPositions().isApprox(
        second.robot->getPositions(), 0.0));
    EXPECT_EQ(first.points->getPoints(), second.points->getPoints());
    EXPECT_EQ(first.points->getColors(), second.points->getColors());
    EXPECT_EQ(
        first.voxels->getOctree()->size(), second.voxels->getOctree()->size());
  }
}

TEST(PointCloudSensorTest, BoxSamplingAndMissingMeshFallbackAreBounded)
{
  for (auto mode :
       {dart_demos::PointCloudSensor::SamplingMode::SampleOnRobot,
        dart_demos::PointCloudSensor::SamplingMode::SampleInBox}) {
    Model model(false);
    model.sensor.samplingMode = mode;
    model.sensor.update(0.001);
    ASSERT_EQ(model.points->getNumPoints(), 500);
    for (const auto& point : model.points->getPoints()) {
      EXPECT_GE(point.minCoeff(), -0.5);
      EXPECT_LE(point.maxCoeff(), 0.5);
    }
  }
}

TEST(PointCloudSensorTest, EmptyRobotUsesBoxFallback)
{
  auto robot = dart::dynamics::Skeleton::create();
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  auto points = std::make_shared<dart::dynamics::PointCloudShape>();
  auto voxels = std::make_shared<dart::dynamics::VoxelGridShape>(0.05);
  dart_demos::PointCloudSensor sensor(robot, frame, points, voxels);

  sensor.update(0.001);
  ASSERT_EQ(points->getNumPoints(), 500);
  for (const auto& point : points->getPoints()) {
    EXPECT_GE(point.minCoeff(), -0.5);
    EXPECT_LE(point.maxCoeff(), 0.5);
  }
}

TEST(
    PointCloudSensorTest,
    RejectsUnaddressablePointsAndOversizedRaysBeforeInsertion)
{
  for (const double resolution : {1e-6, 0.05, 3.1e-5}) {
    Model model;
    if (resolution == 0.05)
      model.shape->setRelativeTranslation(Eigen::Vector3d::Constant(1e6));
    else if (resolution == 3.1e-5)
      model.shape->setRelativeTranslation(Eigen::Vector3d::Constant(-1.1));
    auto voxels = std::make_shared<dart::dynamics::VoxelGridShape>(resolution);
    dart_demos::PointCloudSensor sensor(
        model.robot, model.frame, model.points, voxels);
    const auto origin = model.frame->getWorldTransform().translation().eval();
    EXPECT_THROW(sensor.update(0.001), std::invalid_argument);
    EXPECT_EQ(voxels->getOctree()->size(), 0);
    EXPECT_EQ(model.points->getNumPoints(), 0);
    EXPECT_EQ(model.frame->getWorldTransform().translation(), origin);
  }
}

TEST(PointCloudSensorTest, RejectsNullOwnersAndInvalidTimesteps)
{
  Model model;
  EXPECT_THROW(
      (dart_demos::PointCloudSensor{
          nullptr, model.frame, model.points, model.voxels}),
      std::invalid_argument);
  EXPECT_THROW(
      (dart_demos::PointCloudSensor{
          model.robot, nullptr, model.points, model.voxels}),
      std::invalid_argument);
  EXPECT_THROW(
      (dart_demos::PointCloudSensor{
          model.robot, model.frame, nullptr, model.voxels}),
      std::invalid_argument);
  EXPECT_THROW(
      (dart_demos::PointCloudSensor{
          model.robot, model.frame, model.points, nullptr}),
      std::invalid_argument);
  for (double step :
       {0.0,
        -0.1,
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(model.sensor.update(step), std::invalid_argument);
  EXPECT_EQ(model.points->getNumPoints(), 0);
}

#endif // HAVE_OCTOMAP
