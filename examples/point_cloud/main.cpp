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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#if DART_HAVE_OCTOMAP
  #include <dart/dynamics/voxel_grid_shape.hpp>
#endif

#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

namespace {

dart::dynamics::SkeletonPtr createStaticVisualSkeleton(
    const std::string& name,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double alpha = 1.0)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>()
                   .second;
  auto* shapeNode
      = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  shapeNode->setRelativeTranslation(position);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(color.x(), color.y(), color.z(), alpha));
  return skeleton;
}

std::shared_ptr<dart::dynamics::PointCloudShape> createPointCloudShape()
{
  auto shape = std::make_shared<dart::dynamics::PointCloudShape>(0.055);
  shape->setPointShapeType(dart::dynamics::PointCloudShape::BOX);
  shape->setColorMode(dart::dynamics::PointCloudShape::BIND_PER_POINT);

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector4d> colors;
  points.reserve(48);
  colors.reserve(48);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 6; ++y) {
      const double xf = -0.42 + 0.12 * static_cast<double>(x);
      const double yf = -0.30 + 0.12 * static_cast<double>(y);
      const double zf = 0.05 * std::sin(5.0 * xf) + 0.06 * std::cos(4.0 * yf);
      points.emplace_back(xf, yf, zf);

      const double mix = static_cast<double>(x + y) / 12.0;
      colors.emplace_back(0.18 + 0.70 * mix, 0.28, 0.92 - 0.55 * mix, 0.82);
    }
  }
  shape->addPoint(points);
  shape->setColors(colors);
  return shape;
}

dart::simulation::WorldPtr createPointCloudWorld()
{
  auto world = dart::simulation::World::create("dartsim_point_cloud");
  world->setGravity(Eigen::Vector3d::Zero());

  world->addSkeleton(createStaticVisualSkeleton(
      "visual_point_cloud",
      createPointCloudShape(),
      Eigen::Vector3d(-0.38, -0.05, 0.55),
      Eigen::Vector3d(0.25, 0.48, 0.95),
      0.82));

#if DART_HAVE_OCTOMAP
  auto voxelGridShape = std::make_shared<dart::dynamics::VoxelGridShape>(0.11);
  const std::array<Eigen::Vector3d, 9> voxelCenters{{
      Eigen::Vector3d(-0.22, -0.10, 0.00),
      Eigen::Vector3d(-0.11, -0.10, 0.00),
      Eigen::Vector3d(0.00, -0.10, 0.00),
      Eigen::Vector3d(0.11, -0.10, 0.00),
      Eigen::Vector3d(-0.11, 0.01, 0.11),
      Eigen::Vector3d(0.00, 0.01, 0.11),
      Eigen::Vector3d(0.11, 0.01, 0.11),
      Eigen::Vector3d(0.00, 0.12, 0.22),
      Eigen::Vector3d(0.11, 0.12, 0.22),
  }};
  for (const Eigen::Vector3d& voxel : voxelCenters) {
    voxelGridShape->updateOccupancy(voxel);
  }
  world->addSkeleton(createStaticVisualSkeleton(
      "visual_voxel_grid",
      voxelGridShape,
      Eigen::Vector3d(0.45, 0.06, 0.48),
      Eigen::Vector3d(0.94, 0.52, 0.20),
      0.72));
#endif

  Eigen::Isometry3d sensorTransform = Eigen::Isometry3d::Identity();
  sensorTransform.translation() = Eigen::Vector3d(0.72, -0.42, 0.76);
  auto sensor = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "point_cloud_sensor", sensorTransform);
  sensor->setShape(std::make_shared<dart::dynamics::SphereShape>(0.055));
  sensor->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.95, 0.18, 0.12, 1.0));
  world->addSimpleFrame(sensor);

  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createPointCloudWorld();
  return dart::gui::runApplication(argc, argv, options);
}
