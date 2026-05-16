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

#include <dart/gui/application.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

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

std::shared_ptr<dart::dynamics::HeightmapShaped> createHeightmapExampleShape()
{
  auto shape = std::make_shared<dart::dynamics::HeightmapShaped>();
  constexpr std::size_t xResolution = 7;
  constexpr std::size_t yResolution = 5;
  std::vector<double> heights;
  heights.reserve(xResolution * yResolution);
  for (std::size_t y = 0; y < yResolution; ++y) {
    for (std::size_t x = 0; x < xResolution; ++x) {
      const double xPhase = static_cast<double>(x) * 0.75;
      const double yPhase = static_cast<double>(y) * 0.9;
      const double ridge = (x == 3 || y == 2) ? 0.08 : 0.0;
      heights.push_back(
          0.08 + ridge + 0.08 * std::sin(xPhase) * std::cos(yPhase));
    }
  }
  shape->setHeightField(xResolution, yResolution, heights);
  shape->setScale(Eigen::Vector3d(0.32, 0.32, 1.0));
  return shape;
}

dart::simulation::WorldPtr createHeightmapWorld()
{
  auto world = dart::simulation::World::create("dartsim_heightmap");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createStaticVisualSkeleton(
      "visual_heightmap",
      createHeightmapExampleShape(),
      Eigen::Vector3d(-0.25, 0.0, 0.0),
      Eigen::Vector3d(0.24, 0.58, 0.88)));
  world->addSkeleton(createStaticVisualSkeleton(
      "visual_heightmap_reference_box",
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.48, 0.48, 0.28)),
      Eigen::Vector3d(0.72, 0.0, 0.20),
      Eigen::Vector3d(0.20, 0.72, 0.28),
      0.48));

  int index = 0;
  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      world->addSkeleton(createStaticVisualSkeleton(
          "visual_heightmap_sample_ball_" + std::to_string(index++),
          std::make_shared<dart::dynamics::SphereShape>(0.07),
          Eigen::Vector3d(
              -0.25 + static_cast<double>(x) * 0.42,
              static_cast<double>(y) * 0.32,
              0.45),
          Eigen::Vector3d(0.92, 0.48, 0.16)));
    }
  }
  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createHeightmapWorld();
  return dart::gui::runApplication(argc, argv, options);
}
