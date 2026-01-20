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

#include <dart/gui/vsg/CollisionSceneBuilder.hpp>
#include <dart/gui/vsg/SimpleViewer.hpp>

#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <iostream>

int main()
{
  namespace collision = dart::collision::experimental;
  namespace vsg = dart::gui::vsg;

  std::cout << "DART Collision Visualization Example\n";
  std::cout << "=====================================\n\n";

  collision::CollisionWorld world;

  auto box = world.createObject(
      std::make_unique<collision::BoxShape>(Eigen::Vector3d(0.5, 0.5, 0.5)));
  box.setTransform(
      Eigen::Translation3d(-1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());

  auto sphere
      = world.createObject(std::make_unique<collision::SphereShape>(0.4));
  sphere.setTransform(
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());

  auto capsule
      = world.createObject(std::make_unique<collision::CapsuleShape>(0.3, 0.8));
  capsule.setTransform(
      Eigen::Translation3d(1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());

  auto cylinder = world.createObject(
      std::make_unique<collision::CylinderShape>(0.35, 0.7));
  cylinder.setTransform(
      Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());

  auto collidingSphere
      = world.createObject(std::make_unique<collision::SphereShape>(0.5));
  collidingSphere.setTransform(
      Eigen::Translation3d(0.3, 0.0, 0.0) * Eigen::Isometry3d::Identity());

  collision::CollisionResult result;
  collision::CollisionOption option;
  option.enableContact = true;

  world.collide(option, result);

  std::cout << "Collision result: " << (result.isCollision() ? "HIT" : "NO HIT")
            << "\n";
  std::cout << "Number of contacts: " << result.numContacts() << "\n\n";

  vsg::CollisionSceneBuilder builder;

  builder.addObject(box, vsg::colors::Blue);
  builder.addObject(sphere, vsg::colors::Green);
  builder.addObject(capsule, vsg::colors::Yellow);
  builder.addObject(cylinder, vsg::colors::Cyan);
  builder.addObject(collidingSphere, vsg::colors::Magenta);

  builder.addContacts(result, 0.2, 0.03);

  builder.addSphereCast(
      Eigen::Vector3d(-3.0, 2.0, 0.5),
      Eigen::Vector3d(3.0, 2.0, 0.5),
      0.2,
      nullptr);

  std::cout << "Starting VSG viewer...\n";
  std::cout << "Use mouse to rotate/zoom, close window to exit.\n\n";

  vsg::SimpleViewer viewer(1280, 720, "DART Collision Visualization");
  viewer.addGrid(6.0, 1.0);
  viewer.addAxes(1.0);
  viewer.setScene(builder.build());
  viewer.lookAt(
      Eigen::Vector3d(5.0, 5.0, 4.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());

  viewer.run();

  std::cout << "Done.\n";
  return 0;
}
