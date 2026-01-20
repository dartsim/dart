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

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <iostream>

int main(int argc, char** argv)
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

  std::cout << "\n--- AABB Visualization ---\n";
  collision::Aabb boxAabb
      = collision::Aabb::forBox(Eigen::Vector3d(0.25, 0.25, 0.25));
  collision::Aabb worldAabb = collision::Aabb::transformed(
      boxAabb,
      Eigen::Translation3d(-1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());
  builder.addAabb(worldAabb, vsg::colors::Orange);
  std::cout << "Added AABB for box at (-1.5, 0, 0)\n";

  collision::Aabb sphereAabb = collision::Aabb::forSphere(0.4);
  collision::Aabb worldSphereAabb = collision::Aabb::transformed(
      sphereAabb,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
  builder.addAabb(worldSphereAabb, vsg::colors::Orange);
  std::cout << "Added AABB for sphere at origin\n";

  std::cout << "\n--- Distance Query Visualization ---\n";
  collision::DistanceResult distResult;
  distResult.distance = 1.0;
  distResult.pointOnObject1 = Eigen::Vector3d(-1.5, 0.5, 0.25);
  distResult.pointOnObject2 = Eigen::Vector3d(0.0, 0.5, 0.4);
  distResult.normal
      = (distResult.pointOnObject2 - distResult.pointOnObject1).normalized();
  builder.addDistanceResult(distResult);
  std::cout
      << "Added distance visualization between box and sphere (top edge)\n";

  std::cout << "\n--- Raycast Visualization ---\n";
  collision::Ray ray(
      Eigen::Vector3d(-3.0, -1.5, 1.0),
      Eigen::Vector3d(1.0, 0.3, -0.2).normalized(),
      10.0);
  collision::RaycastResult rayHit;
  rayHit.hit = true;
  rayHit.distance = 1.8;
  rayHit.point = Eigen::Vector3d(-1.25, -0.9, 0.64);
  rayHit.normal = Eigen::Vector3d(-1.0, 0.0, 0.0).normalized();
  builder.addRaycast(ray, &rayHit);
  std::cout << "Added raycast from (-3, -1.5, 1) hitting box\n";

  collision::Ray missRay(
      Eigen::Vector3d(2.0, 2.0, 1.0),
      Eigen::Vector3d(0.0, 1.0, 0.0).normalized(),
      3.0);
  builder.addRaycast(missRay, nullptr, vsg::colors::Gray);
  std::cout << "Added raycast that misses (gray)\n";

  auto scene = builder.build();
  if (auto group = scene.cast<::vsg::Group>()) {
    std::cout << "Built scene with " << group->children.size() << " children"
              << std::endl;
  } else {
    std::cout << "Built scene (not a group)" << std::endl;
  }

  ::vsg::ComputeBounds computeBounds;
  scene->accept(computeBounds);
  auto& bounds = computeBounds.bounds;
  std::cout << "Scene bounds: min(" << bounds.min.x << ", " << bounds.min.y
            << ", " << bounds.min.z << ") max(" << bounds.max.x << ", "
            << bounds.max.y << ", " << bounds.max.z << ")" << std::endl;

  bool headlessMode = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--headless") {
      headlessMode = true;
    }
  }

  if (headlessMode) {
    std::cout << "\nRunning in headless mode...\n";

    auto viewer = vsg::SimpleViewer::headless(1280, 720);
    viewer.setScene(scene);
    viewer.addGrid(6.0, 1.0);
    viewer.addAxes(1.0);
    viewer.lookAt(
        Eigen::Vector3d(5.0, 5.0, 4.0),
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d::UnitZ());

    viewer.run();

    if (viewer.saveScreenshot("collision_viz_headless.ppm")) {
      std::cout << "Screenshot saved to collision_viz_headless.ppm\n";
    } else {
      std::cerr << "Failed to save screenshot\n";
    }

    auto buffer = viewer.captureBuffer();
    std::cout << "Captured buffer: " << buffer.size() << " bytes (" << 1280
              << "x" << 720 << " RGBA)\n";
  } else {
    std::cout << "\nStarting VSG viewer...\n";
    std::cout << "Use mouse to rotate/zoom, close window to exit.\n";
    std::cout << "(Run with --headless for headless mode)\n\n";

    vsg::SimpleViewer viewer(1280, 720, "DART Collision Visualization");
    viewer.setScene(scene);
    viewer.addGrid(6.0, 1.0);
    viewer.addAxes(1.0);
    viewer.lookAt(
        Eigen::Vector3d(5.0, 5.0, 4.0),
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d::UnitZ());

    viewer.run();
  }

  std::cout << "Done.\n";
  return 0;
}
