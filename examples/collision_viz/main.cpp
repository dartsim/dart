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

void printHelp()
{
  std::cout << R"(
DART Collision Visualization Example
=====================================

Usage: collision_viz [OPTIONS]

Options:
  --headless        Run in headless mode (saves screenshot)
  -o, --output FILE Output filename (default: collision_viz.ppm)
  -W, --width N     Window/image width (default: 1280)
  -H, --height N    Window/image height (default: 720)
  -h, --help        Show this help message

Features Demonstrated:
  - Primitive shapes: Box, Sphere, Capsule, Cylinder
  - Contact points (red) and normals (yellow arrows)
  - AABB wireframes (orange outlines)
  - Distance queries (magenta points + cyan line)
  - Raycasts (cyan ray + red hit point)
  - Sphere-cast / CCD trajectories (cyan wireframe spheres)
  - Ground grid and RGB coordinate axes

Controls (windowed mode):
  Left mouse: Rotate | Right mouse: Pan | Scroll: Zoom
)";
}

int main(int argc, char** argv)
{
  namespace collision = dart::collision::experimental;
  namespace vsg = dart::gui::vsg;

  bool headlessMode = false;
  std::string outputFile = "collision_viz.ppm";
  int width = 1280;
  int height = 720;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--headless") {
      headlessMode = true;
    } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
      outputFile = argv[++i];
    } else if ((arg == "-W" || arg == "--width") && i + 1 < argc) {
      width = std::stoi(argv[++i]);
    } else if ((arg == "-H" || arg == "--height") && i + 1 < argc) {
      height = std::stoi(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      printHelp();
      return 0;
    }
  }

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
      = world.createObject(std::make_unique<collision::SphereShape>(0.35));
  collidingSphere.setTransform(
      Eigen::Translation3d(0.0, 0.6, 0.0) * Eigen::Isometry3d::Identity());

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

  builder.addContacts(result, 0.3, 0.05);
  if (result.numContacts() > 0) {
    std::cout << "Contact visualization: " << result.numContacts()
              << " contact point(s) with 0.3m normal arrows\n";
  }

  builder.addSphereCast(
      Eigen::Vector3d(-3.0, 2.0, 0.5),
      Eigen::Vector3d(3.0, 2.0, 0.5),
      0.2,
      nullptr);

  std::cout << "\n--- AABB Visualization ---\n";

  collision::Aabb boxAabb
      = collision::Aabb::forBox(Eigen::Vector3d(0.5, 0.5, 0.5));
  collision::Aabb worldBoxAabb = collision::Aabb::transformed(
      boxAabb,
      Eigen::Translation3d(-1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());
  builder.addAabb(worldBoxAabb, vsg::colors::Orange);
  std::cout << "Added AABB for box at (-1.5, 0, 0): ["
            << worldBoxAabb.min.transpose() << "] to ["
            << worldBoxAabb.max.transpose() << "]\n";

  collision::Aabb sphereAabb = collision::Aabb::forSphere(0.4);
  collision::Aabb worldSphereAabb = collision::Aabb::transformed(
      sphereAabb,
      Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
  builder.addAabb(worldSphereAabb, vsg::colors::Yellow);
  std::cout << "Added AABB for green sphere at origin\n";

  collision::Aabb capsuleAabb = collision::Aabb::forCapsule(0.3, 0.8);
  collision::Aabb worldCapsuleAabb = collision::Aabb::transformed(
      capsuleAabb,
      Eigen::Translation3d(1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());
  builder.addAabb(worldCapsuleAabb, vsg::colors::Orange);
  std::cout << "Added AABB for capsule at (1.5, 0, 0)\n";

  collision::Aabb cylinderAabb = collision::Aabb::forCylinder(0.35, 0.7);
  collision::Aabb worldCylinderAabb = collision::Aabb::transformed(
      cylinderAabb,
      Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
  builder.addAabb(worldCylinderAabb, vsg::colors::Orange);
  std::cout << "Added AABB for cylinder at (3.0, 0, 0)\n";

  std::cout << "\n--- Distance Query Visualization ---\n";
  collision::DistanceResult distResult;
  distResult.distance = 1.0;
  distResult.pointOnObject1 = Eigen::Vector3d(-1.0, 0.8, 0.8);
  distResult.pointOnObject2 = Eigen::Vector3d(0.4, 0.8, 0.6);
  distResult.normal
      = (distResult.pointOnObject2 - distResult.pointOnObject1).normalized();
  builder.addDistanceResult(distResult);
  std::cout << "Added distance visualization:\n";
  std::cout << "  Point 1: " << distResult.pointOnObject1.transpose() << "\n";
  std::cout << "  Point 2: " << distResult.pointOnObject2.transpose() << "\n";
  std::cout << "  Distance: " << distResult.distance
            << " (valid: " << distResult.isValid() << ")\n";

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

  if (headlessMode) {
    std::cout << "\nRunning in headless mode (" << width << "x" << height
              << ")...\n";

    auto viewer = vsg::SimpleViewer::headless(width, height);
    viewer.setScene(scene);
    viewer.addGrid(6.0, 1.0);
    viewer.addAxes(1.0);
    viewer.lookAt(
        Eigen::Vector3d(6.0, -4.0, 3.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d::UnitZ());

    viewer.run();

    if (viewer.saveScreenshot(outputFile)) {
      std::cout << "Screenshot saved to " << outputFile << "\n";
    } else {
      std::cerr << "Failed to save screenshot\n";
    }

    auto buffer = viewer.captureBuffer();
    std::cout << "Captured buffer: " << buffer.size() << " bytes (" << width
              << "x" << height << " RGBA)\n";
  } else {
    std::cout << "\nStarting VSG viewer (" << width << "x" << height
              << ")...\n";
    std::cout << "Use mouse to rotate/zoom, close window to exit.\n";
    std::cout << "(Run with --headless for headless mode)\n\n";

    vsg::SimpleViewer viewer(width, height, "DART Collision Visualization");
    viewer.setScene(scene);
    viewer.addGrid(6.0, 1.0);
    viewer.addAxes(1.0);
    viewer.lookAt(
        Eigen::Vector3d(6.0, -4.0, 3.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d::UnitZ());

    viewer.run();
  }

  std::cout << "Done.\n";
  return 0;
}
