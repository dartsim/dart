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

#include <dart/gui/vsg/collision_scene_builder.hpp>
#include <dart/gui/vsg/im_gui_viewer.hpp>
#include <dart/gui/vsg/simple_viewer.hpp>

#include <dart/collision/experimental/aabb.hpp>
#include <dart/collision/experimental/collision_object.hpp>
#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/experimental/types.hpp>

#include <CLI/CLI.hpp>

#include <iostream>

#include <cstdio>

namespace collision = dart::collision::experimental;
namespace guivsg = dart::gui::vsg;

enum class DemoMode
{
  Shapes,
  Contacts,
  Filtering,
  Distance,
  Raycast,
  CCD,
  Picking
};

collision::Aabb computeWorldAabb(const collision::CollisionObject& obj)
{
  const auto* shape = obj.getShape();
  const auto& transform = obj.getTransform();

  collision::Aabb localAabb;
  switch (shape->getType()) {
    case collision::ShapeType::Sphere: {
      const auto* s = static_cast<const collision::SphereShape*>(shape);
      localAabb = collision::Aabb::forSphere(s->getRadius());
      break;
    }
    case collision::ShapeType::Box: {
      const auto* b = static_cast<const collision::BoxShape*>(shape);
      localAabb = collision::Aabb::forBox(b->getHalfExtents());
      break;
    }
    case collision::ShapeType::Capsule: {
      const auto* c = static_cast<const collision::CapsuleShape*>(shape);
      localAabb = collision::Aabb::forCapsule(c->getRadius(), c->getHeight());
      break;
    }
    case collision::ShapeType::Cylinder: {
      const auto* c = static_cast<const collision::CylinderShape*>(shape);
      localAabb = collision::Aabb::forCylinder(c->getRadius(), c->getHeight());
      break;
    }
    default:
      localAabb = collision::Aabb(
          Eigen::Vector3d(-0.5, -0.5, -0.5), Eigen::Vector3d(0.5, 0.5, 0.5));
  }
  return collision::Aabb::transformed(localAabb, transform);
}

struct DemoState
{
  DemoMode mode = DemoMode::Contacts;
  bool showAABBs = true;
  bool showGrid = true;
  bool showAxes = true;
  float sphereOffset = 0.6f;
  float rayAngle = 0.0f;
  bool needsRebuild = true;

  bool hasPickRay = false;
  Eigen::Vector3d pickRayOrigin = Eigen::Vector3d::Zero();
  Eigen::Vector3d pickRayDirection = Eigen::Vector3d::UnitZ();
  bool pickHit = false;
  Eigen::Vector3d pickHitPoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d pickHitNormal = Eigen::Vector3d::UnitZ();

  double hoverX = 0.0;
  double hoverY = 0.0;
  bool hoverHit = false;
  Eigen::Vector3d hoverHitPoint = Eigen::Vector3d::Zero();
  std::string hoverInfo;

  const collision::CollisionObject* selectedObject = nullptr;
  collision::Aabb selectedAabb;
};

void addPrimitiveTargets(
    guivsg::CollisionSceneBuilder& builder,
    collision::CollisionWorld& world,
    bool showAABBs)
{
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

  builder.addObject(box, guivsg::colors::Blue);
  builder.addObject(sphere, guivsg::colors::Green);
  builder.addObject(capsule, guivsg::colors::Yellow);
  builder.addObject(cylinder, guivsg::colors::Cyan);

  if (showAABBs) {
    collision::Aabb boxAabb
        = collision::Aabb::forBox(Eigen::Vector3d(0.5, 0.5, 0.5));
    collision::Aabb worldBoxAabb = collision::Aabb::transformed(
        boxAabb,
        Eigen::Translation3d(-1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());
    builder.addAabb(worldBoxAabb, guivsg::colors::Orange);

    collision::Aabb sphereAabb = collision::Aabb::forSphere(0.4);
    collision::Aabb worldSphereAabb = collision::Aabb::transformed(
        sphereAabb,
        Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
    builder.addAabb(worldSphereAabb, guivsg::colors::Orange);

    collision::Aabb capsuleAabb = collision::Aabb::forCapsule(0.3, 0.8);
    collision::Aabb worldCapsuleAabb = collision::Aabb::transformed(
        capsuleAabb,
        Eigen::Translation3d(1.5, 0.0, 0.0) * Eigen::Isometry3d::Identity());
    builder.addAabb(worldCapsuleAabb, guivsg::colors::Orange);

    collision::Aabb cylinderAabb = collision::Aabb::forCylinder(0.35, 0.7);
    collision::Aabb worldCylinderAabb = collision::Aabb::transformed(
        cylinderAabb,
        Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
    builder.addAabb(worldCylinderAabb, guivsg::colors::Orange);
  }
}

void buildScene(
    guivsg::CollisionSceneBuilder& builder,
    collision::CollisionWorld& world,
    DemoState& state)
{
  builder.clear();

  switch (state.mode) {
    case DemoMode::Shapes: {
      addPrimitiveTargets(builder, world, state.showAABBs);
      break;
    }

    case DemoMode::Contacts: {
      auto staticSphere
          = world.createObject(std::make_unique<collision::SphereShape>(0.4));
      staticSphere.setTransform(
          Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::Isometry3d::Identity());
      builder.addObject(staticSphere, guivsg::colors::Green);

      auto movingSphere
          = world.createObject(std::make_unique<collision::SphereShape>(0.35));
      movingSphere.setTransform(
          Eigen::Translation3d(0.0, state.sphereOffset, 0.0)
          * Eigen::Isometry3d::Identity());
      builder.addObject(movingSphere, guivsg::colors::Magenta);

      if (state.showAABBs) {
        collision::Aabb staticAabb = collision::Aabb::forSphere(0.4);
        collision::Aabb worldStaticAabb = collision::Aabb::transformed(
            staticAabb,
            Eigen::Translation3d(0.0, 0.0, 0.0)
                * Eigen::Isometry3d::Identity());
        builder.addAabb(worldStaticAabb, guivsg::colors::Orange);

        collision::Aabb movingAabb = collision::Aabb::forSphere(0.35);
        collision::Aabb worldMovingAabb = collision::Aabb::transformed(
            movingAabb,
            Eigen::Translation3d(0.0, state.sphereOffset, 0.0)
                * Eigen::Isometry3d::Identity());
        builder.addAabb(worldMovingAabb, guivsg::colors::Orange);
      }

      collision::CollisionResult result;
      collision::CollisionOption option;
      option.enableContact = true;
      world.collide(option, result);
      builder.addContacts(result, 0.3, 0.05);
      break;
    }

    case DemoMode::Filtering: {
      constexpr std::uint32_t Group1 = 0x01;
      constexpr std::uint32_t Group2 = 0x02;

      auto obj1
          = world.createObject(std::make_unique<collision::SphereShape>(0.3));
      obj1.setTransform(
          Eigen::Translation3d(-0.5, 1.5, 0.0) * Eigen::Isometry3d::Identity());
      obj1.setCollisionGroup(Group1);
      obj1.setCollisionMask(Group2);
      builder.addObject(obj1, guivsg::colors::Red);

      auto obj2
          = world.createObject(std::make_unique<collision::SphereShape>(0.3));
      obj2.setTransform(
          Eigen::Translation3d(0.5, 1.5, 0.0) * Eigen::Isometry3d::Identity());
      obj2.setCollisionGroup(Group2);
      obj2.setCollisionMask(Group1);
      builder.addObject(obj2, guivsg::colors::Blue);

      collision::CollisionResult result;
      collision::CollisionOption option;
      option.enableContact = true;
      world.collide(option, result);
      builder.addContacts(result, 0.2, 0.03);
      break;
    }

    case DemoMode::Distance: {
      auto distSphere1
          = world.createObject(std::make_unique<collision::SphereShape>(0.3));
      distSphere1.setTransform(
          Eigen::Translation3d(-0.8, 1.5, 0.5) * Eigen::Isometry3d::Identity());
      builder.addObject(distSphere1, guivsg::colors::Red);

      auto distSphere2
          = world.createObject(std::make_unique<collision::SphereShape>(0.25));
      distSphere2.setTransform(
          Eigen::Translation3d(0.6, 1.5, 0.3) * Eigen::Isometry3d::Identity());
      builder.addObject(distSphere2, guivsg::colors::Blue);

      if (state.showAABBs) {
        collision::Aabb ds1Aabb = collision::Aabb::forSphere(0.3);
        collision::Aabb worldDs1Aabb = collision::Aabb::transformed(
            ds1Aabb,
            Eigen::Translation3d(-0.8, 1.5, 0.5)
                * Eigen::Isometry3d::Identity());
        builder.addAabb(worldDs1Aabb, guivsg::colors::Orange);

        collision::Aabb ds2Aabb = collision::Aabb::forSphere(0.25);
        collision::Aabb worldDs2Aabb = collision::Aabb::transformed(
            ds2Aabb,
            Eigen::Translation3d(0.6, 1.5, 0.3)
                * Eigen::Isometry3d::Identity());
        builder.addAabb(worldDs2Aabb, guivsg::colors::Orange);
      }

      Eigen::Vector3d center1(-0.8, 1.5, 0.5);
      Eigen::Vector3d center2(0.6, 1.5, 0.3);
      double r1 = 0.3;
      double r2 = 0.25;
      Eigen::Vector3d diff = center2 - center1;
      double centerDist = diff.norm();
      double surfaceDist = centerDist - r1 - r2;

      collision::DistanceResult distResult;
      distResult.distance = surfaceDist;
      distResult.normal = diff.normalized();
      distResult.pointOnObject1 = center1 + distResult.normal * r1;
      distResult.pointOnObject2 = center2 - distResult.normal * r2;
      builder.addDistanceResult(distResult);
      break;
    }

    case DemoMode::Raycast: {
      addPrimitiveTargets(builder, world, state.showAABBs);

      double angle = state.rayAngle * M_PI / 180.0;
      Eigen::Vector3d dir(std::cos(angle), std::sin(angle), -0.1);
      collision::Ray ray(
          Eigen::Vector3d(-3.0, -1.5, 1.0), dir.normalized(), 8.0);

      collision::RaycastOption rayOpt;
      rayOpt.maxDistance = ray.maxDistance;
      collision::RaycastResult rayHit;
      bool hit = world.raycast(ray, rayOpt, rayHit);
      builder.addRaycast(ray, hit ? &rayHit : nullptr);
      break;
    }

    case DemoMode::CCD: {
      addPrimitiveTargets(builder, world, state.showAABBs);

      Eigen::Vector3d ccdStart(-3.0, 0.0, 0.0);
      Eigen::Vector3d ccdEnd(3.0, 0.0, 0.0);
      double ccdRadius = 0.15;

      collision::CcdOption ccdOpt = collision::CcdOption::standard();
      collision::CcdResult ccdHit;
      bool hit = world.sphereCast(ccdStart, ccdEnd, ccdRadius, ccdOpt, ccdHit);
      builder.addSphereCast(
          ccdStart, ccdEnd, ccdRadius, hit ? &ccdHit : nullptr);
      break;
    }

    case DemoMode::Picking: {
      addPrimitiveTargets(builder, world, state.showAABBs);

      if (state.hasPickRay) {
        collision::Ray pickRay(
            state.pickRayOrigin, state.pickRayDirection, 100.0);

        collision::RaycastOption rayOpt;
        rayOpt.maxDistance = pickRay.maxDistance;
        collision::RaycastResult rayHit;
        bool hit = world.raycast(pickRay, rayOpt, rayHit);

        state.pickHit = hit;
        if (hit) {
          state.pickHitPoint = rayHit.point;
          state.pickHitNormal = rayHit.normal;
        }

        builder.addRaycast(pickRay, hit ? &rayHit : nullptr);
      }
      break;
    }
  }

  state.needsRebuild = false;
}

int main(int argc, char* argv[])
{
  CLI::App app("DART Collision Sandbox - Interactive Demo");

  bool headless = false;
  app.add_flag(
      "--headless", headless, "Run in headless mode (render and exit)");

  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);

  int width = 1280;
  app.add_option("-W,--width", width, "Window width")
      ->check(CLI::PositiveNumber);

  int height = 720;
  app.add_option("-H,--height", height, "Window height")
      ->check(CLI::PositiveNumber);

  std::string outputFile = "collision_sandbox.ppm";
  app.add_option("-o,--output", outputFile, "Output file for headless mode");

  CLI11_PARSE(app, argc, argv);

  int scaledWidth = static_cast<int>(width * guiScale);
  int scaledHeight = static_cast<int>(height * guiScale);

  std::cout << "DART Collision Sandbox - Interactive Demo\n";
  std::cout << "=========================================\n\n";

  collision::CollisionWorld world;
  guivsg::CollisionSceneBuilder builder;
  DemoState state;

  if (headless) {
    auto viewer = guivsg::SimpleViewer::headless(scaledWidth, scaledHeight);
    viewer.addGrid(6.0, 1.0);
    viewer.addAxes(1.0);
    viewer.lookAt(
        Eigen::Vector3d(6.0, -4.0, 3.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
        Eigen::Vector3d::UnitZ());

    buildScene(builder, world, state);
    viewer.setScene(builder.build());
    viewer.frame();

    if (viewer.saveScreenshot(outputFile)) {
      std::cout << "Saved screenshot to: " << outputFile << "\n";
    } else {
      std::cerr << "Failed to save screenshot\n";
      return 1;
    }
    std::cout << "Done.\n";
    return 0;
  }

  guivsg::ImGuiViewer viewer(
      scaledWidth, scaledHeight, "DART Collision Sandbox");
  viewer.addGrid(6.0, 1.0);
  viewer.addAxes(1.0);
  viewer.lookAt(
      Eigen::Vector3d(6.0, -4.0, 3.0),
      Eigen::Vector3d(0.5, 0.0, 0.0),
      Eigen::Vector3d::UnitZ());

  float guiScaleF = static_cast<float>(guiScale);
  bool firstFrame = true;
  viewer.setImGuiCallback([&state, guiScaleF, &firstFrame, &viewer, &world]() {
    ImGui::GetIO().FontGlobalScale = guiScaleF;

    if (firstFrame) {
      ImGui::SetNextWindowPos(ImVec2(10 * guiScaleF, 10 * guiScaleF));
      ImGui::SetNextWindowSize(ImVec2(280 * guiScaleF, 320 * guiScaleF));
      firstFrame = false;
    }

    ImGui::Begin("Collision Sandbox Controls");

    ImGui::Text("Demo Mode:");
    if (ImGui::RadioButton("Shapes", state.mode == DemoMode::Shapes)) {
      state.mode = DemoMode::Shapes;
      state.needsRebuild = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Contacts", state.mode == DemoMode::Contacts)) {
      state.mode = DemoMode::Contacts;
      state.needsRebuild = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Filtering", state.mode == DemoMode::Filtering)) {
      state.mode = DemoMode::Filtering;
      state.needsRebuild = true;
    }

    if (ImGui::RadioButton("Distance", state.mode == DemoMode::Distance)) {
      state.mode = DemoMode::Distance;
      state.needsRebuild = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Raycast", state.mode == DemoMode::Raycast)) {
      state.mode = DemoMode::Raycast;
      state.needsRebuild = true;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("CCD", state.mode == DemoMode::CCD)) {
      state.mode = DemoMode::CCD;
      state.needsRebuild = true;
    }

    if (ImGui::RadioButton("Picking", state.mode == DemoMode::Picking)) {
      state.mode = DemoMode::Picking;
      state.hasPickRay = false;
      state.needsRebuild = true;
    }

    ImGui::Separator();

    if (ImGui::Checkbox("Show AABBs", &state.showAABBs)) {
      state.needsRebuild = true;
    }

    ImGui::Separator();

    if (state.mode == DemoMode::Contacts) {
      ImGui::Text("Contact Parameters:");
      if (ImGui::SliderFloat(
              "Sphere Y Offset", &state.sphereOffset, 0.0f, 2.0f)) {
        state.needsRebuild = true;
      }
    }

    if (state.mode == DemoMode::Raycast) {
      ImGui::Text("Ray Parameters:");
      if (ImGui::SliderFloat("Ray Angle", &state.rayAngle, -45.0f, 45.0f)) {
        state.needsRebuild = true;
      }
    }

    if (state.mode == DemoMode::Picking) {
      ImGui::Text("Left-click in 3D view to cast ray");
      if (state.hasPickRay) {
        if (state.pickHit) {
          ImGui::TextColored(
              ImVec4(0.0f, 1.0f, 0.0f, 1.0f),
              "Hit at (%.2f, %.2f, %.2f)",
              state.pickHitPoint.x(),
              state.pickHitPoint.y(),
              state.pickHitPoint.z());
        } else {
          ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "No hit");
        }
      }
    }

    ImGui::End();

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mousePos = io.MousePos;

    if (!io.WantCaptureMouse) {
      char hoverBuf[128];
      Eigen::Vector3d rayOrigin, rayDir;
      if (viewer.computePickingRay(mousePos.x, mousePos.y, rayOrigin, rayDir)) {
        collision::Ray ray(rayOrigin, rayDir, 100.0);
        collision::RaycastOption rayOpt;
        collision::RaycastResult rayHit;
        bool hit = world.raycast(ray, rayOpt, rayHit);

        if (hit && rayHit.object) {
          state.hoverHit = true;
          state.hoverHitPoint = rayHit.point;

          collision::Aabb aabb = computeWorldAabb(*rayHit.object);
          viewer.setHoverHighlight(aabb.min, aabb.max);

          std::snprintf(
              hoverBuf,
              sizeof(hoverBuf),
              "(%.2f, %.2f, %.2f)",
              rayHit.point.x(),
              rayHit.point.y(),
              rayHit.point.z());
          state.hoverInfo = hoverBuf;

          if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            state.selectedObject = rayHit.object;
            state.selectedAabb = aabb;
            viewer.setSelectionHighlight(aabb.min, aabb.max);

            if (state.mode == DemoMode::Picking) {
              state.pickRayOrigin = rayOrigin;
              state.pickRayDirection = rayDir;
              state.hasPickRay = true;
              state.needsRebuild = true;
            }
          }
        } else {
          state.hoverHit = false;
          viewer.clearHoverHighlight();
          state.hoverInfo.clear();

          if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            state.selectedObject = nullptr;
            viewer.clearSelectionHighlight();
          }
        }
      } else {
        viewer.clearHoverHighlight();
        state.hoverInfo.clear();
      }
    } else {
      viewer.clearHoverHighlight();
      state.hoverInfo.clear();
    }

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    float statusBarHeight = 24 * guiScaleF;
    ImGui::SetNextWindowPos(ImVec2(
        viewport->Pos.x, viewport->Pos.y + viewport->Size.y - statusBarHeight));
    ImGui::SetNextWindowSize(ImVec2(viewport->Size.x, statusBarHeight));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 4));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::Begin(
        "##StatusBar",
        nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize
            | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar
            | ImGuiWindowFlags_NoSavedSettings
            | ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImGui::Text("Right-drag: rotate | Middle-drag: pan | Scroll: zoom");
    if (!state.hoverInfo.empty()) {
      ImGui::SameLine(viewport->Size.x - 200 * guiScaleF);
      ImGui::Text("Pos: %s", state.hoverInfo.c_str());
    }

    ImGui::End();
    ImGui::PopStyleVar(2);
  });

  buildScene(builder, world, state);
  viewer.setScene(builder.build());

  while (!viewer.shouldClose()) {
    if (state.needsRebuild) {
      world = collision::CollisionWorld();
      buildScene(builder, world, state);
      viewer.setScene(builder.build());
    }
    viewer.frame();
  }

  std::cout << "Done.\n";
  return 0;
}
