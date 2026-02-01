/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/orbit_camera_controller.hpp>
#include <dart/gui/raylib/raylib_backend.hpp>
#include <dart/gui/scene_viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/All.hpp>

#include <gtest/gtest.h>
#include <raylib.h>

#include <filesystem>
#include <fstream>

#include <cstdio>

namespace {

const std::string kTestOutputDir = "/tmp/dart_gui_test";

void ensureOutputDir()
{
  std::filesystem::create_directories(kTestOutputDir);
}

void removeFile(const std::string& path)
{
  std::filesystem::remove(path);
}

bool fileExists(const std::string& path)
{
  return std::filesystem::exists(path);
}

std::uintmax_t fileSize(const std::string& path)
{
  return std::filesystem::file_size(path);
}

std::shared_ptr<dart::simulation::World> createTestWorld()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto ground = dart::dynamics::Skeleton::create("ground");
  auto [groundJoint, groundBody]
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto groundShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(10, 10, 0.1));
  auto groundShapeNode = groundBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(groundShape);
  groundShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.7, 0.7, 0.7));
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation().z() = -0.05;
  groundJoint->setTransformFromParentBodyNode(groundTf);
  world->addSkeleton(ground);

  auto box = dart::dynamics::Skeleton::create("box");
  auto [boxJoint, boxBody]
      = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.5, 0.5));
  auto boxShapeNode = boxBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(boxShape);
  boxShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.2, 0.2));
  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(boxShape->computeInertia(1.0));
  boxBody->setInertia(inertia);
  Eigen::Isometry3d boxTf = Eigen::Isometry3d::Identity();
  boxTf.translation() = Eigen::Vector3d(0, 0, 1.0);
  boxJoint->setTransformFromParentBodyNode(boxTf);
  world->addSkeleton(box);

  auto sphereSkel = dart::dynamics::Skeleton::create("sphere");
  auto [sphereJoint, sphereBody]
      = sphereSkel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto sphereShape = std::make_shared<dart::dynamics::SphereShape>(0.2);
  auto sphereShapeNode = sphereBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(sphereShape);
  sphereShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.6, 0.8));
  dart::dynamics::Inertia sphereInertia;
  sphereInertia.setMass(0.5);
  sphereInertia.setMoment(sphereShape->computeInertia(0.5));
  sphereBody->setInertia(sphereInertia);
  Eigen::Isometry3d sphereTf = Eigen::Isometry3d::Identity();
  sphereTf.translation() = Eigen::Vector3d(0.5, 0.0, 2.0);
  sphereJoint->setTransformFromParentBodyNode(sphereTf);
  world->addSkeleton(sphereSkel);

  return world;
}

struct TestViewer
{
  dart::gui::SceneViewer viewer;

  TestViewer()
    : viewer(std::make_unique<dart::gui::RaylibBackend>(), makeConfig())
  {
    viewer.setWorld(createTestWorld());
  }

  static dart::gui::ViewerConfig makeConfig()
  {
    dart::gui::ViewerConfig config;
    config.width = 640;
    config.height = 480;
    config.title = "DART Test";
    config.headless = true;
    config.target_fps = 0;
    return config;
  }
};

bool hasNonBackgroundPixels(const std::string& pngPath)
{
  Image img = LoadImage(pngPath.c_str());
  if (!IsImageValid(img)) {
    return false;
  }

  Color* colors = LoadImageColors(img);
  const int pixelCount = img.width * img.height;
  bool foundNonBg = false;
  for (int i = 0; i < pixelCount; ++i) {
    if (colors[i].r != 245 || colors[i].g != 245 || colors[i].b != 245) {
      foundNonBg = true;
      break;
    }
  }

  UnloadImageColors(colors);
  UnloadImage(img);
  return foundNonBg;
}

bool hasYellowPixels(const std::string& pngPath)
{
  Image img = LoadImage(pngPath.c_str());
  if (!IsImageValid(img)) {
    return false;
  }

  Color* colors = LoadImageColors(img);
  const int pixelCount = img.width * img.height;
  bool foundYellow = false;
  for (int i = 0; i < pixelCount; ++i) {
    if (colors[i].r > 200 && colors[i].g > 200 && colors[i].b < 60) {
      foundYellow = true;
      break;
    }
  }

  UnloadImageColors(colors);
  UnloadImage(img);
  return foundYellow;
}

bool imagesAreDifferent(const std::string& pathA, const std::string& pathB)
{
  Image imgA = LoadImage(pathA.c_str());
  Image imgB = LoadImage(pathB.c_str());

  if (!IsImageValid(imgA) || !IsImageValid(imgB)) {
    if (IsImageValid(imgA)) {
      UnloadImage(imgA);
    }
    if (IsImageValid(imgB)) {
      UnloadImage(imgB);
    }
    return true;
  }

  if (imgA.width != imgB.width || imgA.height != imgB.height) {
    UnloadImage(imgA);
    UnloadImage(imgB);
    return true;
  }

  Color* colorsA = LoadImageColors(imgA);
  Color* colorsB = LoadImageColors(imgB);
  const int pixelCount = imgA.width * imgA.height;
  int diffCount = 0;
  for (int i = 0; i < pixelCount; ++i) {
    if (colorsA[i].r != colorsB[i].r || colorsA[i].g != colorsB[i].g
        || colorsA[i].b != colorsB[i].b) {
      ++diffCount;
    }
  }

  UnloadImageColors(colorsA);
  UnloadImageColors(colorsB);
  UnloadImage(imgA);
  UnloadImage(imgB);

  return diffCount > (pixelCount / 100);
}

} // namespace

class RaylibSceneViewerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ensureOutputDir();
  }
};

TEST_F(RaylibSceneViewerTest, HeadlessInitialization)
{
  TestViewer tv;
  EXPECT_TRUE(tv.viewer.isPaused() == false);
  EXPECT_TRUE(tv.viewer.frame());
}

TEST_F(RaylibSceneViewerTest, MultipleFrames)
{
  TestViewer tv;
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(tv.viewer.frame());
  }
}

TEST_F(RaylibSceneViewerTest, ScreenshotCapture)
{
  const std::string path = kTestOutputDir + "/capture_basic.png";
  removeFile(path);

  TestViewer tv;
  tv.viewer.captureScreenshot(path);
  tv.viewer.frame();

  EXPECT_TRUE(fileExists(path)) << "Screenshot file was not created";
  EXPECT_GT(fileSize(path), 1000u) << "Screenshot file is suspiciously small";
}

TEST_F(RaylibSceneViewerTest, RenderedSceneHasContent)
{
  const std::string path = kTestOutputDir + "/capture_content.png";
  removeFile(path);

  TestViewer tv;
  for (int i = 0; i < 3; ++i) {
    tv.viewer.frame();
  }
  tv.viewer.captureScreenshot(path);
  tv.viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_TRUE(hasNonBackgroundPixels(path))
      << "Rendered image should contain non-background pixels (shapes, grid, "
         "axes)";
}

TEST_F(RaylibSceneViewerTest, CameraOrbitChangesView)
{
  const std::string pathBefore = kTestOutputDir + "/capture_orbit_before.png";
  const std::string pathAfter = kTestOutputDir + "/capture_orbit_after.png";
  removeFile(pathBefore);
  removeFile(pathAfter);

  TestViewer tv;
  tv.viewer.pause();

  tv.viewer.captureScreenshot(pathBefore);
  tv.viewer.frame();

  auto& cam = tv.viewer.camera();
  cam.position = Eigen::Vector3d(4.0, -1.0, 3.0);

  tv.viewer.captureScreenshot(pathAfter);
  tv.viewer.frame();

  ASSERT_TRUE(fileExists(pathBefore));
  ASSERT_TRUE(fileExists(pathAfter));
  EXPECT_TRUE(imagesAreDifferent(pathBefore, pathAfter))
      << "Camera orbit should produce a different view";
}

TEST_F(RaylibSceneViewerTest, OrbitCameraControllerHandlesEvents)
{
  dart::gui::OrbitCameraController controller;
  dart::gui::Camera cam;
  cam.position = Eigen::Vector3d(2.0, -2.0, 2.0);
  cam.target = Eigen::Vector3d(0.0, 0.0, 0.0);
  cam.up = Eigen::Vector3d(0.0, 0.0, 1.0);

  const Eigen::Vector3d posBefore = cam.position;

  std::vector<dart::gui::InputEvent> events;
  events.push_back(dart::gui::MouseButtonEvent{
      dart::gui::MouseButton::Right, true, 320, 240});
  events.push_back(dart::gui::MouseMoveEvent{330, 240, 10, 0});
  events.push_back(dart::gui::MouseButtonEvent{
      dart::gui::MouseButton::Right, false, 330, 240});

  controller.handleEvents(events, cam);

  EXPECT_NE(cam.position, posBefore) << "Right-drag should orbit the camera";
  EXPECT_DOUBLE_EQ(cam.target.x(), 0.0);
  EXPECT_DOUBLE_EQ(cam.target.y(), 0.0);
  EXPECT_DOUBLE_EQ(cam.target.z(), 0.0);
}

TEST_F(RaylibSceneViewerTest, OrbitCameraControllerZoom)
{
  dart::gui::OrbitCameraController controller;
  dart::gui::Camera cam;
  cam.position = Eigen::Vector3d(2.0, -2.0, 2.0);
  cam.target = Eigen::Vector3d(0.0, 0.0, 0.0);

  const double distBefore = (cam.position - cam.target).norm();

  std::vector<dart::gui::InputEvent> events;
  events.push_back(dart::gui::ScrollEvent{0.0, 2.0}); // scroll up = zoom in

  controller.handleEvents(events, cam);

  const double distAfter = (cam.position - cam.target).norm();
  EXPECT_LT(distAfter, distBefore)
      << "Scroll up should zoom in (reduce distance)";
}

TEST_F(RaylibSceneViewerTest, OrbitCameraControllerPan)
{
  dart::gui::OrbitCameraController controller;
  dart::gui::Camera cam;
  cam.position = Eigen::Vector3d(2.0, -2.0, 2.0);
  cam.target = Eigen::Vector3d(0.0, 0.0, 0.0);

  const Eigen::Vector3d targetBefore = cam.target;
  const Eigen::Vector3d posBefore = cam.position;

  std::vector<dart::gui::InputEvent> events;
  events.push_back(dart::gui::MouseButtonEvent{
      dart::gui::MouseButton::Middle, true, 320, 240});
  events.push_back(dart::gui::MouseMoveEvent{340, 240, 20, 0});
  events.push_back(dart::gui::MouseButtonEvent{
      dart::gui::MouseButton::Middle, false, 340, 240});

  controller.handleEvents(events, cam);

  const Eigen::Vector3d posOffset = cam.position - posBefore;
  const Eigen::Vector3d targetOffset = cam.target - targetBefore;
  EXPECT_GT(posOffset.norm(), 1e-6) << "Pan should move the camera";
  EXPECT_NEAR(posOffset.x(), targetOffset.x(), 1e-9);
  EXPECT_NEAR(posOffset.y(), targetOffset.y(), 1e-9);
  EXPECT_NEAR(posOffset.z(), targetOffset.z(), 1e-9);
}

TEST_F(RaylibSceneViewerTest, PickingReturnsHit)
{
  TestViewer tv;
  // Run a few frames to initialize
  for (int i = 0; i < 3; ++i) {
    tv.viewer.frame();
  }

  auto backend = std::make_unique<dart::gui::RaylibBackend>();
  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(std::move(backend), config);
  viewer.setWorld(createTestWorld());
  viewer.pause();

  for (int i = 0; i < 3; ++i) {
    viewer.frame();
  }

  EXPECT_FALSE(viewer.selectedNodeId().has_value())
      << "Nothing should be selected initially";
}

TEST_F(RaylibSceneViewerTest, PauseAndStep)
{
  TestViewer tv;
  tv.viewer.frame();

  EXPECT_FALSE(tv.viewer.isPaused());

  tv.viewer.pause();
  EXPECT_TRUE(tv.viewer.isPaused());

  tv.viewer.unpause();
  EXPECT_FALSE(tv.viewer.isPaused());

  tv.viewer.pause();
  tv.viewer.step();
  EXPECT_TRUE(tv.viewer.isPaused());
}

TEST_F(RaylibSceneViewerTest, SelectionHighlightVisible)
{
  const std::string pathNoSel = kTestOutputDir + "/capture_no_selection.png";
  const std::string pathSel = kTestOutputDir + "/capture_with_selection.png";
  removeFile(pathNoSel);
  removeFile(pathSel);

  TestViewer tv;
  tv.viewer.pause();

  tv.viewer.captureScreenshot(pathNoSel);
  tv.viewer.frame();

  ASSERT_TRUE(fileExists(pathNoSel));

  auto& cam = tv.viewer.camera();
  cam.position = Eigen::Vector3d(2.0, -2.0, 2.0);
  cam.target = Eigen::Vector3d(0.0, 0.0, 0.5);

  tv.viewer.frame();

  EXPECT_FALSE(hasYellowPixels(pathNoSel))
      << "No yellow wireframe should appear without selection";
}

TEST_F(RaylibSceneViewerTest, DebugDrawing)
{
  const std::string path = kTestOutputDir + "/capture_debug.png";
  removeFile(path);

  TestViewer tv;
  tv.viewer.pause();
  tv.viewer.frame();

  tv.viewer.addDebugLine(
      Eigen::Vector3d(0, 0, 0),
      Eigen::Vector3d(2, 0, 0),
      Eigen::Vector4d(1, 0, 0, 1));
  tv.viewer.addDebugPoint(
      Eigen::Vector3d(0, 0, 1), Eigen::Vector4d(1, 1, 0, 1), 0.05);

  tv.viewer.captureScreenshot(path);
  tv.viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_GT(fileSize(path), 1000u);
}

TEST_F(RaylibSceneViewerTest, ClearDebug)
{
  TestViewer tv;
  tv.viewer.pause();
  tv.viewer.frame();

  tv.viewer.addDebugLine(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
  tv.viewer.clearDebug();

  tv.viewer.frame();
}

TEST_F(RaylibSceneViewerTest, SimulationProgresses)
{
  auto world = createTestWorld();
  const double timeBefore = world->getTime();

  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  for (int i = 0; i < 10; ++i) {
    viewer.frame();
  }

  EXPECT_GT(world->getTime(), timeBefore)
      << "Simulation time should advance after frames";
}

TEST_F(RaylibSceneViewerTest, PausedSimulationDoesNotProgress)
{
  auto world = createTestWorld();

  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);
  viewer.pause();

  viewer.frame();
  const double timeAfterInit = world->getTime();

  for (int i = 0; i < 10; ++i) {
    viewer.frame();
  }

  EXPECT_DOUBLE_EQ(world->getTime(), timeAfterInit)
      << "Paused simulation should not advance";
}

TEST_F(RaylibSceneViewerTest, NoWorldDoesNotCrash)
{
  dart::gui::ViewerConfig config;
  config.width = 320;
  config.height = 240;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  EXPECT_TRUE(viewer.frame());
  EXPECT_TRUE(viewer.frame());
}
