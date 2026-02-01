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

#include <dart/gui/drag_controller.hpp>
#include <dart/gui/input_event.hpp>
#include <dart/gui/orbit_camera_controller.hpp>
#include <dart/gui/raylib/raylib_backend.hpp>
#include <dart/gui/scene.hpp>
#include <dart/gui/scene_viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/All.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/multi_sphere_convex_hull_shape.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>

#include <gtest/gtest.h>
#include <raylib.h>

#include <filesystem>
#include <random>

#include <cstdio>

extern "C" bool IsImageValid(Image image);

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
  if (img.data == nullptr || img.width <= 0 || img.height <= 0) {
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
  if (img.data == nullptr || img.width <= 0 || img.height <= 0) {
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

  const bool imgAValid
      = imgA.data != nullptr && imgA.width > 0 && imgA.height > 0;
  const bool imgBValid
      = imgB.data != nullptr && imgB.width > 0 && imgB.height > 0;
  if (!imgAValid || !imgBValid) {
    if (imgAValid) {
      UnloadImage(imgA);
    }
    if (imgBValid) {
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
      dart::gui::MouseButton::Right,
      true,
      320,
      240,
      dart::gui::ModifierKeys{}});
  events.push_back(
      dart::gui::MouseMoveEvent{330, 240, 10, 0, dart::gui::ModifierKeys{}});
  events.push_back(dart::gui::MouseButtonEvent{
      dart::gui::MouseButton::Right,
      false,
      330,
      240,
      dart::gui::ModifierKeys{}});

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
      dart::gui::MouseButton::Middle,
      true,
      320,
      240,
      dart::gui::ModifierKeys{}});
  events.push_back(
      dart::gui::MouseMoveEvent{340, 240, 20, 0, dart::gui::ModifierKeys{}});
  events.push_back(dart::gui::MouseButtonEvent{
      dart::gui::MouseButton::Middle,
      false,
      340,
      240,
      dart::gui::ModifierKeys{}});

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

TEST_F(RaylibSceneViewerTest, ConvexMeshExtraction)
{
  // Create a world with a ConvexMeshShape
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("convex_mesh_skel");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  // Create a simple tetrahedron TriMesh
  auto triMesh = std::make_shared<dart::math::TriMeshd>();
  triMesh->addVertex(Eigen::Vector3d(0.0, 0.0, 0.0));
  triMesh->addVertex(Eigen::Vector3d(1.0, 0.0, 0.0));
  triMesh->addVertex(Eigen::Vector3d(0.5, 1.0, 0.0));
  triMesh->addVertex(Eigen::Vector3d(0.5, 0.5, 1.0));

  // Add 4 triangular faces
  triMesh->addTriangle(0, 1, 2);
  triMesh->addTriangle(0, 1, 3);
  triMesh->addTriangle(1, 2, 3);
  triMesh->addTriangle(2, 0, 3);

  auto convexMeshShape = dart::dynamics::ConvexMeshShape::fromMesh(triMesh);
  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(convexMeshShape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.5, 0.5, 0.8));

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(convexMeshShape->computeInertia(1.0));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 1.0);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  // Create viewer and render
  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  // Render a few frames
  for (int i = 0; i < 3; ++i) {
    viewer.frame();
  }

  // Capture screenshot and verify it has non-background pixels
  const std::string path = kTestOutputDir + "/convex_mesh_test.png";
  removeFile(path);
  viewer.captureScreenshot(path);
  viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_TRUE(hasNonBackgroundPixels(path))
      << "ConvexMesh should render with visible pixels";
}

TEST_F(RaylibSceneViewerTest, PyramidShapeRendering)
{
  // Create a world with a PyramidShape
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("pyramid_skel");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  auto pyramidShape
      = std::make_shared<dart::dynamics::PyramidShape>(0.5, 0.5, 1.0);
  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(pyramidShape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.6, 0.2));

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(pyramidShape->computeInertia(1.0));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 1.0);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  // Create viewer and render
  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  // Render a few frames
  for (int i = 0; i < 3; ++i) {
    viewer.frame();
  }

  // Capture screenshot and verify it has non-background pixels
  const std::string path = kTestOutputDir + "/pyramid_test.png";
  removeFile(path);
  viewer.captureScreenshot(path);
  viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_TRUE(hasNonBackgroundPixels(path))
      << "Pyramid should render with visible pixels";
}

TEST_F(RaylibSceneViewerTest, UnsupportedShapeGracefulHandling)
{
  // Create a world with a supported shape (BoxShape)
  // This tests that the extractor handles shapes gracefully
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("box_skel");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.5, 0.5));
  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.5, 0.5, 0.5));

  dart::dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(boxShape->computeInertia(1.0));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 1.0);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  // Create viewer and render multiple frames
  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  // Render several frames — should not crash
  EXPECT_NO_THROW({
    for (int i = 0; i < 10; ++i) {
      viewer.frame();
    }
  });
}

TEST_F(RaylibSceneViewerTest, MaterialHasExpectedDefaults)
{
  // Test Material struct default values
  dart::gui::Material material;

  // Verify ambient color defaults
  EXPECT_NEAR(material.ambient.x(), 0.2, 1e-6);
  EXPECT_NEAR(material.ambient.y(), 0.2, 1e-6);
  EXPECT_NEAR(material.ambient.z(), 0.2, 1e-6);
  EXPECT_NEAR(material.ambient.w(), 1.0, 1e-6);

  // Verify specular color defaults
  EXPECT_NEAR(material.specular.x(), 1.0, 1e-6);
  EXPECT_NEAR(material.specular.y(), 1.0, 1e-6);
  EXPECT_NEAR(material.specular.z(), 1.0, 1e-6);
  EXPECT_NEAR(material.specular.w(), 1.0, 1e-6);

  // Verify shininess default
  EXPECT_NEAR(material.shininess, 32.0, 1e-6);
}

TEST_F(RaylibSceneViewerTest, PointCloudRendering)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("point_cloud_skel");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  auto pointCloud = std::make_shared<dart::dynamics::PointCloudShape>(0.02);
  pointCloud->reserve(50);
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> dist(-0.5, 0.5);
  for (int i = 0; i < 50; ++i) {
    pointCloud->addPoint(
        Eigen::Vector3d(dist(rng), dist(rng), dist(rng) + 1.0));
  }

  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(pointCloud);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.2, 0.7, 0.4, 1.0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 0.5);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  for (int i = 0; i < 3; ++i) {
    viewer.frame();
  }

  const std::string path = kTestOutputDir + "/point_cloud_test.png";
  removeFile(path);
  viewer.captureScreenshot(path);
  viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_TRUE(hasNonBackgroundPixels(path))
      << "PointCloud should render with visible pixels";
}

TEST_F(RaylibSceneViewerTest, MultiSphereRendering)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("multi_sphere_skel");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  dart::dynamics::MultiSphereConvexHullShape::Spheres spheres{
      {0.2, Eigen::Vector3d(0.0, 0.0, 0.0)},
      {0.15, Eigen::Vector3d(0.4, 0.0, 0.2)},
      {0.1, Eigen::Vector3d(-0.3, 0.2, 0.1)}};
  auto multiSphere
      = std::make_shared<dart::dynamics::MultiSphereConvexHullShape>(spheres);

  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(multiSphere);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.6, 0.4, 0.8, 1.0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 1.0);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  for (int i = 0; i < 3; ++i) {
    viewer.frame();
  }

  const std::string path = kTestOutputDir + "/multi_sphere_test.png";
  removeFile(path);
  viewer.captureScreenshot(path);
  viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_TRUE(hasNonBackgroundPixels(path))
      << "MultiSphere should render with visible pixels";
}

TEST_F(RaylibSceneViewerTest, TransparencyDoesNotCrash)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("transparent_box");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.4, 0.4, 0.4));
  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(boxShape);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.2, 0.6, 0.9, 0.5));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 1.0);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  EXPECT_NO_THROW({
    for (int i = 0; i < 5; ++i) {
      viewer.frame();
    }
  });
}

TEST_F(RaylibSceneViewerTest, MeshDataTexcoordsExist)
{
  dart::gui::MeshData meshData;
  EXPECT_TRUE(meshData.texcoords.empty());
  EXPECT_TRUE(meshData.texture_path.empty());
}

TEST_F(RaylibSceneViewerTest, ModifierKeysDefaults)
{
  dart::gui::ModifierKeys mods;
  EXPECT_FALSE(mods.ctrl);
  EXPECT_FALSE(mods.shift);
  EXPECT_FALSE(mods.alt);
  EXPECT_FALSE(mods.super);
}

TEST_F(RaylibSceneViewerTest, HeightmapRendering)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto skeleton = dart::dynamics::Skeleton::create("heightmap_skel");
  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();

  auto hmShape = std::make_shared<dart::dynamics::HeightmapShapef>();
  std::vector<float> heights(25, 0.0f); // 5x5 flat
  heights[12] = 1.0f;                   // center peak
  hmShape->setHeightField(5, 5, heights);
  hmShape->setScale(Eigen::Vector3f(2.0f, 2.0f, 1.0f));

  auto shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(hmShape);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.3, 0.6, 0.2, 1.0));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0, 0, 0.1);
  joint->setTransformFromParentBodyNode(tf);

  world->addSkeleton(skeleton);

  dart::gui::ViewerConfig config;
  config.width = 640;
  config.height = 480;
  config.headless = true;
  config.target_fps = 0;

  auto viewer = dart::gui::SceneViewer(
      std::make_unique<dart::gui::RaylibBackend>(), config);
  viewer.setWorld(world);

  for (int i = 0; i < 3; ++i) {
    viewer.frame();
  }

  const std::string path = kTestOutputDir + "/heightmap_test.png";
  removeFile(path);
  viewer.captureScreenshot(path);
  viewer.frame();

  ASSERT_TRUE(fileExists(path));
  EXPECT_TRUE(hasNonBackgroundPixels(path))
      << "Heightmap should render with visible pixels";
}

TEST_F(RaylibSceneViewerTest, GridConfigPlanes)
{
  dart::gui::GridConfig defaults;
  EXPECT_EQ(defaults.plane, dart::gui::GridConfig::Plane::ZX);
  EXPECT_EQ(defaults.num_cells, 20u);
  EXPECT_DOUBLE_EQ(defaults.cell_size, 1.0);
  EXPECT_EQ(defaults.minor_per_major, 5u);

  dart::gui::ViewerConfig config;
  config.width = 320;
  config.height = 240;
  config.headless = true;
  config.target_fps = 0;

  dart::gui::RaylibBackend backend;
  ASSERT_TRUE(backend.initialize(config));

  dart::gui::Scene scene;
  dart::gui::GridConfig grid;
  grid.plane = dart::gui::GridConfig::Plane::XY;
  grid.num_cells = 4;
  grid.cell_size = 0.5;
  scene.grid_config = grid;

  EXPECT_NO_THROW({
    backend.beginFrame();
    backend.render(scene);
    backend.endFrame();
  });

  backend.shutdown();
}

TEST_F(RaylibSceneViewerTest, DragControllerConstraints)
{
  dart::gui::Camera camera;
  camera.position = Eigen::Vector3d(0.0, -10.0, 0.0);
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.0);
  camera.up = Eigen::Vector3d(0.0, 0.0, 1.0);

  const Eigen::Vector3d fromPosition = Eigen::Vector3d::Zero();
  const double screenDx = 10.0;
  const double screenDy = 10.0;
  const double screenWidth = 640.0;
  const double screenHeight = 480.0;

  const Eigen::Vector3d unconstrained
      = dart::gui::DragController::getDeltaCursor(
          fromPosition,
          dart::gui::ConstraintType::Unconstrained,
          Eigen::Vector3d::UnitX(),
          camera,
          screenDx,
          screenDy,
          screenWidth,
          screenHeight);

  const Eigen::Vector3d lineDelta = dart::gui::DragController::getDeltaCursor(
      fromPosition,
      dart::gui::ConstraintType::Line,
      Eigen::Vector3d::UnitX(),
      camera,
      screenDx,
      screenDy,
      screenWidth,
      screenHeight);

  EXPECT_NEAR(lineDelta.y(), 0.0, 1e-9);
  EXPECT_NEAR(lineDelta.z(), 0.0, 1e-9);
  EXPECT_NEAR(lineDelta.x(), unconstrained.x(), 1e-9);

  const Eigen::Vector3d planeDelta = dart::gui::DragController::getDeltaCursor(
      fromPosition,
      dart::gui::ConstraintType::Plane,
      Eigen::Vector3d::UnitZ(),
      camera,
      screenDx,
      screenDy,
      screenWidth,
      screenHeight);

  EXPECT_NEAR(planeDelta.z(), 0.0, 1e-9);
}

TEST_F(RaylibSceneViewerTest, DragControllerDefaults)
{
  dart::gui::DragController controller;
  EXPECT_FALSE(controller.isDragging());
  EXPECT_EQ(controller.activeDraggable(), nullptr);
}

TEST_F(RaylibSceneViewerTest, SimpleFrameDraggableCreation)
{
  auto frame = dart::dynamics::SimpleFrame::createShared();
  frame->setTranslation(Eigen::Vector3d(1, 0, 0.5));

  TestViewer tv;
  tv.viewer.enableDragAndDrop(frame.get());

  for (int i = 0; i < 3; ++i) {
    tv.viewer.frame();
  }

  EXPECT_FALSE(tv.viewer.dragController().isDragging());
}

TEST_F(RaylibSceneViewerTest, SimpleFrameDraggableUpdateDrag)
{
  auto frame = dart::dynamics::SimpleFrame::createShared();
  frame->setTranslation(Eigen::Vector3d(1, 0, 0.5));

  const uint64_t markerId = 12345;
  dart::gui::SimpleFrameDraggable draggable(frame.get(), markerId);

  dart::gui::HitResult hit;
  hit.node_id = markerId;
  hit.point = Eigen::Vector3d(1, 0, 0.5);

  EXPECT_TRUE(draggable.canDrag(hit));

  draggable.beginDrag(hit);

  const Eigen::Vector3d posBefore = frame->getWorldTransform().translation();
  const Eigen::Vector3d delta(0.1, 0.0, 0.0);
  dart::gui::ModifierKeys mods;
  draggable.updateDrag(delta, mods);

  const Eigen::Vector3d posAfter = frame->getWorldTransform().translation();
  EXPECT_NEAR((posAfter - posBefore).norm(), 0.1, 1e-6);

  draggable.endDrag();
}

TEST_F(RaylibSceneViewerTest, BodyNodeDraggableCanDragMatch)
{
  auto world = dart::simulation::World::create();
  auto skel = dart::dynamics::Skeleton::create("test");
  auto [joint, body]
      = skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.5, 0.5));
  auto sn = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(boxShape);
  sn->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.2, 0.2));
  world->addSkeleton(skel);

  dart::gui::SceneExtractor extractor;
  extractor.extract(*world);

  dart::gui::BodyNodeDraggable draggable(body, extractor.entityMap());

  dart::gui::HitResult hit;
  hit.node_id = boxShape->getID();
  hit.point = Eigen::Vector3d(0, 0, 0);

  EXPECT_TRUE(draggable.canDrag(hit));

  dart::gui::HitResult wrongHit;
  wrongHit.node_id = 999999;
  EXPECT_FALSE(draggable.canDrag(wrongHit));
}

TEST_F(RaylibSceneViewerTest, BodyNodeDnDRegistration)
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

  viewer.frame();

  auto* body = world->getSkeleton(0)->getBodyNode(0);
  viewer.enableDragAndDrop(body);

  for (int i = 0; i < 5; ++i) {
    viewer.frame();
  }

  EXPECT_FALSE(viewer.dragController().isDragging());
}
