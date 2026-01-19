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

#include <dart/gui/All.hpp>

#include <dart/All.hpp>

#include <CLI/CLI.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::constraint;

namespace {

enum class Scenario
{
  MassRatio,
  BoxStack,
  BallDrop,
  Dominos,
  InclinedPlane
};

struct ScenarioInfo
{
  Scenario type;
  std::string name;
  std::string description;
};

std::vector<ScenarioInfo> GetScenarios()
{
  return {
      {Scenario::MassRatio,
       "mass_ratio",
       "Heavy box (1000kg) on light box (1kg) - tests solver stability"},
      {Scenario::BoxStack,
       "box_stack",
       "Pyramid of boxes - tests shock propagation"},
      {Scenario::BallDrop,
       "ball_drop",
       "Many balls dropping - tests many-contact performance"},
      {Scenario::Dominos,
       "dominos",
       "Domino chain reaction - tests impulse propagation"},
      {Scenario::InclinedPlane,
       "inclined_plane",
       "Block on slope - tests friction model accuracy"}};
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNodePtr body = ground->createJointAndBodyNodePair<WeldJoint>().second;

  double thickness = 0.1;
  auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(20.0, thickness, 20.0));
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation().y() = -thickness / 2.0;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);

  return ground;
}

SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.6, 0.6, 0.8))
{
  SkeletonPtr box = Skeleton::create(name);

  BodyNodePtr body = box->createJointAndBodyNodePair<FreeJoint>().second;

  auto shape = std::make_shared<BoxShape>(size);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);

  return box;
}

SkeletonPtr createSphere(
    const std::string& name,
    double radius,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color = Eigen::Vector3d(0.8, 0.4, 0.4))
{
  SkeletonPtr sphere = Skeleton::create(name);

  BodyNodePtr body = sphere->createJointAndBodyNodePair<FreeJoint>().second;

  auto shape = std::make_shared<SphereShape>(radius);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.5);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.6);

  return sphere;
}

WorldPtr createMassRatioScenario()
{
  auto world = World::create("mass_ratio");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double boxSize = 0.5;
  world->addSkeleton(createBox(
      "light_box",
      Eigen::Vector3d(boxSize, boxSize, boxSize),
      1.0,
      Eigen::Vector3d(0, boxSize / 2.0, 0),
      Eigen::Vector3d(0.4, 0.8, 0.4)));

  world->addSkeleton(createBox(
      "heavy_box",
      Eigen::Vector3d(boxSize, boxSize, boxSize),
      1000.0,
      Eigen::Vector3d(0, boxSize * 1.6, 0),
      Eigen::Vector3d(0.8, 0.2, 0.2)));

  return world;
}

WorldPtr createBoxStackScenario()
{
  auto world = World::create("box_stack");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double boxSize = 0.3;
  int layers = 5;

  int boxIndex = 0;
  for (int layer = 0; layer < layers; ++layer) {
    int boxesInLayer = layers - layer;
    double y = boxSize / 2.0 + layer * boxSize * 1.05;
    double startX = -(boxesInLayer - 1) * boxSize * 0.55;

    for (int i = 0; i < boxesInLayer; ++i) {
      double x = startX + i * boxSize * 1.1;
      double hue = static_cast<double>(boxIndex) / (layers * (layers + 1) / 2);
      Eigen::Vector3d color(0.3 + 0.5 * hue, 0.5, 0.8 - 0.5 * hue);

      world->addSkeleton(createBox(
          "box_" + std::to_string(boxIndex),
          Eigen::Vector3d(boxSize, boxSize, boxSize),
          1.0,
          Eigen::Vector3d(x, y, 0),
          color));
      ++boxIndex;
    }
  }

  return world;
}

WorldPtr createBallDropScenario()
{
  auto world = World::create("ball_drop");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double radius = 0.1;
  int gridSize = 5;
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> jitter(-0.02, 0.02);

  int ballIndex = 0;
  for (int x = 0; x < gridSize; ++x) {
    for (int z = 0; z < gridSize; ++z) {
      for (int y = 0; y < 3; ++y) {
        double px = (x - gridSize / 2.0) * radius * 2.5 + jitter(rng);
        double py = radius + y * radius * 2.2 + 0.5;
        double pz = (z - gridSize / 2.0) * radius * 2.5 + jitter(rng);

        double hue = static_cast<double>(ballIndex) / (gridSize * gridSize * 3);
        Eigen::Vector3d color(0.8 - 0.4 * hue, 0.4 + 0.4 * hue, 0.4);

        world->addSkeleton(createSphere(
            "ball_" + std::to_string(ballIndex),
            radius,
            0.5,
            Eigen::Vector3d(px, py, pz),
            color));
        ++ballIndex;
      }
    }
  }

  return world;
}

WorldPtr createDominosScenario()
{
  auto world = World::create("dominos");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  world->addSkeleton(createGround());

  double width = 0.05;
  double height = 0.3;
  double depth = 0.15;
  double spacing = 0.12;
  int count = 20;

  for (int i = 0; i < count; ++i) {
    double x = (i - count / 2.0) * spacing;
    double hue = static_cast<double>(i) / count;
    Eigen::Vector3d color(0.2 + 0.6 * hue, 0.3, 0.8 - 0.5 * hue);

    auto domino = createBox(
        "domino_" + std::to_string(i),
        Eigen::Vector3d(width, height, depth),
        0.5,
        Eigen::Vector3d(x, height / 2.0, 0),
        color);

    if (i == 0) {
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation() = Eigen::Vector3d(x, height / 2.0, 0);
      Eigen::AngleAxisd rotation(0.3, Eigen::Vector3d::UnitZ());
      tf.linear() = rotation.toRotationMatrix();
      domino->getBodyNode(0)->getParentJoint()->setTransformFromParentBodyNode(
          tf);
    }

    world->addSkeleton(domino);
  }

  return world;
}

WorldPtr createInclinedPlaneScenario()
{
  auto world = World::create("inclined_plane");
  world->setGravity(Eigen::Vector3d(0, -9.81, 0));

  SkeletonPtr ramp = Skeleton::create("ramp");
  BodyNodePtr rampBody = ramp->createJointAndBodyNodePair<WeldJoint>().second;

  auto rampShape = std::make_shared<BoxShape>(Eigen::Vector3d(3.0, 0.1, 1.0));
  auto rampNode = rampBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(rampShape);
  rampNode->getVisualAspect()->setColor(Eigen::Vector3d(0.5, 0.5, 0.5));

  double angle = 0.4;
  Eigen::Isometry3d rampTf = Eigen::Isometry3d::Identity();
  rampTf.translation() = Eigen::Vector3d(0, 0.5, 0);
  Eigen::AngleAxisd rampRotation(angle, Eigen::Vector3d::UnitZ());
  rampTf.linear() = rampRotation.toRotationMatrix();
  rampBody->getParentJoint()->setTransformFromParentBodyNode(rampTf);

  rampNode->getDynamicsAspect()->setFrictionCoeff(0.5);
  world->addSkeleton(ramp);

  double boxSize = 0.2;
  auto box = createBox(
      "sliding_box",
      Eigen::Vector3d(boxSize, boxSize, boxSize),
      1.0,
      Eigen::Vector3d(
          -1.0 * std::cos(angle) + 0.5 * std::sin(angle),
          1.0 * std::sin(angle) + 0.5 * std::cos(angle) + boxSize / 2.0,
          0),
      Eigen::Vector3d(0.8, 0.6, 0.2));
  box->getBodyNode(0)->getShapeNode(0)->getDynamicsAspect()->setFrictionCoeff(
      0.3);
  world->addSkeleton(box);

  world->addSkeleton(createGround());

  return world;
}

WorldPtr createScenario(Scenario scenario)
{
  switch (scenario) {
    case Scenario::MassRatio:
      return createMassRatioScenario();
    case Scenario::BoxStack:
      return createBoxStackScenario();
    case Scenario::BallDrop:
      return createBallDropScenario();
    case Scenario::Dominos:
      return createDominosScenario();
    case Scenario::InclinedPlane:
      return createInclinedPlaneScenario();
  }
  return createMassRatioScenario();
}

void listScenarios()
{
  std::cout << "Available scenarios:\n";
  for (const auto& s : GetScenarios()) {
    std::cout << "  " << s.name << ": " << s.description << "\n";
  }
}

Scenario parseScenario(const std::string& name)
{
  for (const auto& s : GetScenarios()) {
    if (s.name == name)
      return s.type;
  }
  return Scenario::MassRatio;
}

int runHeadless(
    Scenario scenario,
    int frames,
    const std::string& outDir,
    int width,
    int height)
{
  namespace fs = std::filesystem;

  if (!outDir.empty()) {
    std::error_code ec;
    fs::create_directories(outDir, ec);
    if (ec) {
      std::cerr << "Failed to create output directory: " << ec.message()
                << "\n";
      return EXIT_FAILURE;
    }
  }

  auto world = createScenario(scenario);
  world->setTimeStep(1.0 / 1000.0);

  Viewer viewer(ViewerConfig::headless(width, height));
  auto shadow = WorldNode::createDefaultShadowTechnique(&viewer);
  osg::ref_ptr<RealTimeWorldNode> worldNode
      = new RealTimeWorldNode(world, shadow);
  viewer.addWorldNode(worldNode);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 2.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.3f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  if (!outDir.empty()) {
    viewer.record(outDir, "frame_", false, 6);
  }

  std::cout << "Running scenario: " << static_cast<int>(scenario) << "\n";
  std::cout << "Frames: " << frames << ", Size: " << width << "x" << height
            << "\n";
  if (!outDir.empty()) {
    std::cout << "Saving to: " << outDir << "\n";
  }

  viewer.simulate(true);

  double simTime = 0.0;
  for (int i = 0; i < frames; ++i) {
    viewer.frame();
    simTime = world->getTime();
  }

  std::cout << "Completed " << frames << " frames, sim time: " << std::fixed
            << std::setprecision(2) << simTime << "s\n";

  return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char* argv[])
{
  CLI::App app("LCP Physics - Challenging simulation scenarios");
  bool headless = false;
  bool listMode = false;
  std::string scenarioName = "mass_ratio";
  int frames = 300;
  std::string outDir;
  int width = 1280;
  int height = 720;
  double guiScale = 1.0;

  app.add_flag("--headless", headless, "Run without display window");
  app.add_flag("--list", listMode, "List available scenarios");
  app.add_option(
      "--scenario", scenarioName, "Scenario to run (default: mass_ratio)");
  app.add_option("--frames", frames, "Number of frames")
      ->check(CLI::PositiveNumber);
  app.add_option("--out", outDir, "Output directory for frame capture");
  app.add_option("--width", width, "Viewport width")
      ->check(CLI::PositiveNumber);
  app.add_option("--height", height, "Viewport height")
      ->check(CLI::PositiveNumber);
  app.add_option("--gui-scale", guiScale, "GUI scale factor")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  if (listMode) {
    listScenarios();
    return EXIT_SUCCESS;
  }

  Scenario scenario = parseScenario(scenarioName);

  if (headless) {
    return runHeadless(scenario, frames, outDir, width, height);
  }

  auto world = createScenario(scenario);
  world->setTimeStep(1.0 / 1000.0);

  ImGuiViewer viewer;
  viewer.setImGuiScale(static_cast<float>(guiScale));
  auto shadow = WorldNode::createDefaultShadowTechnique(&viewer);
  osg::ref_ptr<RealTimeWorldNode> worldNode
      = new RealTimeWorldNode(world, shadow);
  viewer.addWorldNode(worldNode);

  const int windowWidth = static_cast<int>(1280 * guiScale);
  const int windowHeight = static_cast<int>(720 * guiScale);
  viewer.setUpViewInWindow(100, 100, windowWidth, windowHeight);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 2.0f, 3.0f),
      ::osg::Vec3(0.0f, 0.3f, 0.0f),
      ::osg::Vec3(0.0f, 1.0f, 0.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  std::cout << "LCP Physics Example\n";
  std::cout << "Scenario: " << scenarioName << "\n";
  std::cout << "Press SPACE to start/stop simulation\n";

  return viewer.run();
}
