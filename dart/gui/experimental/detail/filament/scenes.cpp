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

#include "scenes.hpp"

#include "scene_fixtures.hpp"

#include <dart/gui/renderable.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <dart/common/uri.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>

#include <cmath>
#include <cstdlib>

namespace dart::gui::filament {
namespace {

std::optional<std::string> getLastPathSegment(std::string value)
{
  if (value.empty()) {
    return std::nullopt;
  }

  const std::size_t terminator = value.find_first_of("?#");
  if (terminator != std::string::npos) {
    value.erase(terminator);
  }

  while (value.ends_with('/') || value.ends_with('\\')) {
    value.pop_back();
  }

  if (value.empty()) {
    return std::nullopt;
  }

  const std::size_t slash = value.find_last_of("/\\");
  std::string segment
      = value.substr(slash == std::string::npos ? 0 : slash + 1);

  if (segment.empty()) {
    return std::nullopt;
  }

  return segment;
}

std::optional<std::string> inferPackageNameFromRobotUri(
    const std::string& robotUri)
{
  if (robotUri.empty()) {
    return std::nullopt;
  }

  dart::common::Uri uri;
  if (!uri.fromStringOrPath(robotUri)) {
    return std::nullopt;
  }

  if (!uri.mScheme || *uri.mScheme != "package" || !uri.mAuthority) {
    return std::nullopt;
  }

  return uri.mAuthority.get();
}

std::optional<std::string> inferPackageNameFromPackageUri(
    const std::string& packageUri)
{
  if (packageUri.empty()) {
    return std::nullopt;
  }

  dart::common::Uri uri;
  if (uri.fromStringOrPath(packageUri)) {
    if (uri.mScheme && *uri.mScheme == "package" && uri.mAuthority) {
      return uri.mAuthority.get();
    }

    if (uri.mPath) {
      if (auto segment = getLastPathSegment(uri.mPath.get())) {
        return segment;
      }
    }
  }

  return getLastPathSegment(packageUri);
}

} // namespace

const char* sceneName(ExampleScene scene)
{
  switch (scene) {
    case ExampleScene::Mvp:
      return "mvp";
    case ExampleScene::HelloWorld:
      return "hello-world";
    case ExampleScene::Boxes:
      return "boxes";
    case ExampleScene::HardcodedDesign:
      return "hardcoded-design";
    case ExampleScene::RigidChain:
      return "rigid-chain";
    case ExampleScene::RigidLoop:
      return "rigid-loop";
    case ExampleScene::MixedChain:
      return "mixed-chain";
    case ExampleScene::CouplerConstraint:
      return "coupler-constraint";
    case ExampleScene::AddDeleteSkels:
      return "add-delete-skels";
    case ExampleScene::Vehicle:
      return "vehicle";
    case ExampleScene::HybridDynamics:
      return "hybrid-dynamics";
    case ExampleScene::JointConstraints:
      return "joint-constraints";
    case ExampleScene::FreeJointCases:
      return "free-joint-cases";
    case ExampleScene::HumanJointLimits:
      return "human-joint-limits";
    case ExampleScene::LcpPhysics:
      return "lcp-physics";
    case ExampleScene::MimicPendulums:
      return "mimic-pendulums";
    case ExampleScene::AtlasPuppet:
      return "atlas-puppet";
    case ExampleScene::HuboPuppet:
      return "hubo-puppet";
    case ExampleScene::AtlasSimbicon:
      return "atlas-simbicon";
    case ExampleScene::OperationalSpaceControl:
      return "operational-space-control";
    case ExampleScene::WamIkFast:
      return "wam-ikfast";
    case ExampleScene::Fetch:
      return "fetch";
    case ExampleScene::Tinkertoy:
      return "tinkertoy";
    case ExampleScene::DragAndDrop:
      return "drag-and-drop";
    case ExampleScene::SimpleFrames:
      return "simple-frames";
    case ExampleScene::SoftBodies:
      return "soft-bodies";
    case ExampleScene::PointCloud:
      return "point-cloud";
    case ExampleScene::CapsuleGroundContact:
      return "capsule-ground-contact";
    case ExampleScene::SimulationEventHandler:
      return "simulation-event-handler";
    case ExampleScene::Polyhedron:
      return "polyhedron";
    case ExampleScene::Heightmap:
      return "heightmap";
    case ExampleScene::G1:
      return "g1";
  }
  return "mvp";
}

bool parseSceneName(std::string_view name, ExampleScene& scene)
{
  if (name == "mvp") {
    scene = ExampleScene::Mvp;
    return true;
  }
  if (name == "hello-world") {
    scene = ExampleScene::HelloWorld;
    return true;
  }
  if (name == "boxes") {
    scene = ExampleScene::Boxes;
    return true;
  }
  if (name == "hardcoded-design") {
    scene = ExampleScene::HardcodedDesign;
    return true;
  }
  if (name == "rigid-chain") {
    scene = ExampleScene::RigidChain;
    return true;
  }
  if (name == "rigid-loop") {
    scene = ExampleScene::RigidLoop;
    return true;
  }
  if (name == "mixed-chain") {
    scene = ExampleScene::MixedChain;
    return true;
  }
  if (name == "coupler-constraint") {
    scene = ExampleScene::CouplerConstraint;
    return true;
  }
  if (name == "add-delete-skels") {
    scene = ExampleScene::AddDeleteSkels;
    return true;
  }
  if (name == "vehicle") {
    scene = ExampleScene::Vehicle;
    return true;
  }
  if (name == "hybrid-dynamics") {
    scene = ExampleScene::HybridDynamics;
    return true;
  }
  if (name == "joint-constraints") {
    scene = ExampleScene::JointConstraints;
    return true;
  }
  if (name == "free-joint-cases") {
    scene = ExampleScene::FreeJointCases;
    return true;
  }
  if (name == "human-joint-limits") {
    scene = ExampleScene::HumanJointLimits;
    return true;
  }
  if (name == "lcp-physics") {
    scene = ExampleScene::LcpPhysics;
    return true;
  }
  if (name == "mimic-pendulums") {
    scene = ExampleScene::MimicPendulums;
    return true;
  }
  if (name == "atlas-puppet") {
    scene = ExampleScene::AtlasPuppet;
    return true;
  }
  if (name == "hubo-puppet") {
    scene = ExampleScene::HuboPuppet;
    return true;
  }
  if (name == "atlas-simbicon") {
    scene = ExampleScene::AtlasSimbicon;
    return true;
  }
  if (name == "operational-space-control") {
    scene = ExampleScene::OperationalSpaceControl;
    return true;
  }
  if (name == "wam-ikfast") {
    scene = ExampleScene::WamIkFast;
    return true;
  }
  if (name == "fetch") {
    scene = ExampleScene::Fetch;
    return true;
  }
  if (name == "tinkertoy") {
    scene = ExampleScene::Tinkertoy;
    return true;
  }
  if (name == "drag-and-drop") {
    scene = ExampleScene::DragAndDrop;
    return true;
  }
  if (name == "simple-frames") {
    scene = ExampleScene::SimpleFrames;
    return true;
  }
  if (name == "soft-bodies") {
    scene = ExampleScene::SoftBodies;
    return true;
  }
  if (name == "point-cloud") {
    scene = ExampleScene::PointCloud;
    return true;
  }
  if (name == "capsule-ground-contact") {
    scene = ExampleScene::CapsuleGroundContact;
    return true;
  }
  if (name == "simulation-event-handler") {
    scene = ExampleScene::SimulationEventHandler;
    return true;
  }
  if (name == "polyhedron") {
    scene = ExampleScene::Polyhedron;
    return true;
  }
  if (name == "heightmap") {
    scene = ExampleScene::Heightmap;
    return true;
  }
  if (name == "g1") {
    scene = ExampleScene::G1;
    return true;
  }
  return false;
}

dart::gui::OrbitCamera initialCameraForScene(ExampleScene scene)
{
  dart::gui::OrbitCamera camera;
  switch (scene) {
    case ExampleScene::DragAndDrop:
      camera.target = Eigen::Vector3d(0.35, 0.15, 0.9);
      camera.yaw = -0.72;
      camera.pitch = 0.58;
      camera.distance = 9.5;
      break;
    case ExampleScene::HelloWorld:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.5);
      camera.yaw = -0.88;
      camera.pitch = 0.36;
      camera.distance = 4.2;
      break;
    case ExampleScene::Boxes:
      camera.target = Eigen::Vector3d(0.0, 0.0, 4.0);
      camera.yaw = -0.78;
      camera.pitch = 0.44;
      camera.distance = 22.0;
      break;
    case ExampleScene::HardcodedDesign:
      camera.target = Eigen::Vector3d(0.0, 0.0, 1.0);
      camera.yaw = -0.78;
      camera.pitch = 0.42;
      camera.distance = 4.0;
      break;
    case ExampleScene::RigidChain:
      camera.target = Eigen::Vector3d(0.0, -0.45, 0.0);
      camera.yaw = -0.72;
      camera.pitch = 0.35;
      camera.distance = 2.4;
      break;
    case ExampleScene::RigidLoop:
      camera.target = Eigen::Vector3d(0.0, -0.35, 0.0);
      camera.yaw = -0.72;
      camera.pitch = 0.35;
      camera.distance = 2.6;
      break;
    case ExampleScene::MixedChain:
      camera.target = Eigen::Vector3d(0.0, 2.1, 0.0);
      camera.yaw = -0.72;
      camera.pitch = 0.38;
      camera.distance = 7.0;
      break;
    case ExampleScene::CouplerConstraint:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.05);
      camera.yaw = -0.77;
      camera.pitch = 0.52;
      camera.distance = 2.4;
      break;
    case ExampleScene::AddDeleteSkels:
      camera.target = Eigen::Vector3d(0.0, 0.65, 0.0);
      camera.yaw = -0.78;
      camera.pitch = 0.42;
      camera.distance = 3.3;
      break;
    case ExampleScene::Vehicle:
      camera.target = Eigen::Vector3d(0.15, 0.45, 0.0);
      camera.yaw = -0.82;
      camera.pitch = 0.34;
      camera.distance = 6.0;
      break;
    case ExampleScene::HybridDynamics:
      camera.target = Eigen::Vector3d(0.0, 0.15, 0.0);
      camera.yaw = -0.82;
      camera.pitch = 0.32;
      camera.distance = 4.2;
      break;
    case ExampleScene::JointConstraints:
      camera.target = Eigen::Vector3d(0.0, 0.45, 0.0);
      camera.yaw = -0.82;
      camera.pitch = 0.32;
      camera.distance = 4.2;
      break;
    case ExampleScene::FreeJointCases:
      camera.target = Eigen::Vector3d(4.0, 0.0, 0.2);
      camera.yaw = -1.05;
      camera.pitch = 0.34;
      camera.distance = 10.5;
      break;
    case ExampleScene::HumanJointLimits:
      camera.target = Eigen::Vector3d(0.0, -0.25, 0.0);
      camera.yaw = -0.72;
      camera.pitch = 0.28;
      camera.distance = 3.0;
      break;
    case ExampleScene::LcpPhysics:
      camera.target = Eigen::Vector3d(0.0, 0.45, 0.0);
      camera.yaw = -0.70;
      camera.pitch = 0.36;
      camera.distance = 5.0;
      break;
    case ExampleScene::MimicPendulums:
      camera.target = Eigen::Vector3d(0.25, 3.0, 1.15);
      camera.yaw = -0.82;
      camera.pitch = 0.34;
      camera.distance = 8.0;
      break;
    case ExampleScene::AtlasPuppet:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.95);
      camera.yaw = -0.72;
      camera.pitch = 0.28;
      camera.distance = 3.8;
      break;
    case ExampleScene::HuboPuppet:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.85);
      camera.yaw = -0.76;
      camera.pitch = 0.30;
      camera.distance = 3.4;
      break;
    case ExampleScene::AtlasSimbicon:
      camera.target = Eigen::Vector3d(0.0, 0.85, 0.0);
      camera.yaw = -0.68;
      camera.pitch = 0.32;
      camera.distance = 5.5;
      break;
    case ExampleScene::OperationalSpaceControl:
      camera.target = Eigen::Vector3d(0.25, 0.0, 0.55);
      camera.yaw = -0.78;
      camera.pitch = 0.26;
      camera.distance = 2.7;
      break;
    case ExampleScene::WamIkFast:
      camera.target = Eigen::Vector3d(0.2, 0.0, 0.55);
      camera.yaw = -0.78;
      camera.pitch = 0.28;
      camera.distance = 3.0;
      break;
    case ExampleScene::Fetch:
      camera.target = Eigen::Vector3d(0.6, 0.35, 0.45);
      camera.yaw = -0.78;
      camera.pitch = 0.32;
      camera.distance = 4.8;
      break;
    case ExampleScene::Tinkertoy:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.25);
      camera.yaw = -0.72;
      camera.pitch = 0.34;
      camera.distance = 3.1;
      break;
    case ExampleScene::SimpleFrames:
      camera.target = Eigen::Vector3d(0.05, 0.0, 0.06);
      camera.yaw = -0.70;
      camera.pitch = 0.48;
      camera.distance = 0.75;
      break;
    case ExampleScene::SoftBodies:
      camera.target = Eigen::Vector3d(0.0, 0.05, 0.0);
      camera.yaw = -0.72;
      camera.pitch = 0.50;
      camera.distance = 2.8;
      break;
    case ExampleScene::PointCloud:
      camera.target = Eigen::Vector3d(0.0, 0.05, 0.25);
      camera.yaw = -0.72;
      camera.pitch = 0.46;
      camera.distance = 2.3;
      break;
    case ExampleScene::CapsuleGroundContact:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.25);
      camera.yaw = -0.74;
      camera.pitch = 0.36;
      camera.distance = 3.4;
      break;
    case ExampleScene::SimulationEventHandler:
      camera.target = Eigen::Vector3d(0.0, 0.2, 1.3);
      camera.yaw = -0.78;
      camera.pitch = 0.42;
      camera.distance = 6.0;
      break;
    case ExampleScene::Polyhedron:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.45);
      camera.yaw = -0.78;
      camera.pitch = 0.42;
      camera.distance = 3.0;
      break;
    case ExampleScene::Heightmap:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.25);
      camera.yaw = -0.74;
      camera.pitch = 0.48;
      camera.distance = 4.3;
      break;
    case ExampleScene::G1:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.85);
      camera.yaw = -0.78;
      camera.pitch = 0.24;
      camera.distance = 3.4;
      break;
    case ExampleScene::Mvp:
      camera.target = Eigen::Vector3d(0.15, 0.55, 0.75);
      camera.yaw = -0.95;
      camera.pitch = 0.38;
      camera.distance = 7.2;
      break;
  }
  return camera;
}

AppOptions parseOptions(int argc, char* argv[])
{
  AppOptions options;
  bool g1PackageNameExplicit = false;
  bool g1PackageUriExplicit = false;
  bool g1RobotUriExplicit = false;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--frames" && i + 1 < argc) {
      options.run.maxFrames = std::atoi(argv[++i]);
    } else if (arg == "--width" && i + 1 < argc) {
      options.run.width = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--height" && i + 1 < argc) {
      options.run.height = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--screenshot" && i + 1 < argc) {
      options.run.screenshotPath = argv[++i];
    } else if (arg == "--out" && i + 1 < argc) {
      options.run.frameOutputDirectory = argv[++i];
    } else if (arg == "--headless") {
      options.run.headless = true;
    } else if (arg == "--hide-ui") {
      options.showUi = false;
      options.showUiExplicit = true;
    } else if (arg == "--show-ui") {
      options.showUi = true;
      options.showUiExplicit = true;
    } else if (arg == "--orbit-light") {
      options.orbitLight = true;
    } else if (arg == "--no-orbit-light") {
      options.orbitLight = false;
    } else if (arg == "--orbit-light-period" && i + 1 < argc) {
      char* end = nullptr;
      const char* value = argv[++i];
      const double orbitLightPeriodSeconds = std::strtod(value, &end);
      if (end == value || *end != '\0'
          || !std::isfinite(orbitLightPeriodSeconds)
          || orbitLightPeriodSeconds <= 0.0) {
        std::cerr << "Invalid --orbit-light-period value '" << value
                  << "'. Expected a positive number of seconds.\n";
        std::exit(2);
      }
      options.orbitLightPeriodSeconds = orbitLightPeriodSeconds;
    } else if (arg == "--gui-scale" && i + 1 < argc) {
      char* end = nullptr;
      const char* value = argv[++i];
      const float guiScale = std::strtof(value, &end);
      if (end == value || *end != '\0' || !std::isfinite(guiScale)
          || guiScale <= 0.0f) {
        std::cerr << "Invalid --gui-scale value '" << value
                  << "'. Expected a positive number.\n";
        std::exit(2);
      }
      options.run.guiScale
          = std::clamp(static_cast<double>(guiScale), 0.5, 4.0);
    } else if (arg == "--profile") {
      options.profile = true;
    } else if (arg == "--scene" && i + 1 < argc) {
      const std::string_view sceneArg(argv[++i]);
      if (!parseSceneName(sceneArg, options.scene)) {
        std::cerr << "Unknown scene '" << sceneArg
                  << "'. Expected 'mvp', 'hello-world', 'boxes', "
                     "'hardcoded-design', 'rigid-chain', 'rigid-loop', "
                     "'mixed-chain', 'coupler-constraint', "
                     "'add-delete-skels', 'vehicle', 'hybrid-dynamics', "
                     "'joint-constraints', 'free-joint-cases', "
                     "'human-joint-limits', 'lcp-physics', "
                     "'mimic-pendulums', 'atlas-puppet', 'hubo-puppet', "
                     "'atlas-simbicon', 'operational-space-control', "
                     "'wam-ikfast', 'fetch', 'tinkertoy', 'drag-and-drop', "
                     "'simple-frames', 'soft-bodies', 'point-cloud', "
                     "'capsule-ground-contact', "
                     "'simulation-event-handler', 'polyhedron', "
                     "'heightmap', or 'g1'.\n";
        std::exit(2);
      }
    } else if (
        (arg == "--g1-package-uri" || arg == "--package-uri") && i + 1 < argc) {
      options.g1PackageUri = argv[++i];
      g1PackageUriExplicit = true;
    } else if (
        (arg == "--g1-robot-uri" || arg == "--robot-uri") && i + 1 < argc) {
      options.g1RobotUri = argv[++i];
      g1RobotUriExplicit = true;
    } else if (
        (arg == "--g1-package-name" || arg == "--package-name")
        && i + 1 < argc) {
      options.g1PackageName = argv[++i];
      g1PackageNameExplicit = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: " << argv[0]
          << " [--frames N] [--width N] [--height N]"
             " [--screenshot PATH] [--out DIR] [--headless]"
             " [--hide-ui|--show-ui]"
             " [--orbit-light|--no-orbit-light]"
             " [--orbit-light-period SECONDS]"
             " [--gui-scale N]"
             " [--profile]"
             " [--scene "
             "mvp|hello-world|boxes|hardcoded-design|rigid-chain|rigid-loop|"
             "mixed-chain|coupler-constraint|add-delete-skels|vehicle|hybrid-"
             "dynamics|joint-constraints|free-joint-cases|human-joint-limits|"
             "lcp-physics|mimic-pendulums|atlas-puppet|hubo-puppet|atlas-"
             "simbicon|operational-space-control|wam-ikfast|fetch|tinkertoy|"
             "drag-and-drop|simple-frames|soft-bodies|point-cloud|capsule-"
             "ground-contact|simulation-event-handler|polyhedron|heightmap|g1]"
             " [--g1-package-uri URI] [--g1-robot-uri URI]"
             " [--g1-package-name NAME]\n";
      std::exit(0);
    }
  }

  if (!g1PackageNameExplicit) {
    if (g1RobotUriExplicit) {
      if (auto packageName = inferPackageNameFromRobotUri(options.g1RobotUri)) {
        options.g1PackageName = *packageName;
      }
    } else if (g1PackageUriExplicit) {
      if (auto packageName
          = inferPackageNameFromPackageUri(options.g1PackageUri)) {
        options.g1PackageName = *packageName;
      }
    } else if (
        auto packageName = inferPackageNameFromRobotUri(options.g1RobotUri)) {
      options.g1PackageName = *packageName;
    }
  }

  dart::gui::normalizeRunOptions(options.run);
  if (options.run.guiScale != 1.0) {
    options.run.width = std::max(
        1,
        static_cast<int>(std::lround(
            static_cast<double>(options.run.width) * options.run.guiScale)));
    options.run.height = std::max(
        1,
        static_cast<int>(std::lround(
            static_cast<double>(options.run.height) * options.run.guiScale)));
  }
  if (options.run.headless && !options.showUiExplicit) {
    options.showUi = false;
  }
  return options;
}

DartScene createDartScene(const AppOptions& options)
{
  if (options.world != nullptr) {
    DartScene scene;
    scene.world = options.world;
    scene.preStep = options.preStep;
    scene.ikHandles.reserve(options.ikHandles.size());
    for (const auto& handle : options.ikHandles) {
      if (handle.target == nullptr) {
        continue;
      }

      IkHandle runtimeHandle;
      runtimeHandle.targetRenderableId
          = dart::gui::makeRenderableId(*handle.target);
      runtimeHandle.label = handle.label;
      runtimeHandle.hotkey = handle.hotkey;
      runtimeHandle.target = handle.target;
      runtimeHandle.ik = handle.ik;
      scene.ikHandles.push_back(std::move(runtimeHandle));
    }
    return scene;
  }

  switch (options.scene) {
    case ExampleScene::Mvp:
      return createMvpDartScene();
    case ExampleScene::HelloWorld:
      return createHelloWorldScene();
    case ExampleScene::Boxes:
      return createBoxesScene();
    case ExampleScene::HardcodedDesign:
      return createHardcodedDesignScene();
    case ExampleScene::RigidChain:
      return createRigidChainScene();
    case ExampleScene::RigidLoop:
      return createRigidLoopScene();
    case ExampleScene::MixedChain:
      return createMixedChainScene();
    case ExampleScene::CouplerConstraint:
      return createCouplerConstraintScene();
    case ExampleScene::AddDeleteSkels:
      return createAddDeleteSkelsScene();
    case ExampleScene::Vehicle:
      return createVehicleScene();
    case ExampleScene::HybridDynamics:
      return createHybridDynamicsScene();
    case ExampleScene::JointConstraints:
      return createJointConstraintsScene();
    case ExampleScene::FreeJointCases:
      return createFreeJointCasesScene();
    case ExampleScene::HumanJointLimits:
      return createHumanJointLimitsScene();
    case ExampleScene::LcpPhysics:
      return createLcpPhysicsScene();
    case ExampleScene::MimicPendulums:
      return createMimicPendulumsScene();
    case ExampleScene::AtlasPuppet:
      return createAtlasPuppetScene();
    case ExampleScene::HuboPuppet:
      return createHuboPuppetScene();
    case ExampleScene::AtlasSimbicon:
      return createAtlasSimbiconScene();
    case ExampleScene::OperationalSpaceControl:
      return createOperationalSpaceControlScene();
    case ExampleScene::WamIkFast:
      return createWamIkFastScene();
    case ExampleScene::Fetch:
      return createFetchScene();
    case ExampleScene::Tinkertoy:
      return createTinkertoyScene();
    case ExampleScene::DragAndDrop:
      return createDragAndDropScene();
    case ExampleScene::SimpleFrames:
      return createSimpleFramesScene();
    case ExampleScene::SoftBodies:
      return createSoftBodiesScene();
    case ExampleScene::PointCloud:
      return createPointCloudScene();
    case ExampleScene::CapsuleGroundContact:
      return createCapsuleGroundContactScene();
    case ExampleScene::SimulationEventHandler:
      return createSimulationEventHandlerScene();
    case ExampleScene::Polyhedron:
      return createPolyhedronScene();
    case ExampleScene::Heightmap:
      return createHeightmapScene();
    case ExampleScene::G1:
      return createG1DartScene(options);
  }
  return createMvpDartScene();
}

} // namespace dart::gui::filament
