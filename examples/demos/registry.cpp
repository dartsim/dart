/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

dart::gui::OrbitCamera makePlannedCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.12);
  camera.yaw = -0.65;
  camera.pitch = 0.28;
  camera.distance = 2.2;
  return camera;
}

dart::gui::ApplicationOptions makePlannedWorldPortScene(
    std::string id,
    std::string title,
    std::string legacySeeds,
    std::string target)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, 0.08);
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), id + "_marker", transform);
  frame->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(0.56, 0.56, 0.16)));
  frame->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.35, 0.56, 0.86, 1.0));

  dart::gui::Panel panel;
  panel.title = std::move(title);
  panel.build = [legacySeeds = std::move(legacySeeds),
                 target = std::move(target)](dart::gui::PanelBuilder& builder) {
    builder.text("status: planned World demo");
    builder.text("legacy seeds: " + legacySeeds);
    builder.separator();
    builder.text("target: " + target);
  };

  dart::gui::ApplicationOptions options;
  options.camera = makePlannedCamera();
  options.advanceSimulation = false;
  options.renderableProvider = [frame]() {
    std::vector<dart::gui::RenderableDescriptor> descriptors;
    if (auto descriptor = dart::gui::describeShapeFrame(*frame)) {
      descriptors.push_back(*descriptor);
    }
    return descriptors;
  };
  options.panels.push_back(std::move(panel));
  return options;
}

} // namespace

std::vector<dart::gui::DemoSceneEntry> makeDemoScenes()
{
  std::vector<dart::gui::DemoSceneEntry> scenes;

  scenes.push_back(
      {"rigid_body",
       "Rigid Body",
       "World Rigid Body",
       "Falling rigid bodies on the World solver.",
       &makeRigidBodyScene});
  scenes.push_back(
      {"planned_inverse_kinematics",
       "Inverse Kinematics",
       "Planned World Ports",
       "Placeholder for World-native IK targets and solver handles.",
       []() {
         return makePlannedWorldPortScene(
             "planned_inverse_kinematics",
             "Inverse Kinematics",
             "wam_ikfast, kr5_arm, atlas_ik",
             "interactive end-effector targets on World multibodies");
       }});
  scenes.push_back(
      {"planned_simbicon_walking",
       "SIMBICON Walking",
       "Planned World Ports",
       "Placeholder for World-native biped gait control.",
       []() {
         return makePlannedWorldPortScene(
             "planned_simbicon_walking",
             "SIMBICON Walking",
             "atlas_simbicon, g1_simbicon, biped_stand",
             "closed-loop walking controller on a World humanoid");
       }});
  scenes.push_back(
      {"planned_operational_space_control",
       "Operational Space Control",
       "Planned World Ports",
       "Placeholder for task-space control on World articulated bodies.",
       []() {
         return makePlannedWorldPortScene(
             "planned_operational_space_control",
             "Operational Space Control",
             "operational_space_control",
             "task-space Jacobian control with World dynamics diagnostics");
       }});
  scenes.push_back(
      {"planned_robot_puppets",
       "Robot Model Puppets",
       "Planned World Ports",
       "Placeholder for maintained robot model loading and pose demos.",
       []() {
         return makePlannedWorldPortScene(
             "planned_robot_puppets",
             "Robot Model Puppets",
             "atlas_puppet, g1_puppet, hubo_puppet",
             "asset-loaded humanoid puppets with World-friendly pose controls");
       }});
  scenes.push_back(
      {"planned_mobile_manipulation",
       "Mobile Manipulation",
       "Planned World Ports",
       "Placeholder for robot and vehicle workflows from the legacy demos.",
       []() {
         return makePlannedWorldPortScene(
             "planned_mobile_manipulation",
             "Mobile Manipulation",
             "fetch, vehicle",
             "mobile base and manipulator workflows using the World API");
       }});
  scenes.push_back(
      {"deformable_body",
       "Deformable Body",
       "IPC Deformable",
       "A spring-net deformable body on the World solver.",
       &makeDeformableBodyScene});
  scenes.push_back(
      {"vbd_deformable",
       "Deformable VBD",
       "Vertex Block Descent",
       "A contact-free hanging cloth driven by the Vertex Block Descent "
       "inner solver (graph-colored Gauss-Seidel block descent).",
       &makeVbdDeformableScene});

  return scenes;
}

} // namespace dart::examples::demos
