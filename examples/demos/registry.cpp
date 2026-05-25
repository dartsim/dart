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

namespace dart::examples::demos {

std::vector<dart::gui::DemoSceneEntry> makeDemoScenes()
{
  std::vector<dart::gui::DemoSceneEntry> scenes;

  // Getting Started
  scenes.push_back(
      {"hello_world",
       "Hello World",
       "Getting Started",
       "A single box falling onto a ground plane.",
       &makeHelloWorldScene});
  scenes.push_back(
      {"empty",
       "Empty Scaffold",
       "Getting Started",
       "Minimal frame hierarchy with drag controls and lifecycle hooks.",
       &makeEmptyScene});

  // Visualization
  scenes.push_back(
      {"shapes",
       "Shapes",
       "Visualization",
       "Assorted primitive shapes resting on the ground.",
       &makeShapesScene});
  scenes.push_back(
      {"simple_frames",
       "Simple Frames",
       "Visualization",
       "Nested SimpleFrames with ellipsoid markers and an arrow shape.",
       &makeSimpleFramesScene});
  scenes.push_back(
      {"drag_and_drop",
       "Drag and Drop",
       "Visualization",
       "Drag and rotate a frame with a gizmo; click renderables to select.",
       &makeDragAndDropScene});
  scenes.push_back(
      {"polyhedron_visual",
       "Polyhedron",
       "Visualization",
       "A convex polyhedron rendered as surface and wireframe.",
       &makePolyhedronVisualScene});
  scenes.push_back(
      {"imgui",
       "ImGui Panels",
       "Visualization",
       "Custom ImGui panels and a gizmo-draggable target frame.",
       &makeImguiScene});

  // Rigid Body
  scenes.push_back(
      {"boxes",
       "Boxes",
       "Rigid Body",
       "A grid of rigid boxes dropped onto the ground.",
       &makeBoxesScene});
  scenes.push_back(
      {"rigid_cubes",
       "Rigid Cubes",
       "Rigid Body",
       "Apply directional forces to a stack of rigid cubes.",
       &makeRigidCubesScene});
  scenes.push_back(
      {"add_delete_skels",
       "Add / Delete Skeletons",
       "Rigid Body",
       "Spawn and delete dynamic cubes at runtime.",
       &makeAddDeleteSkelsScene});
  scenes.push_back(
      {"rigid_chain",
       "Rigid Chain",
       "Rigid Body",
       "A damped articulated chain loaded from a skeleton file.",
       &makeRigidChainScene});
  scenes.push_back(
      {"simulation_event_handler",
       "Simulation Events",
       "Rigid Body",
       "Sensor markers, force/torque arrows, and body selection.",
       &makeSimulationEventHandlerScene});

  // Collision
  scenes.push_back(
      {"capsule_ground_contact",
       "Capsule Ground Contact",
       "Collision",
       "A capsule settling on a plane; reset its orientation.",
       &makeCapsuleGroundContactScene});

  // Constraints & Joints
  scenes.push_back(
      {"hardcoded_design",
       "Hardcoded Design",
       "Constraints & Joints",
       "A hand-built revolute chain driven from the keyboard.",
       &makeHardcodedDesignScene});
  scenes.push_back(
      {"box_stacking",
       "Box Stacking",
       "Constraints & Joints",
       "Stacked boxes comparing Dantzig and PGS LCP solvers.",
       &makeBoxStackingScene});
  scenes.push_back(
      {"coupler_constraint",
       "Coupler Constraint",
       "Constraints & Joints",
       "A coupler constraint compared against a mimic motor.",
       &makeCouplerConstraintScene});
  scenes.push_back(
      {"mimic_pendulums",
       "Mimic Pendulums",
       "Constraints & Joints",
       "Mimic-joint pendulums compared against an uncoupled pair.",
       &makeMimicPendulumsScene});
  scenes.push_back(
      {"rigid_loop",
       "Rigid Loop",
       "Constraints & Joints",
       "A chain closed into a loop with a ball-joint constraint.",
       &makeRigidLoopScene});
  scenes.push_back(
      {"free_joint_cases",
       "Free Joint Cases",
       "Constraints & Joints",
       "FreeJoint integration cases compared against reference bodies.",
       &makeFreeJointCasesScene});

  // Control & IK
  scenes.push_back(
      {"hybrid_dynamics",
       "Hybrid Dynamics",
       "Control & IK",
       "A biped driven by scripted velocity commands with a harness lock.",
       &makeHybridDynamicsScene});
  scenes.push_back(
      {"joint_constraints",
       "Joint Constraints",
       "Control & IK",
       "Balanced full-body PD control with a harness constraint.",
       &makeJointConstraintsScene});

  // Soft Bodies
  scenes.push_back(
      {"mixed_chain",
       "Mixed Chain",
       "Soft Bodies",
       "Apply impulses to a soft link in a mixed rigid/soft chain.",
       &makeMixedChainScene});
  scenes.push_back(
      {"soft_bodies",
       "Soft Bodies",
       "Soft Bodies",
       "Soft-body simulation with recorded-state playback.",
       &makeSoftBodiesScene});

  // Robots
  scenes.push_back(
      {"vehicle",
       "Vehicle",
       "Robots",
       "Drive a steerable car past obstacles.",
       &makeVehicleScene});

  return scenes;
}

} // namespace dart::examples::demos
