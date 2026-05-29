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
  scenes.push_back(
      {"heightmap",
       "Heightmap",
       "Visualization",
       "An interactive procedural heightmap with a configurable grid.",
       &makeHeightmapScene});
  scenes.push_back(
      {"point_cloud",
       "Point Cloud",
       "Visualization",
       "Animated point-cloud and voxel-grid sensor rendering on a robot.",
       &makePointCloudScene});

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
  scenes.push_back(
      {"rigid_shapes",
       "Rigid Shapes",
       "Rigid Body",
       "Spawn assorted rigid shapes with a contact-point cloud.",
       &makeRigidShapesScene});
  // Experimental physics solver (sx::World). Kept in one dedicated category
  // so users can browse the experimental surface without sifting through the
  // legacy Rigid Body / Soft Bodies trees.
  scenes.push_back(
      {"experimental_rigid_body",
       "Rigid Body (Experimental)",
       "Experimental",
       "Falling rigid bodies on the experimental physics solver.",
       &makeExperimentalRigidBodyScene});

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
  scenes.push_back(
      {"lcp_physics",
       "LCP Physics",
       "Constraints & Joints",
       "LCP solver benchmark scenarios comparing Dantzig and PGS.",
       &makeLcpPhysicsScene});
  scenes.push_back(
      {"tinkertoy",
       "Tinkertoy",
       "Constraints & Joints",
       "Interactively assemble jointed structures from blocks.",
       &makeTinkertoyScene});
  scenes.push_back(
      {"human_joint_limits",
       "Human Joint Limits",
       "Constraints & Joints",
       "A human skeleton with neural-network joint-limit constraints.",
       &makeHumanJointLimitsScene});

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
  scenes.push_back(
      {"biped_stand",
       "Biped Stand",
       "Control & IK",
       "A biped balancing with SPD control under push perturbations.",
       &makeBipedStandScene});
  scenes.push_back(
      {"operational_space_control",
       "Operational Space Control",
       "Control & IK",
       "A KR5 arm tracks a draggable target via operational-space control.",
       &makeOperationalSpaceControlScene});
  scenes.push_back(
      {"atlas_puppet",
       "Atlas Puppet",
       "Control & IK",
       "Atlas whole-body IK puppet with balance and support overlays.",
       &makeAtlasPuppetScene});
  scenes.push_back(
      {"hubo_puppet",
       "Hubo Puppet",
       "Control & IK",
       "Hubo puppet driven by analytical arm and leg IK.",
       &makeHuboPuppetScene});

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
  scenes.push_back(
      {"experimental_deformable",
       "Deformable Body (Experimental)",
       "Experimental",
       "A spring-net deformable body on the experimental physics solver.",
       &makeExperimentalDeformableScene});

  // Robots
  scenes.push_back(
      {"g1_puppet",
       "G1 Puppet",
       "Robots",
       "A Unitree G1 whole-body IK puppet (loads a remote model).",
       &makeG1PuppetScene});
  scenes.push_back(
      {"fetch",
       "Fetch",
       "Robots",
       "A Fetch robot's end effector follows an interactive target.",
       &makeFetchScene});
  scenes.push_back(
      {"vehicle",
       "Vehicle",
       "Robots",
       "Drive a steerable car past obstacles.",
       &makeVehicleScene});

  return scenes;
}

} // namespace dart::examples::demos
