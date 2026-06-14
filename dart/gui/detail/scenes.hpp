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

#ifndef DART_GUI_DETAIL_SCENES_HPP_
#define DART_GUI_DETAIL_SCENES_HPP_

#include <dart/gui/application.hpp>
#include <dart/gui/detail/frame_viewport.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/scene.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::gui::detail {

struct IkHandle
{
  dart::gui::RenderableId targetRenderableId = 0;
  std::string label;
  int hotkey = 0;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  dart::dynamics::InverseKinematicsPtr ik;
  dart::gui::InverseKinematicsSolveMode solveMode
      = dart::gui::InverseKinematicsSolveMode::Target;
};

struct DartScene
{
  std::vector<IkHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
  std::function<std::vector<dart::gui::DebugLabelDescriptor>()> debugLabels;
  std::function<dart::gui::DebugScene()> debugProvider;
  std::vector<dart::gui::BodyNodeDragHandle> bodyNodeDragHandles;
  std::vector<dart::gui::KeyboardAction> keyboardActions;
  std::function<void()> preStep;
  std::function<void()> postStep;
  double timeStep = 1.0 / 60.0;
  double time = 0.0;
  std::size_t contactCount = 0u;
  bool advanceSimulation = true;
  dart::gui::RenderSettings renderSettings;
  std::function<std::vector<dart::gui::RenderableDescriptor>()>
      renderableProvider;
  std::function<dart::gui::RenderableSelection()> selectedRenderableProvider;
  std::function<void(dart::gui::RenderableId)> onRenderableSelected;
  std::function<void(const dart::gui::ForceDragEvent&)> onForceDrag;
  std::function<void(dart::gui::ViewportPaneKind)> onViewportPaneActivated;
  ViewportPaneActivationState viewportPaneActivation;
  bool dockingEnabled = false;
};

enum class ExampleScene
{
  Mvp,
  HelloWorld,
  Boxes,
  HardcodedDesign,
  RigidChain,
  RigidLoop,
  MixedChain,
  CouplerConstraint,
  AddDeleteSkels,
  Vehicle,
  HybridDynamics,
  JointConstraints,
  FreeJointCases,
  HumanJointLimits,
  LcpPhysics,
  MimicPendulums,
  AtlasPuppet,
  HuboPuppet,
  AtlasSimbicon,
  OperationalSpaceControl,
  WamIkFast,
  Fetch,
  Tinkertoy,
  DragAndDrop,
  SimpleFrames,
  SoftBodies,
  PointCloud,
  CapsuleGroundContact,
  SimulationEventHandler,
  Polyhedron,
  Heightmap,
  G1,
};

struct AppOptions
{
  dart::gui::RunOptions run;
  std::function<void()> preStep;
  std::function<void()> postStep;
  std::function<void()> preRender;
  std::function<void()> postRender;
  double timeStep = 1.0 / 60.0;
  bool advanceSimulation = true;
  std::optional<dart::gui::OrbitCamera> camera;
  std::function<dart::gui::OrbitCameraControlOptions()> cameraControlsProvider;
  std::function<bool(dart::gui::OrbitCamera&)> cameraUpdater;
  std::function<dart::gui::ViewportLayoutOptions(const dart::gui::OrbitCamera&)>
      viewportLayoutProvider;
  std::function<void(dart::gui::ViewportPaneKind)> onViewportPaneActivated;
  dart::gui::RenderSettings renderSettings;
  bool renderOutputModeExplicit = false;
  std::vector<dart::gui::Panel> panels;
  std::vector<dart::gui::Gizmo> gizmos;
  std::function<std::vector<dart::gui::DebugLabelDescriptor>()> debugLabels;
  std::function<dart::gui::DebugScene()> debugProvider;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::BodyNodeDragHandle> bodyNodeDragHandles;
  std::vector<dart::gui::KeyboardAction> keyboardActions;
  std::function<std::vector<dart::gui::RenderableDescriptor>()>
      renderableProvider;
  std::function<dart::gui::RenderableSelection()> selectedRenderableProvider;
  std::function<void(dart::gui::RenderableId)> onRenderableSelected;
  std::function<void(const dart::gui::ForceDragEvent&)> onForceDrag;
  bool dockingEnabled = false;
  ExampleScene scene = ExampleScene::Mvp;
  bool showUi = true;
  bool showUiExplicit = false;
  bool windowWidthExplicit = false;
  bool windowHeightExplicit = false;
  bool profile = false;
  bool showPerfHud = false;
  bool orbitLight = true;
  double orbitLightPeriodSeconds = 80.0;
  std::string g1PackageName = "g1_description";
  std::string g1PackageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string g1RobotUri = "package://g1_description/g1_29dof.urdf";
};

inline constexpr const char* kWamFixtureSkeletonName = "visual_wam_robot";
inline constexpr const char* kOperationalSpaceControlWamSkeletonName
    = "visual_operational_space_control_wam";
inline constexpr const char* kOperationalSpaceControlTargetFrameName
    = "operational_space_control_target";
inline constexpr const char* kOperationalSpaceControlGroundSkeletonName
    = "visual_operational_space_control_ground";
inline constexpr const char* kWamIkFastFixtureSkeletonName
    = "visual_wam_ikfast_robot";
inline constexpr const char* kWamIkFastTargetFrameName = "wam_ikfast_target";
inline constexpr const char* kWamIkFastGroundSkeletonName
    = "visual_wam_ikfast_ground";
inline constexpr const char* kFetchRobotFixtureSkeletonName
    = "robot0:base_link";
inline constexpr const char* kFetchObjectFixtureSkeletonName = "object0";
inline constexpr const char* kFetchTargetFrameName
    = "fetch_pick_and_place_target";
inline constexpr const char* kTinkertoyFixtureSkeletonPrefix
    = "visual_tinkertoy_toy_";
inline constexpr const char* kTinkertoyTargetFrameName = "tinkertoy_target";
inline constexpr const char* kTinkertoyForceLineFrameName
    = "tinkertoy_force_line";
inline constexpr const char* kTinkertoyAxisFramePrefix = "tinkertoy_axis_";
inline constexpr const char* kAtlasFixtureSkeletonName
    = "visual_atlas_torso_mesh";
inline constexpr const char* kAtlasRobotFixtureSkeletonName
    = "visual_atlas_robot";
inline constexpr const char* kAtlasPuppetFixtureGroundSkeletonName
    = "visual_atlas_puppet_ground";
inline constexpr const char* kHuboPuppetRobotFixtureSkeletonName
    = "visual_hubo_puppet_robot";
inline constexpr const char* kHuboPuppetFixtureGroundSkeletonName
    = "visual_hubo_puppet_ground";
inline constexpr const char* kHuboPuppetIkTargetFramePrefix
    = "hubo_puppet_ik_target_";
inline constexpr const char* kAtlasSimbiconFixtureGroundSkeletonName
    = "visual_atlas_simbicon_ground";
inline constexpr const char* kAtlasPuppetIkTargetFramePrefix
    = "atlas_puppet_ik_target_";
inline constexpr const char* kHelloWorldBoxFixtureSkeletonName
    = "visual_hello_world_box";
inline constexpr const char* kHelloWorldGroundFixtureSkeletonName
    = "visual_hello_world_ground";
inline constexpr const char* kBoxesFixtureBoxSkeletonPrefix
    = "visual_boxes_box_";
inline constexpr const char* kBoxesFixtureGroundSkeletonName
    = "visual_boxes_ground";
inline constexpr const char* kHardcodedDesignFixtureSkeletonName
    = "visual_hardcoded_design";
inline constexpr const char* kRigidChainFixtureSkeletonName
    = "visual_rigid_chain";
inline constexpr const char* kRigidLoopFixtureSkeletonName
    = "visual_rigid_loop";
inline constexpr const char* kMixedChainFixtureSkeletonName
    = "visual_mixed_chain";
inline constexpr const char* kCouplerConstraintFixtureSkeletonPrefix
    = "visual_coupler_constraint_";
inline constexpr const char* kCouplerConstraintFixtureFramePrefix
    = "visual_coupler_constraint_";
inline constexpr const char* kAddDeleteSkelsFixtureCubeSkeletonPrefix
    = "visual_add_delete_skels_cube_";
inline constexpr const char* kAddDeleteSkelsFixtureGroundSkeletonName
    = "visual_add_delete_skels_ground";
inline constexpr const char* kVehicleFixtureCarSkeletonName
    = "visual_vehicle_car";
inline constexpr const char* kVehicleFixtureGroundSkeletonName
    = "visual_vehicle_ground";
inline constexpr const char* kVehicleFixtureObstacleSkeletonPrefix
    = "visual_vehicle_obstacle_";
inline constexpr const char* kHybridDynamicsFixtureBipedSkeletonName
    = "visual_hybrid_dynamics_biped";
inline constexpr const char* kHybridDynamicsFixtureGroundSkeletonName
    = "visual_hybrid_dynamics_ground";
inline constexpr const char* kJointConstraintsFixtureBipedSkeletonName
    = "visual_joint_constraints_biped";
inline constexpr const char* kJointConstraintsFixtureGroundSkeletonName
    = "visual_joint_constraints_ground";
inline constexpr const char* kFreeJointCasesActiveSkeletonPrefix
    = "visual_free_joint_case_active_";
inline constexpr const char* kFreeJointCasesReferenceSkeletonPrefix
    = "visual_free_joint_case_reference_";
inline constexpr const char* kHumanJointLimitsFixtureSkeletonName
    = "visual_human_joint_limits_human";
inline constexpr const char* kHumanJointLimitsFixtureGroundSkeletonName
    = "visual_human_joint_limits_ground";
inline constexpr const char* kLcpPhysicsFixtureBoxSkeletonPrefix
    = "visual_lcp_physics_box_";
inline constexpr const char* kLcpPhysicsFixtureSphereSkeletonPrefix
    = "visual_lcp_physics_sphere_";
inline constexpr const char* kLcpPhysicsFixtureGroundSkeletonName
    = "visual_lcp_physics_ground";
inline constexpr const char* kMimicPendulumsFixtureSkeletonPrefix
    = "visual_mimic_pendulums_";
inline constexpr const char* kMimicPendulumsFixtureGroundSkeletonName
    = "visual_mimic_pendulums_ground";
inline constexpr const char* kPyramidFixtureSkeletonName = "visual_pyramid";
inline constexpr const char* kMultiSphereFixtureSkeletonName
    = "visual_multi_sphere";
inline constexpr const char* kLineSegmentFixtureSkeletonName
    = "visual_line_segments";
inline constexpr const char* kConvexMeshFixtureSkeletonName
    = "visual_convex_mesh";
inline constexpr const char* kPointCloudFixtureSkeletonName
    = "visual_point_cloud";
inline constexpr const char* kCapsuleGroundContactCapsuleSkeletonName
    = "visual_capsule_ground_contact_capsule";
inline constexpr const char* kCapsuleGroundContactGroundSkeletonName
    = "visual_capsule_ground_contact_ground";
inline constexpr const char* kSimulationEventHandlerBoxSkeletonPrefix
    = "visual_simulation_event_handler_box_";
inline constexpr const char* kSimulationEventHandlerSphereSkeletonName
    = "visual_simulation_event_handler_sphere";
inline constexpr const char* kSimulationEventHandlerGroundSkeletonName
    = "visual_simulation_event_handler_ground";
inline constexpr const char* kSimulationEventHandlerFastSensorFrameName
    = "simulation_event_handler_fast_sensor_marker";
inline constexpr const char* kSimulationEventHandlerSlowSensorFrameName
    = "simulation_event_handler_slow_sensor_marker";
inline constexpr const char* kHeightmapFixtureSkeletonName = "visual_heightmap";
inline constexpr const char* kSoftMeshFixtureSkeletonName = "visual_soft_mesh";
inline constexpr const char* kVoxelGridFixtureSkeletonName
    = "visual_voxel_grid";
inline constexpr const char* kPbrEnvironmentFixtureSkeletonName
    = "visual_pbr_environment";
inline constexpr const char* kPolyhedronFixtureSkeletonName
    = "visual_polyhedron_surface";
inline constexpr const char* kPolyhedronWireframeFixtureSkeletonName
    = "visual_polyhedron_wireframe";
inline constexpr const char* kSimpleFramesFixtureBoxFramePrefix
    = "simple_frames_box_";
inline constexpr const char* kSimpleFramesFixtureEllipsoidFramePrefix
    = "simple_frames_marker_";
inline constexpr const char* kSimpleFramesFixtureArrowFrameName
    = "simple_frames_arrow";
inline constexpr const char* kG1FixtureSkeletonName = "visual_g1_robot";
inline constexpr std::size_t kBoxesFixtureBoxCount = 125;
inline constexpr std::size_t kHardcodedDesignFixtureBoxCount = 3;
inline constexpr std::size_t kRigidChainFixtureBoxCount = 10;
inline constexpr std::size_t kRigidLoopFixtureBoxCount = 10;
inline constexpr std::size_t kMixedChainFixtureBoxCount = 10;
inline constexpr std::size_t kMixedChainFixtureSoftMeshCount = 5;
inline constexpr std::size_t kCouplerConstraintFixtureBoxCount = 4;
inline constexpr std::size_t kCouplerConstraintFixtureLineCount = 6;
inline constexpr std::size_t kAddDeleteSkelsFixtureCubeCount = 5;
inline constexpr std::size_t kAddDeleteSkelsFixtureGroundCount = 1;
inline constexpr std::size_t kVehicleFixtureCarBoxCount = 1;
inline constexpr std::size_t kVehicleFixtureWheelCylinderCount = 4;
inline constexpr std::size_t kVehicleFixtureGroundCount = 1;
inline constexpr std::size_t kVehicleFixtureObstacleCount = 2;
inline constexpr std::size_t kHybridDynamicsFixtureBipedBoxCount = 20;
inline constexpr std::size_t kHybridDynamicsFixtureGroundCount = 1;
inline constexpr std::size_t kJointConstraintsFixtureBipedBoxCount = 20;
inline constexpr std::size_t kJointConstraintsFixtureGroundCount = 1;
inline constexpr std::size_t kFreeJointCasesCaseCount = 5;
inline constexpr std::size_t kHumanJointLimitsFixtureMeshCount = 6;
inline constexpr std::size_t kHumanJointLimitsFixtureMultiSphereCount = 8;
inline constexpr std::size_t kHumanJointLimitsFixtureBoxCount = 4;
inline constexpr std::size_t kHumanJointLimitsFixtureGroundCount = 1;
inline constexpr std::size_t kLcpPhysicsFixtureBoxCount = 20;
inline constexpr std::size_t kLcpPhysicsFixtureSphereCount = 12;
inline constexpr std::size_t kLcpPhysicsFixtureGroundCount = 1;
inline constexpr std::size_t kMimicPendulumsFixtureBoxCount = 4;
inline constexpr std::size_t kMimicPendulumsFixtureCylinderCount = 15;
inline constexpr std::size_t kMimicPendulumsFixtureGroundCount = 1;
inline constexpr std::size_t kAtlasPuppetFixtureGroundCount = 1;
inline constexpr std::size_t kAtlasPuppetIkTargetCount = 4;
inline constexpr std::size_t kMinHuboPuppetRenderableCount = 20;
inline constexpr std::size_t kHuboPuppetFixtureGroundCount = 1;
inline constexpr std::size_t kHuboPuppetIkTargetCount = 6;
inline constexpr std::size_t kAtlasSimbiconFixtureGroundCount = 1;
inline constexpr std::size_t kOperationalSpaceControlTargetCount = 1;
inline constexpr std::size_t kOperationalSpaceControlGroundCount = 1;
inline constexpr std::size_t kWamIkFastTargetCount = 1;
inline constexpr std::size_t kWamIkFastGroundCount = 1;
inline constexpr std::size_t kFetchTargetCount = 1;
inline constexpr std::size_t kTinkertoyFixtureBoxCount = 11;
inline constexpr std::size_t kTinkertoyFixtureCylinderCount = 3;
inline constexpr std::size_t kTinkertoyFixtureSphereCount = 4;
inline constexpr std::size_t kTinkertoyTargetCount = 1;
inline constexpr std::size_t kTinkertoyForceLineCount = 1;
inline constexpr std::size_t kTinkertoyAxisLineCount = 3;
inline constexpr std::size_t kSimpleFramesFixtureBoxCount = 3;
inline constexpr std::size_t kSimpleFramesFixtureEllipsoidCount = 4;
inline constexpr std::size_t kSoftBodiesFixtureMinSoftMeshCount = 5;
inline constexpr std::size_t kMinPbrEnvironmentRenderableCount = 4;
inline constexpr std::size_t kMinDragAndDropFrameRenderableCount = 5;
inline constexpr std::size_t kMinG1RenderableCount = 20;

const char* sceneName(ExampleScene scene);

bool parseSceneName(std::string_view name, ExampleScene& scene);

dart::gui::OrbitCamera initialCameraForScene(ExampleScene scene);

AppOptions parseOptions(
    int argc,
    char* argv[],
    const std::optional<dart::gui::RunOptions>& runDefaults = std::nullopt);

DartScene createDartScene(const AppOptions& options);

std::vector<dart::gui::RenderableDescriptor> collectSceneRenderables(
    const DartScene& scene);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_SCENES_HPP_
