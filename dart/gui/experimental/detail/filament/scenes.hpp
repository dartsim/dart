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

#ifndef DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCENES_HPP_
#define DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCENES_HPP_

#include <dart/dynamics/fwd.hpp>
#include <dart/gui/experimental/scene.hpp>
#include <dart/simulation/fwd.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::gui::experimental::filament {

struct G1IkHandle
{
  dart::gui::experimental::RenderableId targetRenderableId = 0;
  std::string label;
  int hotkey = 0;
  std::shared_ptr<dart::dynamics::SimpleFrame> target;
  dart::dynamics::InverseKinematicsPtr ik;
};

struct DartScene
{
  dart::simulation::WorldPtr world;
  std::vector<G1IkHandle> ikHandles;
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
  dart::gui::experimental::RunOptions run;
  ExampleScene scene = ExampleScene::Mvp;
  bool showUi = true;
  bool showUiExplicit = false;
  bool profile = false;
  bool orbitLight = true;
  double orbitLightPeriodSeconds = 80.0;
  std::string g1PackageName = "g1_description";
  std::string g1PackageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string g1RobotUri = "package://g1_description/g1_29dof.urdf";
};

inline constexpr const char* kWamFixtureSkeletonName = "visual_wam_robot";
inline constexpr const char* kAtlasFixtureSkeletonName = "visual_atlas_torso_mesh";
inline constexpr const char* kAtlasRobotFixtureSkeletonName = "visual_atlas_robot";
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
inline constexpr const char* kRigidLoopFixtureSkeletonName = "visual_rigid_loop";
inline constexpr const char* kMixedChainFixtureSkeletonName = "visual_mixed_chain";
inline constexpr const char* kPyramidFixtureSkeletonName = "visual_pyramid";
inline constexpr const char* kMultiSphereFixtureSkeletonName = "visual_multi_sphere";
inline constexpr const char* kLineSegmentFixtureSkeletonName = "visual_line_segments";
inline constexpr const char* kConvexMeshFixtureSkeletonName = "visual_convex_mesh";
inline constexpr const char* kPointCloudFixtureSkeletonName = "visual_point_cloud";
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
inline constexpr const char* kVoxelGridFixtureSkeletonName = "visual_voxel_grid";
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
inline constexpr std::size_t kSimpleFramesFixtureBoxCount = 3;
inline constexpr std::size_t kSimpleFramesFixtureEllipsoidCount = 4;
inline constexpr std::size_t kSoftBodiesFixtureMinSoftMeshCount = 5;
inline constexpr std::size_t kMinPbrEnvironmentRenderableCount = 4;
inline constexpr std::size_t kMinDragAndDropFrameRenderableCount = 5;
inline constexpr std::size_t kMinG1RenderableCount = 20;

const char* sceneName(ExampleScene scene);

bool parseSceneName(std::string_view name, ExampleScene& scene);

dart::gui::experimental::OrbitCamera initialCameraForScene(ExampleScene scene);

AppOptions parseOptions(int argc, char* argv[]);

DartScene createDartScene(const AppOptions& options);

} // namespace dart::gui::experimental::filament

#endif // DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCENES_HPP_
