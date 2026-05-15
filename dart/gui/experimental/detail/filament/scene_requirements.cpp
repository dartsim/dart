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

#include <dart/gui/experimental/detail/filament/scene_requirements.hpp>

#include <dart/config.hpp>
#include <dart/gui/experimental/detail/filament/renderable_resources.hpp>

#include <algorithm>
#include <ostream>

namespace dart::gui::experimental::filament {

namespace {

using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::ShapeKind;

bool requireEqual(
    std::size_t actual,
    std::size_t expected,
    const char* subject,
    const char* unit,
    std::ostream& output)
{
  if (actual == expected) {
    return true;
  }

  output << "Expected the " << subject << " to provide " << expected << ' '
         << unit << ", but extracted " << actual << "\n";
  return false;
}

bool requireAtLeast(
    std::size_t actual,
    std::size_t minimum,
    const char* subject,
    const char* unit,
    std::ostream& output)
{
  if (actual >= minimum) {
    return true;
  }

  output << "Expected the " << subject << " to provide at least " << minimum
         << ' ' << unit << ", but extracted " << actual << "\n";
  return false;
}

bool requireCreatedEqual(
    std::size_t actual,
    std::size_t expected,
    const char* subject,
    std::ostream& output)
{
  if (actual == expected) {
    return true;
  }

  output << "Only " << actual << " of " << expected << ' ' << subject
         << " were created\n";
  return false;
}

bool requireCreatedAtLeast(
    std::size_t actual,
    std::size_t minimum,
    const char* subject,
    std::ostream& output)
{
  if (actual >= minimum) {
    return true;
  }

  output << "Only " << actual << " of " << minimum << ' ' << subject
         << " were created\n";
  return false;
}

} // namespace

void accumulateSceneContent(
    SceneContentCounts& counts,
    const RenderableDescriptor& descriptor)
{
  if (!descriptor.material.visible) {
    return;
  }

  if (descriptor.skeletonName == kWamFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.wamMeshes;
  }
  if (descriptor.skeletonName == kOperationalSpaceControlWamSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.operationalSpaceControlWamMeshes;
  }
  if (descriptor.shapeFrameName == kOperationalSpaceControlTargetFrameName
      && descriptor.geometry.kind == ShapeKind::Sphere) {
    ++counts.operationalSpaceControlTargets;
  }
  if (descriptor.skeletonName == kOperationalSpaceControlGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.operationalSpaceControlGrounds;
  }
  if (descriptor.skeletonName == kAtlasFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.atlasMeshes;
  }
  if (descriptor.skeletonName == kAtlasRobotFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.atlasRobotMeshes;
  }
  if (descriptor.skeletonName == kAtlasPuppetFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.atlasPuppetGrounds;
  }
  if (descriptor.shapeFrameName.starts_with(kAtlasPuppetIkTargetFramePrefix)
      && descriptor.geometry.kind == ShapeKind::Sphere) {
    ++counts.atlasPuppetIkTargets;
  }
  if (descriptor.skeletonName == kHelloWorldBoxFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.helloWorldBoxes;
  }
  if (descriptor.skeletonName == kHelloWorldGroundFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.helloWorldGrounds;
  }
  if (descriptor.skeletonName.starts_with(kBoxesFixtureBoxSkeletonPrefix)
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.boxesExampleBoxes;
  }
  if (descriptor.skeletonName == kBoxesFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.boxesExampleGrounds;
  }
  if (descriptor.skeletonName == kHardcodedDesignFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.hardcodedDesignBoxes;
  }
  if (descriptor.skeletonName == kRigidChainFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.rigidChainBoxes;
  }
  if (descriptor.skeletonName == kRigidLoopFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.rigidLoopBoxes;
  }
  if (descriptor.skeletonName == kMixedChainFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.mixedChainBoxes;
  }
  if (descriptor.skeletonName == kMixedChainFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::SoftMesh) {
    ++counts.mixedChainSoftMeshes;
  }
  if (descriptor.skeletonName.starts_with(
          kCouplerConstraintFixtureSkeletonPrefix)
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.couplerConstraintBoxes;
  }
  if (descriptor.shapeFrameName.starts_with(kCouplerConstraintFixtureFramePrefix)
      && descriptor.geometry.kind == ShapeKind::LineSegments) {
    ++counts.couplerConstraintLines;
  }
  if (descriptor.skeletonName.starts_with(
          kAddDeleteSkelsFixtureCubeSkeletonPrefix)
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.addDeleteSkelsCubes;
  }
  if (descriptor.skeletonName == kAddDeleteSkelsFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.addDeleteSkelsGrounds;
  }
  if (descriptor.skeletonName == kVehicleFixtureCarSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.vehicleCarBoxes;
  }
  if (descriptor.skeletonName == kVehicleFixtureCarSkeletonName
      && descriptor.geometry.kind == ShapeKind::Cylinder) {
    ++counts.vehicleWheelCylinders;
  }
  if (descriptor.skeletonName == kVehicleFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.vehicleGrounds;
  }
  if (descriptor.skeletonName.starts_with(kVehicleFixtureObstacleSkeletonPrefix)
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.vehicleObstacles;
  }
  if (descriptor.skeletonName == kHybridDynamicsFixtureBipedSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.hybridDynamicsBipedBoxes;
  }
  if (descriptor.skeletonName == kHybridDynamicsFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.hybridDynamicsGrounds;
  }
  if (descriptor.skeletonName == kJointConstraintsFixtureBipedSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.jointConstraintsBipedBoxes;
  }
  if (descriptor.skeletonName == kJointConstraintsFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.jointConstraintsGrounds;
  }
  if (descriptor.skeletonName.starts_with(kMimicPendulumsFixtureSkeletonPrefix)
      && descriptor.skeletonName != kMimicPendulumsFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.mimicPendulumsBoxes;
  }
  if (descriptor.skeletonName.starts_with(kMimicPendulumsFixtureSkeletonPrefix)
      && descriptor.skeletonName != kMimicPendulumsFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Cylinder) {
    ++counts.mimicPendulumsCylinders;
  }
  if (descriptor.skeletonName == kMimicPendulumsFixtureGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.mimicPendulumsGrounds;
  }
  if (descriptor.skeletonName == kPyramidFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Pyramid) {
    ++counts.pyramids;
  }
  if (descriptor.skeletonName == kMultiSphereFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::MultiSphere) {
    ++counts.multiSpheres;
  }
  if (descriptor.skeletonName == kLineSegmentFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::LineSegments) {
    ++counts.lineSegments;
  }
  if (descriptor.skeletonName == kConvexMeshFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::ConvexMesh) {
    ++counts.convexMeshes;
  }
  if (descriptor.skeletonName == kPointCloudFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::PointCloud) {
    ++counts.pointClouds;
  }
  if (descriptor.skeletonName == kCapsuleGroundContactCapsuleSkeletonName
      && descriptor.geometry.kind == ShapeKind::Capsule) {
    ++counts.capsuleGroundContactCapsules;
  }
  if (descriptor.skeletonName == kCapsuleGroundContactGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.capsuleGroundContactGrounds;
  }
  if (descriptor.skeletonName.starts_with(
          kSimulationEventHandlerBoxSkeletonPrefix)
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.simulationEventHandlerBoxes;
  }
  if (descriptor.skeletonName == kSimulationEventHandlerSphereSkeletonName
      && descriptor.geometry.kind == ShapeKind::Sphere) {
    ++counts.simulationEventHandlerSpheres;
  }
  if (descriptor.skeletonName == kSimulationEventHandlerGroundSkeletonName
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.simulationEventHandlerGrounds;
  }
  if ((descriptor.shapeFrameName == kSimulationEventHandlerFastSensorFrameName
       || descriptor.shapeFrameName == kSimulationEventHandlerSlowSensorFrameName)
      && descriptor.geometry.kind == ShapeKind::Sphere) {
    ++counts.simulationEventHandlerSensorMarkers;
  }
  if (descriptor.skeletonName == kHeightmapFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Heightmap) {
    ++counts.heightmaps;
  }
  if (descriptor.geometry.kind == ShapeKind::SoftMesh) {
    ++counts.softBodyMeshes;
  }
  if (descriptor.skeletonName == kSoftMeshFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::SoftMesh) {
    ++counts.softMeshes;
  }
  if (descriptor.skeletonName == kVoxelGridFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::VoxelGrid) {
    ++counts.voxelGrids;
  }
  if (descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.pbrEnvironmentMeshes;
  }
  if (descriptor.skeletonName == kPolyhedronFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::ConvexMesh) {
    ++counts.polyhedronSurfaces;
  }
  if (descriptor.skeletonName == kPolyhedronWireframeFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::LineSegments) {
    ++counts.polyhedronWireframes;
  }
  if (descriptor.skeletonName == kG1FixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.g1Meshes;
  }
  if (descriptor.shapeFrameName.starts_with(kSimpleFramesFixtureBoxFramePrefix)
      && descriptor.geometry.kind == ShapeKind::Box) {
    ++counts.simpleFrameBoxes;
  }
  if (descriptor.shapeFrameName.starts_with(
          kSimpleFramesFixtureEllipsoidFramePrefix)
      && descriptor.geometry.kind == ShapeKind::Ellipsoid) {
    ++counts.simpleFrameEllipsoids;
  }
  if (descriptor.shapeFrameName == kSimpleFramesFixtureArrowFrameName
      && descriptor.geometry.kind == ShapeKind::LineSegments) {
    ++counts.simpleFrameArrowLines;
  }
  if (!descriptor.shapeFrameName.empty()) {
    ++counts.dragAndDropFrames;
  }
}

SceneContentCounts countSceneContent(
    const std::vector<RenderableDescriptor>& descriptors)
{
  SceneContentCounts counts;
  for (const RenderableDescriptor& descriptor : descriptors) {
    accumulateSceneContent(counts, descriptor);
  }
  return counts;
}

SceneContentCounts countCreatedSceneContent(
    const std::vector<RenderableDescriptor>& descriptors,
    const std::vector<SceneRenderable>& sceneRenderables)
{
  SceneContentCounts counts;
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    const auto descriptor = std::find_if(
        descriptors.begin(),
        descriptors.end(),
        [&](const RenderableDescriptor& candidate) {
          return candidate.id == sceneRenderable.id;
        });
    if (descriptor != descriptors.end()) {
      accumulateSceneContent(counts, *descriptor);
    }
  }
  return counts;
}

bool validateSceneDescriptorContent(
    ExampleScene scene,
    const SceneContentCounts& counts,
    std::ostream& output)
{
  if (scene == ExampleScene::Mvp) {
    if (!requireAtLeast(
            counts.wamMeshes,
            5,
            "WAM robot fixture",
            "visible mesh renderables",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.atlasMeshes,
            1,
            "Atlas mesh fixture",
            "visible mesh renderable",
            output)) {
      return false;
    }
    if (!requireAtLeast(
            counts.atlasRobotMeshes,
            20,
            "Atlas robot fixture",
            "visible mesh renderables",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.pyramids,
            1,
            "pyramid fixture",
            "visible pyramid renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.multiSpheres,
            1,
            "multi-sphere fixture",
            "visible multi-sphere renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.lineSegments,
            1,
            "line segment fixture",
            "visible line renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.convexMeshes,
            1,
            "convex mesh fixture",
            "visible convex mesh renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.pointClouds,
            1,
            "point cloud fixture",
            "visible point cloud renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.heightmaps,
            1,
            "heightmap fixture",
            "visible heightmap renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.softMeshes,
            1,
            "soft mesh fixture",
            "visible soft mesh renderable descriptor",
            output)) {
      return false;
    }
#if DART_HAVE_OCTOMAP
    if (!requireEqual(
            counts.voxelGrids,
            1,
            "voxel grid fixture",
            "visible voxel grid renderable descriptor",
            output)) {
      return false;
    }
#endif
    return requireAtLeast(
        counts.pbrEnvironmentMeshes,
        kMinPbrEnvironmentRenderableCount,
        "PBR environment fixture",
        "visible mesh renderables",
        output);
  }

  if (scene == ExampleScene::G1) {
    return requireAtLeast(
        counts.g1Meshes,
        kMinG1RenderableCount,
        "G1 robot fixture",
        "visible mesh renderables",
        output);
  }

  if (scene == ExampleScene::AtlasPuppet) {
    if (!requireAtLeast(
            counts.atlasRobotMeshes,
            20,
            "Atlas puppet fixture",
            "visible mesh renderables",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.atlasPuppetGrounds,
            kAtlasPuppetFixtureGroundCount,
            "atlas-puppet scene",
            "visible ground renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.atlasPuppetIkTargets,
        kAtlasPuppetIkTargetCount,
        "atlas-puppet scene",
        "visible IK target renderable descriptors",
        output);
  }

  if (scene == ExampleScene::OperationalSpaceControl) {
    if (!requireAtLeast(
            counts.operationalSpaceControlWamMeshes,
            5,
            "operational-space-control scene",
            "visible WAM mesh renderables",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.operationalSpaceControlTargets,
            kOperationalSpaceControlTargetCount,
            "operational-space-control scene",
            "visible target renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.operationalSpaceControlGrounds,
        kOperationalSpaceControlGroundCount,
        "operational-space-control scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::HelloWorld) {
    if (!requireEqual(
            counts.helloWorldBoxes,
            1,
            "hello world scene",
            "visible dynamic box renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.helloWorldGrounds,
        1,
        "hello world scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::Boxes) {
    if (!requireEqual(
            counts.boxesExampleBoxes,
            kBoxesFixtureBoxCount,
            "boxes scene",
            "visible dynamic box renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.boxesExampleGrounds,
        1,
        "boxes scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::HardcodedDesign) {
    return requireEqual(
        counts.hardcodedDesignBoxes,
        kHardcodedDesignFixtureBoxCount,
        "hardcoded-design scene",
        "visible manually constructed box renderable descriptors",
        output);
  }

  if (scene == ExampleScene::RigidChain) {
    return requireEqual(
        counts.rigidChainBoxes,
        kRigidChainFixtureBoxCount,
        "rigid-chain scene",
        "visible chain box renderable descriptors",
        output);
  }

  if (scene == ExampleScene::RigidLoop) {
    return requireEqual(
        counts.rigidLoopBoxes,
        kRigidLoopFixtureBoxCount,
        "rigid-loop scene",
        "visible closed-loop chain box renderable descriptors",
        output);
  }

  if (scene == ExampleScene::MixedChain) {
    if (!requireEqual(
            counts.mixedChainBoxes,
            kMixedChainFixtureBoxCount,
            "mixed-chain scene",
            "visible chain box renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.mixedChainSoftMeshes,
        kMixedChainFixtureSoftMeshCount,
        "mixed-chain scene",
        "visible soft mesh renderable descriptors",
        output);
  }

  if (scene == ExampleScene::CouplerConstraint) {
    if (!requireEqual(
            counts.couplerConstraintBoxes,
            kCouplerConstraintFixtureBoxCount,
            "coupler-constraint scene",
            "visible rig box renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.couplerConstraintLines,
        kCouplerConstraintFixtureLineCount,
        "coupler-constraint scene",
        "visible guide line renderable descriptors",
        output);
  }

  if (scene == ExampleScene::AddDeleteSkels) {
    if (!requireEqual(
            counts.addDeleteSkelsCubes,
            kAddDeleteSkelsFixtureCubeCount,
            "add-delete-skels scene",
            "visible cube renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.addDeleteSkelsGrounds,
        kAddDeleteSkelsFixtureGroundCount,
        "add-delete-skels scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::Vehicle) {
    if (!requireEqual(
            counts.vehicleCarBoxes,
            kVehicleFixtureCarBoxCount,
            "vehicle scene",
            "visible car body renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.vehicleWheelCylinders,
            kVehicleFixtureWheelCylinderCount,
            "vehicle scene",
            "visible wheel cylinder renderable descriptors",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.vehicleGrounds,
            kVehicleFixtureGroundCount,
            "vehicle scene",
            "visible ground renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.vehicleObstacles,
        kVehicleFixtureObstacleCount,
        "vehicle scene",
        "visible obstacle box renderable descriptors",
        output);
  }

  if (scene == ExampleScene::HybridDynamics) {
    if (!requireEqual(
            counts.hybridDynamicsBipedBoxes,
            kHybridDynamicsFixtureBipedBoxCount,
            "hybrid-dynamics scene",
            "visible biped box renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.hybridDynamicsGrounds,
        kHybridDynamicsFixtureGroundCount,
        "hybrid-dynamics scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::JointConstraints) {
    if (!requireEqual(
            counts.jointConstraintsBipedBoxes,
            kJointConstraintsFixtureBipedBoxCount,
            "joint-constraints scene",
            "visible biped box renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.jointConstraintsGrounds,
        kJointConstraintsFixtureGroundCount,
        "joint-constraints scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::MimicPendulums) {
    if (!requireEqual(
            counts.mimicPendulumsBoxes,
            kMimicPendulumsFixtureBoxCount,
            "mimic-pendulums scene",
            "visible base-pole box renderable descriptors",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.mimicPendulumsCylinders,
            kMimicPendulumsFixtureCylinderCount,
            "mimic-pendulums scene",
            "visible pendulum cylinder renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.mimicPendulumsGrounds,
        kMimicPendulumsFixtureGroundCount,
        "mimic-pendulums scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::Heightmap) {
    return requireEqual(
        counts.heightmaps,
        1,
        "heightmap scene",
        "visible heightmap renderable descriptor",
        output);
  }

  if (scene == ExampleScene::Polyhedron) {
    if (!requireEqual(
            counts.polyhedronSurfaces,
            1,
            "polyhedron fixture",
            "visible convex-mesh renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.polyhedronWireframes,
        1,
        "polyhedron wireframe fixture",
        "visible line renderable descriptor",
        output);
  }

  if (scene == ExampleScene::DragAndDrop) {
    return requireAtLeast(
        counts.dragAndDropFrames,
        kMinDragAndDropFrameRenderableCount,
        "drag-and-drop scene",
        "visible frame renderables",
        output);
  }

  if (scene == ExampleScene::SimpleFrames) {
    if (!requireEqual(
            counts.simpleFrameBoxes,
            kSimpleFramesFixtureBoxCount,
            "simple-frames scene",
            "visible box frame renderable descriptors",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.simpleFrameEllipsoids,
            kSimpleFramesFixtureEllipsoidCount,
            "simple-frames scene",
            "visible ellipsoid frame renderable descriptors",
            output)) {
      return false;
    }
    return requireEqual(
        counts.simpleFrameArrowLines,
        1,
        "simple-frames scene",
        "visible arrow line renderable descriptor",
        output);
  }

  if (scene == ExampleScene::SoftBodies) {
    return requireAtLeast(
        counts.softBodyMeshes,
        kSoftBodiesFixtureMinSoftMeshCount,
        "soft-bodies scene",
        "visible soft-mesh renderable descriptors",
        output);
  }

  if (scene == ExampleScene::PointCloud) {
    if (!requireEqual(
            counts.pointClouds,
            1,
            "point-cloud scene",
            "visible point-cloud renderable descriptor",
            output)) {
      return false;
    }
#if DART_HAVE_OCTOMAP
    return requireEqual(
        counts.voxelGrids,
        1,
        "point-cloud scene",
        "visible voxel-grid renderable descriptor",
        output);
#else
    return true;
#endif
  }

  if (scene == ExampleScene::CapsuleGroundContact) {
    if (!requireEqual(
            counts.capsuleGroundContactCapsules,
            1,
            "capsule-ground-contact scene",
            "visible capsule renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.capsuleGroundContactGrounds,
        1,
        "capsule-ground-contact scene",
        "visible ground renderable descriptor",
        output);
  }

  if (scene == ExampleScene::SimulationEventHandler) {
    if (!requireEqual(
            counts.simulationEventHandlerBoxes,
            3,
            "simulation-event-handler scene",
            "visible box renderable descriptors",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.simulationEventHandlerSpheres,
            1,
            "simulation-event-handler scene",
            "visible sphere renderable descriptor",
            output)) {
      return false;
    }
    if (!requireEqual(
            counts.simulationEventHandlerGrounds,
            1,
            "simulation-event-handler scene",
            "visible ground renderable descriptor",
            output)) {
      return false;
    }
    return requireEqual(
        counts.simulationEventHandlerSensorMarkers,
        2,
        "simulation-event-handler scene",
        "visible sensor marker descriptors",
        output);
  }

  return true;
}

bool validateCreatedSceneContent(
    ExampleScene scene,
    const SceneContentCounts& expected,
    const SceneContentCounts& created,
    std::ostream& output)
{
  if (scene == ExampleScene::Mvp) {
    if (!requireCreatedAtLeast(
            created.wamMeshes, expected.wamMeshes, "WAM robot mesh renderables", output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.atlasMeshes, expected.atlasMeshes, "Atlas mesh renderables", output)) {
      return false;
    }
    if (!requireCreatedAtLeast(
            created.atlasRobotMeshes,
            expected.atlasRobotMeshes,
            "Atlas robot mesh renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.pyramids, expected.pyramids, "pyramid renderables", output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.multiSpheres,
            expected.multiSpheres,
            "multi-sphere renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.lineSegments,
            expected.lineSegments,
            "line segment renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.convexMeshes,
            expected.convexMeshes,
            "convex mesh renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.pointClouds,
            expected.pointClouds,
            "point cloud renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.heightmaps,
            expected.heightmaps,
            "heightmap renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.softMeshes,
            expected.softMeshes,
            "soft mesh renderables",
            output)) {
      return false;
    }
#if DART_HAVE_OCTOMAP
    if (!requireCreatedEqual(
            created.voxelGrids,
            expected.voxelGrids,
            "voxel grid renderables",
            output)) {
      return false;
    }
#endif
    return requireCreatedAtLeast(
        created.pbrEnvironmentMeshes,
        expected.pbrEnvironmentMeshes,
        "PBR environment mesh renderables",
        output);
  }

  if (scene == ExampleScene::G1) {
    return requireCreatedAtLeast(
        created.g1Meshes, expected.g1Meshes, "G1 robot mesh renderables", output);
  }

  if (scene == ExampleScene::AtlasPuppet) {
    if (!requireCreatedAtLeast(
            created.atlasRobotMeshes,
            expected.atlasRobotMeshes,
            "Atlas puppet mesh renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.atlasPuppetGrounds,
            expected.atlasPuppetGrounds,
            "atlas-puppet ground renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.atlasPuppetIkTargets,
        expected.atlasPuppetIkTargets,
        "atlas-puppet IK target renderables",
        output);
  }

  if (scene == ExampleScene::OperationalSpaceControl) {
    if (!requireCreatedAtLeast(
            created.operationalSpaceControlWamMeshes,
            expected.operationalSpaceControlWamMeshes,
            "operational-space-control WAM mesh renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.operationalSpaceControlTargets,
            expected.operationalSpaceControlTargets,
            "operational-space-control target renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.operationalSpaceControlGrounds,
        expected.operationalSpaceControlGrounds,
        "operational-space-control ground renderables",
        output);
  }

  if (scene == ExampleScene::HelloWorld) {
    if (!requireCreatedEqual(
            created.helloWorldBoxes,
            expected.helloWorldBoxes,
            "hello world dynamic box renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.helloWorldGrounds,
        expected.helloWorldGrounds,
        "hello world ground renderables",
        output);
  }

  if (scene == ExampleScene::Boxes) {
    if (!requireCreatedEqual(
            created.boxesExampleBoxes,
            expected.boxesExampleBoxes,
            "boxes dynamic box renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.boxesExampleGrounds,
        expected.boxesExampleGrounds,
        "boxes ground renderables",
        output);
  }

  if (scene == ExampleScene::HardcodedDesign) {
    return requireCreatedEqual(
        created.hardcodedDesignBoxes,
        expected.hardcodedDesignBoxes,
        "hardcoded-design box renderables",
        output);
  }

  if (scene == ExampleScene::RigidChain) {
    return requireCreatedEqual(
        created.rigidChainBoxes,
        expected.rigidChainBoxes,
        "rigid-chain box renderables",
        output);
  }

  if (scene == ExampleScene::RigidLoop) {
    return requireCreatedEqual(
        created.rigidLoopBoxes,
        expected.rigidLoopBoxes,
        "rigid-loop box renderables",
        output);
  }

  if (scene == ExampleScene::MixedChain) {
    if (!requireCreatedEqual(
            created.mixedChainBoxes,
            expected.mixedChainBoxes,
            "mixed-chain box renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.mixedChainSoftMeshes,
        expected.mixedChainSoftMeshes,
        "mixed-chain soft mesh renderables",
        output);
  }

  if (scene == ExampleScene::CouplerConstraint) {
    if (!requireCreatedEqual(
            created.couplerConstraintBoxes,
            expected.couplerConstraintBoxes,
            "coupler-constraint box renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.couplerConstraintLines,
        expected.couplerConstraintLines,
        "coupler-constraint line renderables",
        output);
  }

  if (scene == ExampleScene::AddDeleteSkels) {
    if (!requireCreatedEqual(
            created.addDeleteSkelsCubes,
            expected.addDeleteSkelsCubes,
            "add-delete-skels cube renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.addDeleteSkelsGrounds,
        expected.addDeleteSkelsGrounds,
        "add-delete-skels ground renderables",
        output);
  }

  if (scene == ExampleScene::Vehicle) {
    if (!requireCreatedEqual(
            created.vehicleCarBoxes,
            expected.vehicleCarBoxes,
            "vehicle car body renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.vehicleWheelCylinders,
            expected.vehicleWheelCylinders,
            "vehicle wheel cylinder renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.vehicleGrounds,
            expected.vehicleGrounds,
            "vehicle ground renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.vehicleObstacles,
        expected.vehicleObstacles,
        "vehicle obstacle renderables",
        output);
  }

  if (scene == ExampleScene::HybridDynamics) {
    if (!requireCreatedEqual(
            created.hybridDynamicsBipedBoxes,
            expected.hybridDynamicsBipedBoxes,
            "hybrid-dynamics biped box renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.hybridDynamicsGrounds,
        expected.hybridDynamicsGrounds,
        "hybrid-dynamics ground renderables",
        output);
  }

  if (scene == ExampleScene::JointConstraints) {
    if (!requireCreatedEqual(
            created.jointConstraintsBipedBoxes,
            expected.jointConstraintsBipedBoxes,
            "joint-constraints biped box renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.jointConstraintsGrounds,
        expected.jointConstraintsGrounds,
        "joint-constraints ground renderables",
        output);
  }

  if (scene == ExampleScene::MimicPendulums) {
    if (!requireCreatedEqual(
            created.mimicPendulumsBoxes,
            expected.mimicPendulumsBoxes,
            "mimic-pendulums box renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.mimicPendulumsCylinders,
            expected.mimicPendulumsCylinders,
            "mimic-pendulums cylinder renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.mimicPendulumsGrounds,
        expected.mimicPendulumsGrounds,
        "mimic-pendulums ground renderables",
        output);
  }

  if (scene == ExampleScene::Heightmap) {
    return requireCreatedEqual(
        created.heightmaps, expected.heightmaps, "heightmap renderables", output);
  }

  if (scene == ExampleScene::Polyhedron) {
    if (!requireCreatedEqual(
            created.polyhedronSurfaces,
            expected.polyhedronSurfaces,
            "polyhedron surface renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.polyhedronWireframes,
        expected.polyhedronWireframes,
        "polyhedron wireframe renderables",
        output);
  }

  if (scene == ExampleScene::DragAndDrop) {
    return requireCreatedAtLeast(
        created.dragAndDropFrames,
        expected.dragAndDropFrames,
        "drag-and-drop frame renderables",
        output);
  }

  if (scene == ExampleScene::SimpleFrames) {
    if (!requireCreatedEqual(
            created.simpleFrameBoxes,
            expected.simpleFrameBoxes,
            "simple-frames box renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.simpleFrameEllipsoids,
            expected.simpleFrameEllipsoids,
            "simple-frames ellipsoid renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.simpleFrameArrowLines,
        expected.simpleFrameArrowLines,
        "simple-frames arrow line renderables",
        output);
  }

  if (scene == ExampleScene::SoftBodies) {
    return requireCreatedAtLeast(
        created.softBodyMeshes,
        expected.softBodyMeshes,
        "soft-bodies soft-mesh renderables",
        output);
  }

  if (scene == ExampleScene::PointCloud) {
    if (!requireCreatedEqual(
            created.pointClouds,
            expected.pointClouds,
            "point-cloud renderables",
            output)) {
      return false;
    }
#if DART_HAVE_OCTOMAP
    return requireCreatedEqual(
        created.voxelGrids,
        expected.voxelGrids,
        "point-cloud voxel-grid renderables",
        output);
#else
    return true;
#endif
  }

  if (scene == ExampleScene::CapsuleGroundContact) {
    if (!requireCreatedEqual(
            created.capsuleGroundContactCapsules,
            expected.capsuleGroundContactCapsules,
            "capsule-ground-contact capsule renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.capsuleGroundContactGrounds,
        expected.capsuleGroundContactGrounds,
        "capsule-ground-contact ground renderables",
        output);
  }

  if (scene == ExampleScene::SimulationEventHandler) {
    if (!requireCreatedEqual(
            created.simulationEventHandlerBoxes,
            expected.simulationEventHandlerBoxes,
            "simulation-event-handler box renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.simulationEventHandlerSpheres,
            expected.simulationEventHandlerSpheres,
            "simulation-event-handler sphere renderables",
            output)) {
      return false;
    }
    if (!requireCreatedEqual(
            created.simulationEventHandlerGrounds,
            expected.simulationEventHandlerGrounds,
            "simulation-event-handler ground renderables",
            output)) {
      return false;
    }
    return requireCreatedEqual(
        created.simulationEventHandlerSensorMarkers,
        expected.simulationEventHandlerSensorMarkers,
        "simulation-event-handler sensor marker renderables",
        output);
  }

  return true;
}

} // namespace dart::gui::experimental::filament
