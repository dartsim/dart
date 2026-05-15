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
  if (descriptor.skeletonName == kAtlasFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.atlasMeshes;
  }
  if (descriptor.skeletonName == kAtlasRobotFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Mesh) {
    ++counts.atlasRobotMeshes;
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
  if (descriptor.skeletonName == kHeightmapFixtureSkeletonName
      && descriptor.geometry.kind == ShapeKind::Heightmap) {
    ++counts.heightmaps;
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

  return true;
}

} // namespace dart::gui::experimental::filament
