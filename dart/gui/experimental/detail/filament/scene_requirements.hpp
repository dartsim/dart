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

#ifndef DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCENE_REQUIREMENTS_HPP_
#define DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCENE_REQUIREMENTS_HPP_

#include <dart/gui/experimental/detail/filament/scenes.hpp>
#include <dart/gui/experimental/renderable.hpp>

#include <iosfwd>
#include <vector>

#include <cstddef>

namespace dart::gui::experimental::filament {

struct SceneRenderable;

struct SceneContentCounts
{
  std::size_t wamMeshes = 0;
  std::size_t atlasMeshes = 0;
  std::size_t atlasRobotMeshes = 0;
  std::size_t helloWorldBoxes = 0;
  std::size_t helloWorldGrounds = 0;
  std::size_t boxesExampleBoxes = 0;
  std::size_t boxesExampleGrounds = 0;
  std::size_t hardcodedDesignBoxes = 0;
  std::size_t rigidChainBoxes = 0;
  std::size_t rigidLoopBoxes = 0;
  std::size_t mixedChainBoxes = 0;
  std::size_t mixedChainSoftMeshes = 0;
  std::size_t couplerConstraintBoxes = 0;
  std::size_t couplerConstraintLines = 0;
  std::size_t addDeleteSkelsCubes = 0;
  std::size_t addDeleteSkelsGrounds = 0;
  std::size_t pyramids = 0;
  std::size_t multiSpheres = 0;
  std::size_t lineSegments = 0;
  std::size_t convexMeshes = 0;
  std::size_t pointClouds = 0;
  std::size_t capsuleGroundContactCapsules = 0;
  std::size_t capsuleGroundContactGrounds = 0;
  std::size_t simulationEventHandlerBoxes = 0;
  std::size_t simulationEventHandlerSpheres = 0;
  std::size_t simulationEventHandlerGrounds = 0;
  std::size_t simulationEventHandlerSensorMarkers = 0;
  std::size_t heightmaps = 0;
  std::size_t softMeshes = 0;
  std::size_t softBodyMeshes = 0;
  std::size_t voxelGrids = 0;
  std::size_t pbrEnvironmentMeshes = 0;
  std::size_t polyhedronSurfaces = 0;
  std::size_t polyhedronWireframes = 0;
  std::size_t simpleFrameBoxes = 0;
  std::size_t simpleFrameEllipsoids = 0;
  std::size_t simpleFrameArrowLines = 0;
  std::size_t g1Meshes = 0;
  std::size_t dragAndDropFrames = 0;
};

void accumulateSceneContent(
    SceneContentCounts& counts,
    const dart::gui::experimental::RenderableDescriptor& descriptor);

SceneContentCounts countSceneContent(
    const std::vector<dart::gui::experimental::RenderableDescriptor>&
        descriptors);

SceneContentCounts countCreatedSceneContent(
    const std::vector<dart::gui::experimental::RenderableDescriptor>&
        descriptors,
    const std::vector<SceneRenderable>& sceneRenderables);

bool validateSceneDescriptorContent(
    ExampleScene scene,
    const SceneContentCounts& counts,
    std::ostream& output);

bool validateCreatedSceneContent(
    ExampleScene scene,
    const SceneContentCounts& expected,
    const SceneContentCounts& created,
    std::ostream& output);

} // namespace dart::gui::experimental::filament

#endif // DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SCENE_REQUIREMENTS_HPP_
