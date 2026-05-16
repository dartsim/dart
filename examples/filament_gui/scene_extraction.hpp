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

#ifndef DART_EXAMPLES_FILAMENT_GUI_SCENE_EXTRACTION_HPP_
#define DART_EXAMPLES_FILAMENT_GUI_SCENE_EXTRACTION_HPP_

#include <dart/gui/experimental/scene.hpp>

namespace dart::examples::filament_gui {

using gui::experimental::GeometryDescriptor;
using gui::experimental::DebugDrawOptions;
using gui::experimental::DebugLineDescriptor;
using gui::experimental::MaterialDescriptor;
using gui::experimental::OrbitCamera;
using gui::experimental::OrbitCameraBasis;
using gui::experimental::OrbitCameraUpdate;
using gui::experimental::PickHit;
using gui::experimental::PickRay;
using gui::experimental::RenderableDescriptor;
using gui::experimental::RenderableId;
using gui::experimental::RunOptions;
using gui::experimental::ShapeKind;
using gui::experimental::ViewerLifecycleState;
using gui::experimental::cameraEye;
using gui::experimental::describeShape;
using gui::experimental::extractContactDebugLines;
using gui::experimental::extractDebugLines;
using gui::experimental::extractRenderables;
using gui::experimental::computePlaneDragTranslation;
using gui::experimental::intersectRenderable;
using gui::experimental::intersectPlane;
using gui::experimental::makeRenderableId;
using gui::experimental::makeOrbitCameraBasis;
using gui::experimental::makePerspectivePickRay;
using gui::experimental::makeSelectionDebugLines;
using gui::experimental::markFrameRendered;
using gui::experimental::markFrameSkipped;
using gui::experimental::markScreenshotRequested;
using gui::experimental::markSimulationAdvanced;
using gui::experimental::normalizeRunOptions;
using gui::experimental::pickNearestRenderable;
using gui::experimental::requestSingleStep;
using gui::experimental::shouldRequestScreenshot;
using gui::experimental::shouldAdvanceSimulation;
using gui::experimental::shouldStopAfterFrame;
using gui::experimental::togglePaused;
using gui::experimental::translateFrameRenderable;
using gui::experimental::translateFreeJointRenderable;
using gui::experimental::translateSimpleFrameRenderable;
using gui::experimental::updateOrbitCamera;
using gui::experimental::writeRgbaPpm;

} // namespace dart::examples::filament_gui

#endif // DART_EXAMPLES_FILAMENT_GUI_SCENE_EXTRACTION_HPP_
