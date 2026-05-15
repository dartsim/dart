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

#ifndef DART_GUI_EXPERIMENTAL_VIEWER_HPP_
#define DART_GUI_EXPERIMENTAL_VIEWER_HPP_

#include <dart/gui/viewer.hpp>

namespace dart::gui::experimental {

using ::dart::gui::addOrbitCameraScroll;
using ::dart::gui::cameraEye;
using ::dart::gui::computeCameraRelativeNudge;
using ::dart::gui::DirectionalNudgeInput;
using ::dart::gui::makeFrameOutputPath;
using ::dart::gui::makeOrbitCameraBasis;
using ::dart::gui::makePerspectivePickRay;
using ::dart::gui::makePerspectiveProjection;
using ::dart::gui::markFrameRendered;
using ::dart::gui::markFrameSkipped;
using ::dart::gui::markScreenshotRequested;
using ::dart::gui::markSimulationAdvanced;
using ::dart::gui::normalizeRunOptions;
using ::dart::gui::OrbitCamera;
using ::dart::gui::OrbitCameraBasis;
using ::dart::gui::OrbitCameraController;
using ::dart::gui::OrbitCameraControllerInput;
using ::dart::gui::OrbitCameraUpdate;
using ::dart::gui::PerspectiveProjection;
using ::dart::gui::PickRay;
using ::dart::gui::ProjectionOptions;
using ::dart::gui::requestSingleStep;
using ::dart::gui::resetOrbitCameraTracking;
using ::dart::gui::RunOptions;
using ::dart::gui::shouldAdvanceSimulation;
using ::dart::gui::shouldCaptureFrameOutput;
using ::dart::gui::shouldRequestScreenshot;
using ::dart::gui::shouldStopAfterFrame;
using ::dart::gui::togglePaused;
using ::dart::gui::updateOrbitCamera;
using ::dart::gui::updateOrbitCameraController;
using ::dart::gui::ViewerLifecycleState;
using ::dart::gui::writeRgbaPpm;

} // namespace dart::gui::experimental

#endif // DART_GUI_EXPERIMENTAL_VIEWER_HPP_
