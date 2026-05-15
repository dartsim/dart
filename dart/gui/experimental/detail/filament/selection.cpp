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

#include "selection.hpp"

#include "input.hpp"
#include "scenes.hpp"

#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/gui/experimental/interaction.hpp>
#include <dart/gui/experimental/renderable.hpp>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace dart::gui::experimental::filament {

using dart::gui::experimental::OrbitCamera;
using dart::gui::experimental::PickRay;
using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::RenderableId;
using dart::gui::experimental::ViewerLifecycleState;
using dart::gui::experimental::computePlaneDragTranslation;
using dart::gui::experimental::extractRenderables;
using dart::gui::experimental::intersectPlane;
using dart::gui::experimental::makeOrbitCameraBasis;
using dart::gui::experimental::makePerspectivePickRay;
using dart::gui::experimental::pickNearestRenderable;
using dart::gui::experimental::translateFrameRenderable;

namespace {

const RenderableDescriptor* findRenderableDescriptor(
    const std::vector<RenderableDescriptor>& descriptors, RenderableId id)
{
  const auto descriptor = std::find_if(
      descriptors.begin(),
      descriptors.end(),
      [id](const RenderableDescriptor& candidate) {
        return candidate.id == id;
      });
  return descriptor == descriptors.end() ? nullptr : &*descriptor;
}

} // namespace

G1IkHandle* findG1IkHandle(DartScene& scene, RenderableId targetRenderableId)
{
  const auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [&](const G1IkHandle& candidate) {
        return candidate.targetRenderableId == targetRenderableId;
      });
  return handle == scene.ikHandles.end() ? nullptr : &*handle;
}

const G1IkHandle* findG1IkHandle(
    const DartScene& scene, RenderableId targetRenderableId)
{
  const auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [&](const G1IkHandle& candidate) {
        return candidate.targetRenderableId == targetRenderableId;
      });
  return handle == scene.ikHandles.end() ? nullptr : &*handle;
}

std::string selectionLabelForRenderable(
    const DartScene& scene, const RenderableDescriptor& descriptor)
{
  if (const auto* handle = findG1IkHandle(scene, descriptor.id)) {
    return handle->label + " IK target";
  }

  std::string label = descriptor.skeletonName.empty()
                          ? descriptor.shapeFrameName
                          : descriptor.skeletonName + "/" + descriptor.bodyName;
  if (!descriptor.shapeNodeName.empty()) {
    label += "/" + descriptor.shapeNodeName;
  }
  label += " (" + descriptor.geometry.shapeType + ")";
  return label;
}

bool translateRenderableAndApplyIk(
    DartScene& scene,
    const RenderableDescriptor& descriptor,
    const Eigen::Vector3d& worldTranslation)
{
  if (!translateFrameRenderable(descriptor, worldTranslation)) {
    return false;
  }

  if (auto* handle = findG1IkHandle(scene, descriptor.id)) {
    if (handle->ik) {
      handle->ik->getSolver()->setNumMaxIterations(30);
      handle->ik->solveAndApply(true);
    }
  }

  return true;
}

RenderableId SelectionController::selectedRenderableId() const
{
  return mSelectedRenderableId;
}

const std::string& SelectionController::selectedLabel() const
{
  return mSelectedLabel;
}

bool SelectionController::isDraggingSelection() const
{
  return mLeftMouseStartedDrag;
}

void SelectionController::select(RenderableId renderableId, std::string label)
{
  mSelectedRenderableId = renderableId;
  mSelectedLabel = std::move(label);
}

void SelectionController::clear()
{
  mSelectedRenderableId = 0;
  mSelectedLabel = "none";
}

void SelectionController::applyKeyboardNudge(
    GLFWwindow* window,
    const OrbitCamera& camera,
    DartScene& scene,
    std::vector<RenderableDescriptor>& descriptors,
    ViewerLifecycleState& lifecycle,
    double stepSize)
{
  if (window == nullptr || mSelectedRenderableId == 0) {
    return;
  }

  const Eigen::Vector3d nudge
      = selectedNudgeFromKeyboard(window, camera, stepSize);
  if (nudge.squaredNorm() <= 0.0) {
    return;
  }

  const RenderableDescriptor* selectedDescriptor
      = findRenderableDescriptor(descriptors, mSelectedRenderableId);
  if (selectedDescriptor == nullptr
      || !translateRenderableAndApplyIk(scene, *selectedDescriptor, nudge)) {
    return;
  }

  lifecycle.paused = true;
  descriptors = extractRenderables(*scene.world);
}

void SelectionController::updateMouseSelection(
    GLFWwindow* window,
    const OrbitCamera& camera,
    int framebufferWidth,
    int framebufferHeight,
    bool showUi,
    double guiScale,
    DartScene& scene,
    std::vector<RenderableDescriptor>& descriptors,
    ViewerLifecycleState& lifecycle)
{
  if (window == nullptr) {
    return;
  }

  double cursorX = 0.0;
  double cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);
  const bool isLeftMousePressed
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  if (isLeftMousePressed && !mWasLeftMousePressed) {
    mLeftMousePressX = cursorX;
    mLeftMousePressY = cursorY;
    mLeftMouseStartedDrag = false;
    mLeftMouseStartedOnPanel
        = showUi && isInsideStatusPanel(cursorX, cursorY, guiScale);

    if (!mLeftMouseStartedOnPanel && mSelectedRenderableId != 0
        && isDragModifierDown(window)) {
      const RenderableDescriptor* selectedDescriptor
          = findRenderableDescriptor(descriptors, mSelectedRenderableId);
      if (selectedDescriptor != nullptr) {
        const auto basis = makeOrbitCameraBasis(camera);
        const PickRay ray = makePerspectivePickRay(
            camera, cursorX, cursorY, framebufferWidth, framebufferHeight);
        const Eigen::Vector3d planePoint
            = selectedDescriptor->worldTransform.translation();
        const Eigen::Vector3d planeNormal = basis.forward;
        if (intersectPlane(ray, planePoint, planeNormal)) {
          mLeftMouseStartedDrag = true;
          mSelectedDragLastRay = ray;
          mSelectedDragPlanePoint = planePoint;
          mSelectedDragPlaneNormal = planeNormal;
          lifecycle.paused = true;
        }
      }
    }
  }

  if (isLeftMousePressed && mLeftMouseStartedDrag) {
    const PickRay ray = makePerspectivePickRay(
        camera, cursorX, cursorY, framebufferWidth, framebufferHeight);
    const auto translation = computePlaneDragTranslation(
        mSelectedDragLastRay,
        ray,
        mSelectedDragPlanePoint,
        mSelectedDragPlaneNormal);
    if (translation && translation->squaredNorm() > 1e-12) {
      const RenderableDescriptor* selectedDescriptor
          = findRenderableDescriptor(descriptors, mSelectedRenderableId);
      if (selectedDescriptor != nullptr
          && translateRenderableAndApplyIk(scene, *selectedDescriptor, *translation)) {
        mSelectedDragLastRay = ray;
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
    }
  }

  if (!isLeftMousePressed && mWasLeftMousePressed) {
    const double dragDistance
        = std::hypot(cursorX - mLeftMousePressX, cursorY - mLeftMousePressY);
    if (!mLeftMouseStartedOnPanel && !mLeftMouseStartedDrag
        && dragDistance < 4.0) {
      const auto hit = pickNearestRenderable(
          descriptors,
          makePerspectivePickRay(
              camera, cursorX, cursorY, framebufferWidth, framebufferHeight));
      if (hit) {
        mSelectedRenderableId = hit->id;
        const RenderableDescriptor& descriptor = descriptors[hit->renderableIndex];
        mSelectedLabel = selectionLabelForRenderable(scene, descriptor);
      } else {
        clear();
      }
    }
    mLeftMouseStartedDrag = false;
  }

  mWasLeftMousePressed = isLeftMousePressed;
}

} // namespace dart::gui::experimental::filament
