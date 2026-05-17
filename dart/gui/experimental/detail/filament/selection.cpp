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

#include <dart/gui/gizmo.hpp>
#include <dart/gui/interaction.hpp>
#include <dart/gui/renderable.hpp>

#include <dart/dynamics/inverse_kinematics.hpp>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <utility>

#include <cmath>

namespace dart::gui::filament {

using dart::gui::computeAxisDragTranslation;
using dart::gui::computePlaneDragTranslation;
using dart::gui::extractRenderables;
using dart::gui::intersectPlane;
using dart::gui::makeOrbitCameraBasis;
using dart::gui::makePerspectivePickRay;
using dart::gui::OrbitCamera;
using dart::gui::pickNearestGizmoHandle;
using dart::gui::pickNearestRenderable;
using dart::gui::PickRay;
using dart::gui::RenderableDescriptor;
using dart::gui::RenderableId;
using dart::gui::rotateFrameRenderable;
using dart::gui::rotateGizmoTarget;
using dart::gui::translateFrameRenderable;
using dart::gui::translateGizmoTarget;
using dart::gui::ViewerLifecycleState;

namespace {

constexpr double kRotationRadiansPerPixel = 0.01;

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

std::optional<int> selectedRotationAxisIndexFromKeyboard(GLFWwindow* window)
{
  if (isKeyDown(window, GLFW_KEY_X) || isKeyDown(window, GLFW_KEY_1)) {
    return 0;
  }
  if (isKeyDown(window, GLFW_KEY_Y) || isKeyDown(window, GLFW_KEY_2)) {
    return 1;
  }
  if (isKeyDown(window, GLFW_KEY_Z) || isKeyDown(window, GLFW_KEY_3)) {
    return 2;
  }

  return std::nullopt;
}

Eigen::Vector3d selectedRotationAxis(
    GLFWwindow* window,
    const OrbitCamera& camera,
    const RenderableDescriptor& descriptor)
{
  if (const auto axisIndex = selectedRotationAxisIndexFromKeyboard(window)) {
    Eigen::Vector3d axis = descriptor.worldTransform.linear().col(*axisIndex);
    const double axisNorm = axis.norm();
    if (axis.allFinite() && axisNorm > 1e-12) {
      return axis / axisNorm;
    }
  }

  const auto basis = makeOrbitCameraBasis(camera);
  return basis.forward;
}

bool rotateRenderableAndApplyIk(
    DartScene& scene,
    const RenderableDescriptor& descriptor,
    const Eigen::Vector3d& worldAxis,
    double angle)
{
  if (!rotateFrameRenderable(descriptor, worldAxis, angle)) {
    return false;
  }

  if (auto* handle = findIkHandle(scene, descriptor.id)) {
    if (handle->ik) {
      handle->ik->getSolver()->setNumMaxIterations(30);
      handle->ik->solveAndApply(true);
    }
  }

  return true;
}

std::string selectionLabelForGizmo(const dart::gui::Gizmo& gizmo)
{
  return (gizmo.label.empty() ? std::string("gizmo") : gizmo.label) + " gizmo";
}

bool isRotationGizmoHandle(dart::gui::GizmoHandleKind handle)
{
  return handle == dart::gui::GizmoHandleKind::RotateX
         || handle == dart::gui::GizmoHandleKind::RotateY
         || handle == dart::gui::GizmoHandleKind::RotateZ;
}

bool isPlaneTranslationGizmoHandle(dart::gui::GizmoHandleKind handle)
{
  return handle == dart::gui::GizmoHandleKind::TranslateXY
         || handle == dart::gui::GizmoHandleKind::TranslateYZ
         || handle == dart::gui::GizmoHandleKind::TranslateXZ;
}

} // namespace

IkHandle* findIkHandle(DartScene& scene, RenderableId targetRenderableId)
{
  const auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [&](const IkHandle& candidate) {
        return candidate.targetRenderableId == targetRenderableId;
      });
  return handle == scene.ikHandles.end() ? nullptr : &*handle;
}

const IkHandle* findIkHandle(
    const DartScene& scene, RenderableId targetRenderableId)
{
  const auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [&](const IkHandle& candidate) {
        return candidate.targetRenderableId == targetRenderableId;
      });
  return handle == scene.ikHandles.end() ? nullptr : &*handle;
}

std::string selectionLabelForRenderable(
    const DartScene& scene, const RenderableDescriptor& descriptor)
{
  if (const auto* handle = findIkHandle(scene, descriptor.id)) {
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

  if (auto* handle = findIkHandle(scene, descriptor.id)) {
    if (handle->ik) {
      handle->ik->getSolver()->setNumMaxIterations(30);
      handle->ik->solveAndApply(true);
    }
  }

  return true;
}

bool translateIkHandleTargetAndApplyIk(
    IkHandle& handle, const Eigen::Vector3d& worldTranslation)
{
  if (handle.target == nullptr) {
    return false;
  }

  dart::gui::Gizmo gizmo;
  gizmo.target = handle.target;
  if (!translateGizmoTarget(gizmo, worldTranslation)) {
    return false;
  }

  if (handle.ik) {
    handle.ik->getSolver()->setNumMaxIterations(30);
    handle.ik->solveAndApply(true);
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

const std::optional<Eigen::Vector3d>& SelectionController::selectedPoint() const
{
  return mSelectedPoint;
}

const std::optional<Eigen::Vector3d>& SelectionController::selectedNormal()
    const
{
  return mSelectedNormal;
}

std::optional<dart::gui::GizmoHandleHit>
SelectionController::highlightedGizmoHandle() const
{
  if (mActiveGizmoHandle) {
    return mActiveGizmoHandle;
  }
  return mHoveredGizmoHandle;
}

bool SelectionController::isDraggingSelection() const
{
  return mLeftMouseStartedDrag;
}

void SelectionController::select(RenderableId renderableId, std::string label)
{
  mSelectedRenderableId = renderableId;
  mSelectedLabel = std::move(label);
  mSelectedPoint.reset();
  mSelectedNormal.reset();
}

void SelectionController::clear()
{
  mSelectedRenderableId = 0;
  mSelectedLabel = "none";
  mSelectedPoint.reset();
  mSelectedNormal.reset();
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
  if (selectedDescriptor != nullptr) {
    if (!translateRenderableAndApplyIk(scene, *selectedDescriptor, nudge)) {
      return;
    }
  } else {
    auto* handle = findIkHandle(scene, mSelectedRenderableId);
    if (handle == nullptr
        || !translateIkHandleTargetAndApplyIk(*handle, nudge)) {
      return;
    }
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
  const bool cursorOverPanel
      = showUi && isInsideStatusPanel(cursorX, cursorY, guiScale);
  const PickRay cursorRay = makePerspectivePickRay(
      camera, cursorX, cursorY, framebufferWidth, framebufferHeight);
  const bool gizmoDragActive
      = mLeftMouseStartedDrag
        && (mSelectedDragMode == DragMode::GizmoTranslateAxis
            || mSelectedDragMode == DragMode::GizmoTranslatePlane
            || mSelectedDragMode == DragMode::GizmoRotateAxis);
  if (gizmoDragActive) {
    mHoveredGizmoHandle.reset();
  } else if (!cursorOverPanel) {
    mHoveredGizmoHandle
        = pickNearestGizmoHandle(scene.gizmos, cursorRay, guiScale);
  } else {
    mHoveredGizmoHandle.reset();
  }

  if (isLeftMousePressed && !mWasLeftMousePressed) {
    mLeftMousePressX = cursorX;
    mLeftMousePressY = cursorY;
    mLeftMouseStartedDrag = false;
    mSelectedDragIsAxisConstrained = false;
    mLeftMouseStartedOnPanel = cursorOverPanel;

    if (!mLeftMouseStartedOnPanel) {
      if (const auto gizmoHit
          = pickNearestGizmoHandle(scene.gizmos, cursorRay, guiScale)) {
        mLeftMouseStartedDrag = true;
        if (isRotationGizmoHandle(gizmoHit->handle)) {
          mSelectedDragMode = DragMode::GizmoRotateAxis;
        } else if (isPlaneTranslationGizmoHandle(gizmoHit->handle)) {
          mSelectedDragMode = DragMode::GizmoTranslatePlane;
        } else {
          mSelectedDragMode = DragMode::GizmoTranslateAxis;
        }
        mSelectedDragIsAxisConstrained = true;
        mActiveGizmoIndex = gizmoHit->gizmoIndex;
        mActiveGizmoHandle = *gizmoHit;
        mSelectedDragLastRay = cursorRay;
        mSelectedDragLastCursorX = cursorX;
        mSelectedDragLastCursorY = cursorY;
        mSelectedDragPlanePoint = gizmoHit->point;
        mSelectedDragAxisDirection = gizmoHit->axis;
        mSelectedDragPlaneNormal = gizmoHit->axis;
        mSelectedRotationAxis = gizmoHit->axis;
        mSelectedRenderableId = 0;
        mSelectedPoint = gizmoHit->point;
        mSelectedNormal.reset();
        mSelectedLabel
            = selectionLabelForGizmo(scene.gizmos[mActiveGizmoIndex]);
        lifecycle.paused = true;
      }
    }

    if (!mLeftMouseStartedOnPanel && !mLeftMouseStartedDrag
        && mSelectedRenderableId != 0 && isDragModifierDown(window)) {
      const RenderableDescriptor* selectedDescriptor
          = findRenderableDescriptor(descriptors, mSelectedRenderableId);
      if (selectedDescriptor != nullptr) {
        if (isRotationDragModifierDown(window)
            && !selectedDescriptor->simpleFrame.expired()) {
          mLeftMouseStartedDrag = true;
          mSelectedDragMode = DragMode::Rotate;
          mSelectedRotationAxis
              = selectedRotationAxis(window, camera, *selectedDescriptor);
          mSelectedDragLastCursorX = cursorX;
          mSelectedDragLastCursorY = cursorY;
          lifecycle.paused = true;
        } else if (!isRotationDragModifierDown(window)) {
          const auto basis = makeOrbitCameraBasis(camera);
          const Eigen::Vector3d planePoint
              = selectedDescriptor->worldTransform.translation();
          const Eigen::Vector3d planeNormal = basis.forward;
          if (const auto axis = selectedDragAxisFromKeyboard(window)) {
            if (computeAxisDragTranslation(
                    cursorRay, cursorRay, planePoint, *axis)) {
              mLeftMouseStartedDrag = true;
              mSelectedDragMode = DragMode::Translate;
              mSelectedDragIsAxisConstrained = true;
              mSelectedDragLastRay = cursorRay;
              mSelectedDragPlanePoint = planePoint;
              mSelectedDragAxisDirection = *axis;
              lifecycle.paused = true;
            }
          } else if (intersectPlane(cursorRay, planePoint, planeNormal)) {
            mLeftMouseStartedDrag = true;
            mSelectedDragMode = DragMode::Translate;
            mSelectedDragLastRay = cursorRay;
            mSelectedDragPlanePoint = planePoint;
            mSelectedDragPlaneNormal = planeNormal;
            lifecycle.paused = true;
          }
        }
      }
    }
  }

  if (isLeftMousePressed && mLeftMouseStartedDrag) {
    if (mSelectedDragMode == DragMode::Rotate) {
      const double cursorDelta = (cursorX - mSelectedDragLastCursorX)
                                 - (cursorY - mSelectedDragLastCursorY);
      const double angle = cursorDelta * kRotationRadiansPerPixel;
      const RenderableDescriptor* selectedDescriptor
          = findRenderableDescriptor(descriptors, mSelectedRenderableId);
      if (std::abs(angle) > 1e-12 && selectedDescriptor != nullptr
          && rotateRenderableAndApplyIk(
              scene, *selectedDescriptor, mSelectedRotationAxis, angle)) {
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
      mSelectedDragLastCursorX = cursorX;
      mSelectedDragLastCursorY = cursorY;
    } else if (mSelectedDragMode == DragMode::GizmoTranslateAxis) {
      const PickRay ray = makePerspectivePickRay(
          camera, cursorX, cursorY, framebufferWidth, framebufferHeight);
      const auto translation = computeAxisDragTranslation(
          mSelectedDragLastRay,
          ray,
          mSelectedDragPlanePoint,
          mSelectedDragAxisDirection);
      if (translation && translation->squaredNorm() > 1e-12
          && mActiveGizmoIndex < scene.gizmos.size()
          && translateGizmoTarget(
              scene.gizmos[mActiveGizmoIndex], *translation)) {
        mSelectedDragLastRay = ray;
        mSelectedPoint = scene.gizmos[mActiveGizmoIndex]
                             .target->getWorldTransform()
                             .translation();
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
    } else if (mSelectedDragMode == DragMode::GizmoTranslatePlane) {
      const PickRay ray = makePerspectivePickRay(
          camera, cursorX, cursorY, framebufferWidth, framebufferHeight);
      const auto translation = computePlaneDragTranslation(
          mSelectedDragLastRay,
          ray,
          mSelectedDragPlanePoint,
          mSelectedDragPlaneNormal);
      if (translation && translation->squaredNorm() > 1e-12
          && mActiveGizmoIndex < scene.gizmos.size()
          && translateGizmoTarget(
              scene.gizmos[mActiveGizmoIndex], *translation)) {
        mSelectedDragLastRay = ray;
        mSelectedPoint = scene.gizmos[mActiveGizmoIndex]
                             .target->getWorldTransform()
                             .translation();
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
    } else if (mSelectedDragMode == DragMode::GizmoRotateAxis) {
      const double cursorDelta = (cursorX - mSelectedDragLastCursorX)
                                 - (cursorY - mSelectedDragLastCursorY);
      const double angle = cursorDelta * kRotationRadiansPerPixel;
      if (std::abs(angle) > 1e-12 && mActiveGizmoIndex < scene.gizmos.size()
          && rotateGizmoTarget(
              scene.gizmos[mActiveGizmoIndex], mSelectedRotationAxis, angle)) {
        mSelectedPoint = scene.gizmos[mActiveGizmoIndex]
                             .target->getWorldTransform()
                             .translation();
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
      mSelectedDragLastCursorX = cursorX;
      mSelectedDragLastCursorY = cursorY;
    } else {
      const PickRay ray = makePerspectivePickRay(
          camera, cursorX, cursorY, framebufferWidth, framebufferHeight);
      const auto translation = mSelectedDragIsAxisConstrained
                                   ? computeAxisDragTranslation(
                                         mSelectedDragLastRay,
                                         ray,
                                         mSelectedDragPlanePoint,
                                         mSelectedDragAxisDirection)
                                   : computePlaneDragTranslation(
                                         mSelectedDragLastRay,
                                         ray,
                                         mSelectedDragPlanePoint,
                                         mSelectedDragPlaneNormal);
      if (translation && translation->squaredNorm() > 1e-12) {
        const RenderableDescriptor* selectedDescriptor
            = findRenderableDescriptor(descriptors, mSelectedRenderableId);
        if (selectedDescriptor != nullptr
            && translateRenderableAndApplyIk(
                scene, *selectedDescriptor, *translation)) {
          mSelectedDragLastRay = ray;
          lifecycle.paused = true;
          descriptors = extractRenderables(*scene.world);
        }
      }
    }
  }

  if (!isLeftMousePressed && mWasLeftMousePressed) {
    const double dragDistance
        = std::hypot(cursorX - mLeftMousePressX, cursorY - mLeftMousePressY);
    if (!mLeftMouseStartedOnPanel && !mLeftMouseStartedDrag
        && dragDistance < 4.0) {
      const auto hit = pickNearestRenderable(descriptors, cursorRay);
      if (hit) {
        mSelectedRenderableId = hit->id;
        mSelectedPoint = hit->point;
        mSelectedNormal = hit->normal;
        const RenderableDescriptor& descriptor
            = descriptors[hit->renderableIndex];
        mSelectedLabel = selectionLabelForRenderable(scene, descriptor);
      } else {
        clear();
      }
    }
    mLeftMouseStartedDrag = false;
    mSelectedDragMode = DragMode::Translate;
    mActiveGizmoIndex = 0u;
    mActiveGizmoHandle.reset();
  }

  mWasLeftMousePressed = isLeftMousePressed;
}

} // namespace dart::gui::filament
