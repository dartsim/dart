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

#include "frame_viewport.hpp"
#include "input.hpp"
#include "scenes.hpp"

#include <dart/gui/gizmo.hpp>
#include <dart/gui/interaction.hpp>
#include <dart/gui/renderable.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/joint.hpp>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <utility>

#include <cmath>

namespace dart::gui::detail {

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

bool solveIkHandle(IkHandle& handle);

namespace {

constexpr double kRotationRadiansPerPixel = 0.01;
constexpr double kGizmoWorldScale = 1.0;

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

std::size_t pointerPaneIndex(
    const FrameViewport& viewport, double cursorX, double cursorY)
{
  const std::optional<std::size_t> hovered
      = viewportPaneIndexAtCursor(viewport, cursorX, cursorY);
  return hovered.value_or(activeViewportPaneIndex(viewport));
}

PickRay makePanePickRay(
    const ViewportPaneFrame& pane, double cursorX, double cursorY)
{
  const double localCursorX = std::clamp(
      cursorX - static_cast<double>(pane.x),
      0.0,
      static_cast<double>(std::max(1, pane.width)));
  const double localCursorY = std::clamp(
      cursorY - static_cast<double>(pane.y),
      0.0,
      static_cast<double>(std::max(1, pane.height)));
  return makePerspectivePickRay(
      pane.camera, localCursorX, localCursorY, pane.width, pane.height);
}

void getFramebufferCursorPosition(
    GLFWwindow* window, const FrameViewport& viewport, double& x, double& y)
{
  glfwGetCursorPos(window, &x, &y);
  int windowWidth = viewport.width;
  int windowHeight = viewport.height;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  if (windowWidth > 0) {
    x *= static_cast<double>(viewport.width) / static_cast<double>(windowWidth);
  }
  if (windowHeight > 0) {
    y *= static_cast<double>(viewport.height)
         / static_cast<double>(windowHeight);
  }
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

bool isAltDown(GLFWwindow* window)
{
  return isKeyDown(window, GLFW_KEY_LEFT_ALT)
         || isKeyDown(window, GLFW_KEY_RIGHT_ALT);
}

bool isShiftDown(GLFWwindow* window)
{
  return isKeyDown(window, GLFW_KEY_LEFT_SHIFT)
         || isKeyDown(window, GLFW_KEY_RIGHT_SHIFT);
}

bool isBodyNodeDragModifierDown(GLFWwindow* window)
{
  return isAltDown(window) || isDragModifierDown(window) || isShiftDown(window);
}

bool isBodyNodeRotationModifierDown(GLFWwindow* window)
{
  return isDragModifierDown(window) && !isShiftDown(window);
}

bool isBodyNodeParentJointModifierDown(GLFWwindow* window)
{
  return isShiftDown(window) && !isDragModifierDown(window);
}

dart::dynamics::BodyNode* bodyNodeForDescriptor(
    const RenderableDescriptor& descriptor)
{
  const auto skeleton = descriptor.skeleton.lock();
  if (skeleton == nullptr || descriptor.bodyName.empty()) {
    return nullptr;
  }

  return const_cast<dart::dynamics::BodyNode*>(
      skeleton->getBodyNode(descriptor.bodyName));
}

const dart::gui::BodyNodeDragHandle* findBodyNodeDragHandle(
    const DartScene& scene, const RenderableDescriptor& descriptor)
{
  auto* bodyNode = bodyNodeForDescriptor(descriptor);
  if (bodyNode == nullptr) {
    return nullptr;
  }

  const auto handle = std::find_if(
      scene.bodyNodeDragHandles.begin(),
      scene.bodyNodeDragHandles.end(),
      [&](const dart::gui::BodyNodeDragHandle& candidate) {
        return candidate.bodyNode == bodyNode;
      });
  return handle == scene.bodyNodeDragHandles.end() ? nullptr : &*handle;
}

std::string selectionLabelForBodyNodeDragHandle(
    const dart::gui::BodyNodeDragHandle& handle,
    const RenderableDescriptor& descriptor)
{
  if (!handle.label.empty()) {
    return handle.label + " body drag";
  }
  if (!descriptor.bodyName.empty()) {
    return descriptor.bodyName + " body drag";
  }
  return "body drag";
}

void configureBodyNodeDragIkDofs(
    dart::dynamics::InverseKinematics& ik,
    dart::dynamics::BodyNode& bodyNode,
    bool parentJointOnly,
    bool useWholeBody)
{
  if (parentJointOnly) {
    std::vector<std::size_t> dofs;
    if (auto* joint = bodyNode.getParentJoint()) {
      dofs.reserve(joint->getNumDofs());
      for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
        if (auto* dof = joint->getDof(i)) {
          dofs.push_back(dof->getIndexInSkeleton());
        }
      }
    }
    ik.setDofs(dofs);
    return;
  }

  if (useWholeBody) {
    ik.useWholeBody();
  } else {
    ik.useChain();
  }
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
    solveIkHandle(*handle);
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
  if (const auto* handle = findBodyNodeDragHandle(scene, descriptor)) {
    return selectionLabelForBodyNodeDragHandle(*handle, descriptor);
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
    solveIkHandle(*handle);
  }

  return true;
}

bool solveIkHandle(IkHandle& handle)
{
  if (!handle.ik) {
    return true;
  }

  if (handle.solveMode
      == dart::gui::InverseKinematicsSolveMode::SkeletonHierarchy) {
    auto* node = handle.ik->getNode();
    const auto bodyNode = node ? node->getBodyNodePtr() : nullptr;
    const auto skeleton = bodyNode ? bodyNode->getSkeleton() : nullptr;
    if (skeleton) {
      const auto wholeBodyIk = skeleton->getIK(true);
      if (wholeBodyIk) {
        return wholeBodyIk->solveAndApply(true);
      }
    }
  }

  handle.ik->getSolver()->setNumMaxIterations(30);
  return handle.ik->solveAndApply(true);
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

  return solveIkHandle(handle);
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

RenderableId SelectionController::selectionDebugRenderableId() const
{
  return mSelectionBoundsVisible ? mSelectedRenderableId : 0;
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
  mSelectionBoundsVisible = true;
}

void SelectionController::clear()
{
  mSelectedRenderableId = 0;
  mSelectedLabel = "none";
  mSelectedPoint.reset();
  mSelectedNormal.reset();
  mSelectionBoundsVisible = true;
}

bool SelectionController::beginBodyNodeDrag(
    GLFWwindow* window,
    const OrbitCamera& camera,
    DartScene& scene,
    const RenderableDescriptor& descriptor,
    const PickRay& cursorRay,
    double cursorX,
    double cursorY,
    ViewerLifecycleState& lifecycle)
{
  const auto* handle = findBodyNodeDragHandle(scene, descriptor);
  auto* bodyNode = bodyNodeForDescriptor(descriptor);
  if (handle == nullptr || handle->bodyNode == nullptr
      || handle->bodyNode != bodyNode || bodyNode == nullptr) {
    return false;
  }

  auto ik = dart::dynamics::InverseKinematics::create(bodyNode);
  if (ik == nullptr) {
    return false;
  }
  ik->setGradientMethod<dart::dynamics::InverseKinematics::JacobianTranspose>();
  ik->getSolver()->setNumMaxIterations(30);

  ActiveBodyNodeDrag bodyDrag;
  bodyDrag.bodyNode = bodyNode;
  bodyDrag.ik = std::move(ik);
  bodyDrag.pivot = bodyNode->getWorldTransform().translation();
  bodyDrag.savedRotation = bodyNode->getWorldTransform().rotation();
  bodyDrag.savedGlobalOffset = Eigen::Vector3d::Zero();
  const Eigen::Vector3d dragPoint
      = mSelectedPoint.value_or(descriptor.worldTransform.translation());
  bodyDrag.savedGlobalOffset = dragPoint - bodyDrag.pivot;
  bodyDrag.savedLocalOffset
      = bodyDrag.savedRotation.transpose() * bodyDrag.savedGlobalOffset;
  bodyDrag.targetTransform = bodyNode->getWorldTransform();
  bodyDrag.targetTransform.translation()
      = bodyDrag.pivot + bodyDrag.savedGlobalOffset;
  configureBodyNodeDragIkDofs(
      *bodyDrag.ik,
      *bodyNode,
      isBodyNodeParentJointModifierDown(window),
      handle->useWholeBody);

  mActiveBodyNodeDrag = std::move(bodyDrag);
  mLeftMouseStartedDrag = true;
  mSelectedLabel = selectionLabelForBodyNodeDragHandle(*handle, descriptor);
  mSelectionBoundsVisible = false;
  mSelectedDragLastCursorX = cursorX;
  mSelectedDragLastCursorY = cursorY;
  mSelectedDragLastRay = cursorRay;
  mSelectedDragPlanePoint = bodyDrag.targetTransform.translation();
  mSelectedDragIsAxisConstrained = false;

  if (isBodyNodeRotationModifierDown(window)) {
    mSelectedDragMode = DragMode::BodyRotate;
    mSelectedRotationAxis = selectedRotationAxis(window, camera, descriptor);
  } else {
    mSelectedDragMode = DragMode::BodyTranslate;
    if (const auto axis = selectedDragAxisFromKeyboard(window)) {
      if (computeAxisDragTranslation(
              cursorRay, cursorRay, mSelectedDragPlanePoint, *axis)) {
        mSelectedDragIsAxisConstrained = true;
        mSelectedDragAxisDirection = *axis;
      }
    }
    if (!mSelectedDragIsAxisConstrained) {
      const auto basis = makeOrbitCameraBasis(camera);
      mSelectedDragPlaneNormal = basis.forward;
      if (!intersectPlane(
              cursorRay, mSelectedDragPlanePoint, mSelectedDragPlaneNormal)) {
        mActiveBodyNodeDrag.reset();
        mLeftMouseStartedDrag = false;
        return false;
      }
    }
  }

  lifecycle.paused = true;
  return true;
}

bool SelectionController::applyBodyNodeDragTranslation(
    const Eigen::Vector3d& worldTranslation)
{
  if (!mActiveBodyNodeDrag || !worldTranslation.allFinite()
      || mActiveBodyNodeDrag->bodyNode == nullptr
      || mActiveBodyNodeDrag->ik == nullptr) {
    return false;
  }

  Eigen::Isometry3d target = mActiveBodyNodeDrag->targetTransform;
  target.linear() = mActiveBodyNodeDrag->savedRotation;
  target.translation() += worldTranslation;
  mActiveBodyNodeDrag->targetTransform = target;
  mActiveBodyNodeDrag->ik->setOffset(mActiveBodyNodeDrag->savedLocalOffset);
  mActiveBodyNodeDrag->ik->getTarget()->setTransform(target);
  return mActiveBodyNodeDrag->ik->solveAndApply(true);
}

bool SelectionController::applyBodyNodeDragRotation(
    const Eigen::Vector3d& worldAxis, double angle)
{
  if (!mActiveBodyNodeDrag || !worldAxis.allFinite() || !std::isfinite(angle)
      || mActiveBodyNodeDrag->bodyNode == nullptr
      || mActiveBodyNodeDrag->ik == nullptr) {
    return false;
  }

  const double axisNorm = worldAxis.norm();
  if (axisNorm <= 1e-12) {
    return false;
  }

  mActiveBodyNodeDrag->targetTransform.translation()
      = mActiveBodyNodeDrag->pivot;
  mActiveBodyNodeDrag->targetTransform.linear()
      = Eigen::AngleAxisd(angle, worldAxis / axisNorm).toRotationMatrix()
        * mActiveBodyNodeDrag->targetTransform.linear();
  mActiveBodyNodeDrag->ik->setOffset();
  mActiveBodyNodeDrag->ik->getTarget()->setTransform(
      mActiveBodyNodeDrag->targetTransform);
  return mActiveBodyNodeDrag->ik->solveAndApply(true);
}

namespace {

// Mouse-spring (force-drag) tuning. These are viewer-tunable: kp is the spring
// stiffness pulling the grabbed point toward the cursor target, kd damps the
// point velocity to suppress oscillation. Defaults are chosen to feel snappy on
// the ~1 kg demo bodies without exploding the integrator; tune kp up for a
// stiffer grab and kd up if a dragged body rings.
constexpr double kForceDragStiffness = 60.0;
constexpr double kForceDragDamping = 8.0;

// Resolve the world-space point a fixed depth along a pick ray. The depth is
// measured along the (normalized) ray direction so the drag target tracks the
// cursor on a plane parallel to the image plane at the grab distance.
Eigen::Vector3d forceDragTargetPoint(
    const dart::gui::PickRay& ray, double rayDepth)
{
  const double directionNorm = ray.direction.norm();
  const Eigen::Vector3d unitDirection
      = directionNorm > 1e-12 ? (ray.direction / directionNorm).eval()
                              : ray.direction;
  return ray.origin + unitDirection * rayDepth;
}

} // namespace

bool SelectionController::beginForceDrag(
    DartScene& scene,
    const RenderableDescriptor& descriptor,
    const PickRay& cursorRay,
    const Eigen::Vector3d& hitPointWorld,
    ViewerLifecycleState& lifecycle)
{
  (void)lifecycle;
  auto* bodyNode = bodyNodeForDescriptor(descriptor);
  // Non-BodyNode renderables (e.g. sx SimpleFrame mirrors) need an application
  // sink. Without one there is nothing to push, so skip and let camera orbit.
  if (bodyNode == nullptr && !scene.onForceDrag) {
    return false;
  }

  const double directionNorm = cursorRay.direction.norm();
  const Eigen::Vector3d unitDirection
      = directionNorm > 1e-12 ? (cursorRay.direction / directionNorm).eval()
                              : cursorRay.direction;

  ActiveForceDrag drag;
  drag.renderableId = descriptor.id;
  drag.renderableName = descriptor.shapeFrameName.empty()
                            ? descriptor.bodyName
                            : descriptor.shapeFrameName;
  drag.bodyNode = bodyNode;
  // Record the grab offset in the owner body's local frame so the application
  // point tracks the body as it rotates, yielding torque for off-center grabs.
  // Shape-node descriptors may be offset from the body origin, so legacy
  // BodyNode renderables must use the BodyNode transform instead of the
  // descriptor's shape-frame transform.
  const Eigen::Isometry3d ownerTransform = bodyNode != nullptr
                                               ? bodyNode->getWorldTransform()
                                               : descriptor.worldTransform;
  drag.savedLocalOffset = ownerTransform.linear().transpose()
                          * (hitPointWorld - ownerTransform.translation());
  drag.rayDepth = unitDirection.dot(hitPointWorld - cursorRay.origin);

  mActiveForceDrag = std::move(drag);
  mLeftMouseStartedDrag = true;
  mSelectedDragMode = DragMode::Force;
  mSelectionBoundsVisible = false;
  return true;
}

void SelectionController::updateForceDrag(
    DartScene& scene,
    const std::vector<RenderableDescriptor>& descriptors,
    const PickRay& cursorRay)
{
  if (!mActiveForceDrag) {
    return;
  }

  const Eigen::Vector3d target
      = forceDragTargetPoint(cursorRay, mActiveForceDrag->rayDepth);
  if (!target.allFinite()) {
    return;
  }

  if (mActiveForceDrag->bodyNode != nullptr) {
    // Legacy path: the grabbed point is the body origin offset by the saved
    // local grab offset (rotated into the world); apply a spring-damper force
    // there every frame via the in-engine external-force buffer.
    dart::dynamics::BodyNode* bodyNode = mActiveForceDrag->bodyNode;
    const Eigen::Isometry3d bodyTransform = bodyNode->getWorldTransform();
    const Eigen::Vector3d worldOffset
        = bodyTransform.linear() * mActiveForceDrag->savedLocalOffset;
    const Eigen::Vector3d appPoint = bodyTransform.translation() + worldOffset;
    const Eigen::Vector3d pointVelocity
        = bodyNode->getLinearVelocity(mActiveForceDrag->savedLocalOffset);
    Eigen::Vector3d force = kForceDragStiffness * (target - appPoint)
                            - kForceDragDamping * pointVelocity;
    if (!force.allFinite()) {
      return;
    }
    bodyNode->addExtForce(
        force, appPoint, /*isForceLocal=*/false, /*isOffsetLocal=*/false);
    return;
  }

  // Non-BodyNode path: we cannot read the body's pose or velocity from here, so
  // recompute the application point from the renderable's last-known transform
  // (refreshed each frame in the descriptor list) and omit damping — the
  // application owns the body and applies the one-shot force on its next step.
  if (!scene.onForceDrag) {
    return;
  }
  const RenderableDescriptor* descriptor
      = findRenderableDescriptor(descriptors, mActiveForceDrag->renderableId);
  if (descriptor == nullptr) {
    return;
  }
  const Eigen::Isometry3d& bodyTransform = descriptor->worldTransform;
  const Eigen::Vector3d appPoint
      = bodyTransform.translation()
        + bodyTransform.linear() * mActiveForceDrag->savedLocalOffset;
  const Eigen::Vector3d force = kForceDragStiffness * (target - appPoint);
  if (!force.allFinite()) {
    return;
  }

  dart::gui::ForceDragEvent event;
  event.renderableId = mActiveForceDrag->renderableId;
  event.renderableName = mActiveForceDrag->renderableName;
  event.applicationPoint = appPoint;
  event.force = force;
  event.active = true;
  scene.onForceDrag(event);
}

void SelectionController::endForceDrag(DartScene& scene)
{
  if (!mActiveForceDrag) {
    return;
  }
  // Signal a final inactive event so non-BodyNode applications stop reapplying
  // their one-shot force. Legacy bodies need no teardown (the buffer clears on
  // the next step), but the inactive event is harmless for them too.
  if (mActiveForceDrag->bodyNode == nullptr && scene.onForceDrag) {
    dart::gui::ForceDragEvent event;
    event.renderableId = mActiveForceDrag->renderableId;
    event.renderableName = mActiveForceDrag->renderableName;
    event.active = false;
    scene.onForceDrag(event);
  }
  mActiveForceDrag.reset();
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

bool SelectionController::updateMouseSelection(
    GLFWwindow* window,
    const FrameViewport& viewport,
    bool showUi,
    bool uiCapturesMouse,
    double guiScale,
    DartScene& scene,
    std::vector<RenderableDescriptor>& descriptors,
    ViewerLifecycleState& lifecycle)
{
  if (window == nullptr) {
    return false;
  }
  (void)showUi;
  (void)guiScale;

  double cursorX = 0.0;
  double cursorY = 0.0;
  getFramebufferCursorPosition(window, viewport, cursorX, cursorY);
  const bool isLeftMousePressed
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  const std::size_t hoveredPaneIndex
      = pointerPaneIndex(viewport, cursorX, cursorY);
  if (isLeftMousePressed && !mWasLeftMousePressed) {
    mActivePointerPaneIndex = hoveredPaneIndex;
  }
  const std::size_t inputPaneIndex
      = isLeftMousePressed && mActivePointerPaneIndex.has_value()
            ? *mActivePointerPaneIndex
            : hoveredPaneIndex;
  const std::size_t maxPaneIndex
      = viewport.paneCount > 0
            ? std::min(
                  viewport.paneCount - 1u, dart::gui::kMaxViewportPanes - 1u)
            : 0u;
  const ViewportPaneFrame& inputPane
      = viewport.panes[std::min(inputPaneIndex, maxPaneIndex)];
  const OrbitCamera& camera = inputPane.camera;
  const bool cursorOverPanel = uiCapturesMouse;
  const PickRay cursorRay = makePanePickRay(inputPane, cursorX, cursorY);
  const bool gizmoDragActive
      = mLeftMouseStartedDrag
        && (mSelectedDragMode == DragMode::GizmoTranslateAxis
            || mSelectedDragMode == DragMode::GizmoTranslatePlane
            || mSelectedDragMode == DragMode::GizmoRotateAxis);
  if (gizmoDragActive) {
    mHoveredGizmoHandle.reset();
  } else if (!cursorOverPanel) {
    mHoveredGizmoHandle
        = pickNearestGizmoHandle(scene.gizmos, cursorRay, kGizmoWorldScale);
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
          = pickNearestGizmoHandle(scene.gizmos, cursorRay, kGizmoWorldScale)) {
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
        && isBodyNodeDragModifierDown(window)) {
      const RenderableDescriptor* selectedDescriptor = nullptr;
      if (mSelectedRenderableId != 0) {
        selectedDescriptor
            = findRenderableDescriptor(descriptors, mSelectedRenderableId);
      }

      if (selectedDescriptor == nullptr
          || findBodyNodeDragHandle(scene, *selectedDescriptor) == nullptr) {
        if (const auto hit = pickNearestRenderable(descriptors, cursorRay)) {
          selectedDescriptor = &descriptors[hit->renderableIndex];
          mSelectedRenderableId = hit->id;
          mSelectionBoundsVisible = true;
          mSelectedPoint = hit->point;
          mSelectedNormal = hit->normal;
          mSelectedLabel = selectionLabelForRenderable(
              scene, descriptors[hit->renderableIndex]);
        }
      }

      if (selectedDescriptor != nullptr) {
        beginBodyNodeDrag(
            window,
            camera,
            scene,
            *selectedDescriptor,
            cursorRay,
            cursorX,
            cursorY,
            lifecycle);
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

    // Plain left-press (no kinematic-drag modifier) over a renderable begins a
    // mouse "force-drag": a spring that pushes/pulls the picked body. A hit on
    // empty space leaves mLeftMouseStartedDrag false so camera orbit still
    // runs.
    if (!mLeftMouseStartedOnPanel && !mLeftMouseStartedDrag
        && !isBodyNodeDragModifierDown(window) && !isDragModifierDown(window)) {
      if (const auto hit = pickNearestRenderable(descriptors, cursorRay)) {
        const RenderableDescriptor& descriptor
            = descriptors[hit->renderableIndex];
        if (beginForceDrag(
                scene, descriptor, cursorRay, hit->point, lifecycle)) {
          mSelectedRenderableId = hit->id;
          mSelectedPoint = hit->point;
          mSelectedNormal = hit->normal;
          mSelectedLabel = selectionLabelForRenderable(scene, descriptor);
        }
      }
    }
  }

  if (isLeftMousePressed && mLeftMouseStartedDrag) {
    if (mSelectedDragMode == DragMode::Force) {
      const PickRay ray = makePanePickRay(inputPane, cursorX, cursorY);
      updateForceDrag(scene, descriptors, ray);
    } else if (mSelectedDragMode == DragMode::Rotate) {
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
    } else if (mSelectedDragMode == DragMode::BodyRotate) {
      const double cursorDelta = (cursorX - mSelectedDragLastCursorX)
                                 - (cursorY - mSelectedDragLastCursorY);
      const double angle = cursorDelta * kRotationRadiansPerPixel;
      if (std::abs(angle) > 1e-12
          && applyBodyNodeDragRotation(mSelectedRotationAxis, angle)) {
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
      mSelectedDragLastCursorX = cursorX;
      mSelectedDragLastCursorY = cursorY;
    } else if (mSelectedDragMode == DragMode::BodyTranslate) {
      const PickRay ray = makePanePickRay(inputPane, cursorX, cursorY);
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
      if (translation && translation->squaredNorm() > 1e-12
          && applyBodyNodeDragTranslation(*translation)) {
        mSelectedDragLastRay = ray;
        lifecycle.paused = true;
        descriptors = extractRenderables(*scene.world);
      }
    } else if (mSelectedDragMode == DragMode::GizmoTranslateAxis) {
      const PickRay ray = makePanePickRay(inputPane, cursorX, cursorY);
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
      const PickRay ray = makePanePickRay(inputPane, cursorX, cursorY);
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
      const PickRay ray = makePanePickRay(inputPane, cursorX, cursorY);
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

  bool selectionCommitted = false;
  if (!isLeftMousePressed && mWasLeftMousePressed) {
    const double dragDistance
        = std::hypot(cursorX - mLeftMousePressX, cursorY - mLeftMousePressY);
    if (!mLeftMouseStartedOnPanel && !mLeftMouseStartedDrag
        && dragDistance < 4.0) {
      selectionCommitted = true;
      const auto hit = pickNearestRenderable(descriptors, cursorRay);
      if (hit) {
        mSelectedRenderableId = hit->id;
        mSelectionBoundsVisible = true;
        mSelectedPoint = hit->point;
        mSelectedNormal = hit->normal;
        const RenderableDescriptor& descriptor
            = descriptors[hit->renderableIndex];
        mSelectedLabel = selectionLabelForRenderable(scene, descriptor);
      } else {
        clear();
      }
    }
    endForceDrag(scene);
    mLeftMouseStartedDrag = false;
    mSelectedDragMode = DragMode::Translate;
    mActiveGizmoIndex = 0u;
    mActiveGizmoHandle.reset();
    mActiveBodyNodeDrag.reset();
    mActivePointerPaneIndex.reset();
  }

  mWasLeftMousePressed = isLeftMousePressed;
  return selectionCommitted;
}

} // namespace dart::gui::detail
