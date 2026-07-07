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

#include "DragForce.hpp"

#include <algorithm>

#include <cmath>

namespace dart_demos {

namespace {
using dart::dynamics::BodyNode;
} // namespace

//==============================================================================
/// Persistent (never torn down) mouse handler, the same lifetime pattern as
/// DemoHost's DemoKeyHandler: installed once at startup and intentionally
/// never deleted (DefaultEventHandler's destructor does not delete its
/// registered MouseEventHandlers, and this one must outlive every scene
/// switch, so there is no natural per-scene owner to hand it to -- a single,
/// bounded, one-time allocation for the process lifetime).
///
/// Fires on every mouse event (DefaultEventHandler::triggerMouseEventHandlers
/// runs it on PUSH/DRAG/RELEASE/MOVE), but only *acts* on PUSH (pick once,
/// starting either a Ctrl+drag or a plain-click selection) and RELEASE
/// (end the drag); the continuous per-frame target/force computation lives in
/// DragForce::applyPreStep/applyPreRefresh instead, driven by the world's own
/// frame loop rather than by mouse-event cadence.
class DragForce::MouseHandler : public dart::gui::osg::MouseEventHandler
{
public:
  MouseHandler(
      DragForce* owner, std::function<void(BodyNode*)> onPlainClickSelect)
    : mOwner(owner), mOnPlainClickSelect(std::move(onPlainClickSelect))
  {
  }

  void update() override
  {
    if (!mEventHandler)
      return;

    // Don't let a click on an ImGui panel (e.g. the navigator, the Diagnostics
    // log) pick through to the 3D scene underneath it.
    if (ImGui::GetIO().WantCaptureMouse)
      return;

    const auto event
        = mEventHandler->getButtonEvent(dart::gui::osg::LEFT_MOUSE);

    if (event == dart::gui::osg::BUTTON_PUSH) {
      const bool ctrl = (mEventHandler->getModKeyMask()
                         & ::osgGA::GUIEventAdapter::MODKEY_CTRL)
                        != 0;
      const auto& picks = mEventHandler->getButtonPicks(
          dart::gui::osg::LEFT_MOUSE, dart::gui::osg::BUTTON_PUSH);

      BodyNode* body = nullptr;
      Eigen::Vector3d point = Eigen::Vector3d::Zero();
      if (!picks.empty()) {
        body = dynamic_cast<BodyNode*>(picks.front().frame->getParentFrame());
        point = picks.front().position;
      }

      if (ctrl) {
        if (body)
          mOwner->startDrag(body, point);
      } else if (mOnPlainClickSelect) {
        mOnPlainClickSelect(body); // body may be nullptr: click-to-deselect.
      }
    } else if (event == dart::gui::osg::BUTTON_RELEASE) {
      mOwner->endDrag();
    }
  }

private:
  DragForce* mOwner;
  std::function<void(BodyNode*)> mOnPlainClickSelect;
};

//==============================================================================
DragForce::DragForce() = default;

//==============================================================================
DragForce::~DragForce() = default;

//==============================================================================
void DragForce::installMouseHandler(
    dart::gui::osg::ImGuiViewer* viewer,
    std::function<void(BodyNode*)> onPlainClickSelect)
{
  mViewer = viewer;
  mMouseHandler = new MouseHandler(this, std::move(onPlainClickSelect));
  viewer->getDefaultEventHandler()->addMouseEventHandler(mMouseHandler);
}

//==============================================================================
void DragForce::onSceneInstalled(const dart::simulation::WorldPtr& world)
{
  mWorld = world;
  mLineFrame.reset();
  mLineShape.reset();
}

//==============================================================================
void DragForce::reset(dart::gui::osg::ImGuiViewer* viewer)
{
  endDrag();
  detachGizmo(viewer);
  if (mLineFrame && mWorld)
    mWorld->removeSimpleFrame(mLineFrame);
  mLineFrame.reset();
  mLineShape.reset();
  mWorld.reset();
  mStatusBodyName.clear();
  mStatusForceMag = 0.0;
}

//==============================================================================
void DragForce::startDrag(BodyNode* body, const Eigen::Vector3d& worldPoint)
{
  if (!body)
    return;
  mDragBody = body;
  mDragLocalPoint = body->getWorldTransform().inverse() * worldPoint;
  mDragAnchorWorld = worldPoint;
  mDragging = true;
  addSubject(mDragBody); // idempotent if it is also the gizmo body
}

//==============================================================================
void DragForce::endDrag()
{
  // Keep the subscription alive if the gizmo still references the same body.
  if (mDragBody && mDragBody != mGizmoBody)
    removeSubject(mDragBody);
  mDragging = false;
  mDragBody = nullptr;
}

//==============================================================================
Eigen::Vector3d DragForce::computeDragTarget() const
{
  if (!mViewer)
    return mDragAnchorWorld;
  auto* deh = mViewer->getDefaultEventHandler();
  if (!deh)
    return mDragAnchorWorld;

  return mDragAnchorWorld
         + deh->getDeltaCursor(mDragAnchorWorld, dart::gui::osg::UNCONSTRAINED);
}

//==============================================================================
void DragForce::applySpring(
    BodyNode* body,
    const Eigen::Vector3d& localPoint,
    const Eigen::Vector3d& target)
{
  const Eigen::Vector3d worldPoint = body->getWorldTransform() * localPoint;
  Eigen::Vector3d force = static_cast<double>(mCoeff) * (target - worldPoint);
  const double forceNorm = force.norm();
  // Never let a non-finite force reach the integrator: a NaN/Inf here (e.g. a
  // NaN body transform on a diverging scene) would permanently poison the sim,
  // since addExtForce feeds velocity/position integration before it is cleared.
  if (!force.allFinite())
    return;
  if (forceNorm > mMaxForce && forceNorm > 0.0)
    force = mMaxForce * force / forceNorm;

  body->addExtForce(force, localPoint);

  mStatusBodyName = body->getName();
  mStatusForceMag = force.norm();
}

//==============================================================================
void DragForce::applyPreStep()
{
  if (mDragging && mDragBody)
    applySpring(mDragBody, mDragLocalPoint, computeDragTarget());

  // "while running" (preStep only fires while the sim is simulating, per
  // WorldNode::refresh()): drive the gizmo spring continuously toward
  // wherever the gizmo currently sits, independent of whether the user is
  // actively grabbing it right now.
  if (mGizmoBody && mGizmoFrame) {
    applySpring(
        mGizmoBody,
        Eigen::Vector3d::Zero(),
        mGizmoFrame->getWorldTransform().translation());
  }
}

//==============================================================================
void DragForce::teleportToGizmo()
{
  if (!mGizmoBody || !mGizmoFrame)
    return;

  auto* freeJoint
      = dynamic_cast<dart::dynamics::FreeJoint*>(mGizmoBody->getParentJoint());
  if (!freeJoint)
    return; // Only a FreeJoint has a well-defined 6-DOF pose to set directly;
            // an articulated (non-free) body would need IK, which is
            // BodyNodeDnD's job (a separate, per-scene opt-in feature), not
            // this gizmo's.

  const Eigen::Isometry3d desiredWorldTf = mGizmoFrame->getWorldTransform();
  const auto* parentBody = mGizmoBody->getParentBodyNode();
  const Eigen::Isometry3d parentWorldTf = parentBody
                                              ? parentBody->getWorldTransform()
                                              : Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d relTf
      = freeJoint->getTransformFromParentBodyNode().inverse()
        * parentWorldTf.inverse() * desiredWorldTf
        * freeJoint->getTransformFromChildBodyNode();
  freeJoint->setPositions(dart::dynamics::FreeJoint::convertToPositions(relTf));
}

//==============================================================================
void DragForce::ensureLineVisual()
{
  if (mLineFrame || !mWorld)
    return;

  mLineFrame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), "drag_force_line");
  mLineShape = std::make_shared<dart::dynamics::LineSegmentShape>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 3.0f);
  mLineShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);
  mLineFrame->setShape(mLineShape);
  mLineFrame->createVisualAspect();
  mLineFrame->getVisualAspect()->setColor(
      Eigen::Vector4d(1.0, 0.55, 0.05, 1.0));
  mWorld->addSimpleFrame(mLineFrame);
}

//==============================================================================
void DragForce::hideLineVisual()
{
  if (mLineFrame)
    mLineFrame->getVisualAspect(true)->setHidden(true);
  mStatusBodyName.clear();
  mStatusForceMag = 0.0;
}

//==============================================================================
void DragForce::applyPreRefresh(bool paused)
{
  if (mDragging && mDragBody) {
    ensureLineVisual();
    const Eigen::Vector3d worldPoint
        = mDragBody->getWorldTransform() * mDragLocalPoint;
    const Eigen::Vector3d target = computeDragTarget();
    if (mLineShape) {
      mLineShape->setVertex(0, worldPoint);
      mLineShape->setVertex(1, target);
    }
    if (mLineFrame)
      mLineFrame->getVisualAspect(true)->setHidden(false);
  } else if (mGizmoBody && mGizmoFrame) {
    if (paused)
      teleportToGizmo();

    ensureLineVisual();
    if (mLineShape) {
      mLineShape->setVertex(0, mGizmoBody->getWorldTransform().translation());
      mLineShape->setVertex(1, mGizmoFrame->getWorldTransform().translation());
    }
    if (mLineFrame)
      mLineFrame->getVisualAspect(true)->setHidden(false);
  } else {
    hideLineVisual();
  }
}

//==============================================================================
void DragForce::attachGizmo(
    BodyNode* body,
    const dart::simulation::WorldPtr& world,
    dart::gui::osg::ImGuiViewer* viewer)
{
  detachGizmo(viewer);
  if (!body || !world)
    return;

  mGizmoBody = body;
  mGizmoFrame = std::make_shared<dart::gui::osg::InteractiveFrame>(
      dart::dynamics::Frame::World(),
      "gizmo_" + body->getName(),
      body->getWorldTransform());
  world->addSimpleFrame(mGizmoFrame);
  mGizmoDnd = viewer->enableDragAndDrop(mGizmoFrame.get());
  addSubject(mGizmoBody); // idempotent if it is also the drag body
}

//==============================================================================
void DragForce::detachGizmo(dart::gui::osg::ImGuiViewer* viewer)
{
  if (mGizmoDnd && viewer) {
    viewer->disableDragAndDrop(mGizmoDnd);
    mGizmoDnd = nullptr;
  }
  if (mGizmoFrame && mWorld)
    mWorld->removeSimpleFrame(mGizmoFrame);
  mGizmoFrame.reset();
  // Keep the subscription alive if a momentary drag still references it.
  if (mGizmoBody && mGizmoBody != mDragBody)
    removeSubject(mGizmoBody);
  mGizmoBody = nullptr;
}

//==============================================================================
void DragForce::handleDestructionNotification(
    const dart::common::Subject* subject)
{
  // The body is mid-destruction and auto-detaches itself as a Subject, so we
  // only drop our own references -- never touch the dying body, and do not
  // call removeSubject() here (mirrors
  // Inspector::handleDestructionNotification).
  if (subject == mDragBody) {
    mDragging = false;
    mDragBody = nullptr;
  }
  if (subject == mGizmoBody) {
    // The gizmo's InteractiveFrame is a separate frame in the (still-alive)
    // world, so tear it down here rather than leaving it dangling on screen.
    if (mGizmoDnd && mViewer) {
      mViewer->disableDragAndDrop(mGizmoDnd);
      mGizmoDnd = nullptr;
    }
    if (mGizmoFrame && mWorld)
      mWorld->removeSimpleFrame(mGizmoFrame);
    mGizmoFrame.reset();
    mGizmoBody = nullptr;
  }
  hideLineVisual();
}

//==============================================================================
void DragForce::renderToolbarStatus(double /*guiScale*/) const
{
  if (mStatusBodyName.empty()) {
    ImGui::TextDisabled("Drag: (none)");
  } else {
    ImGui::Text(
        "Drag: %s |F|=%.1f N", mStatusBodyName.c_str(), mStatusForceMag);
  }
}

//==============================================================================
void DragForce::renderTunables(double guiScale)
{
  ImGui::SetNextItemWidth(180.0f * static_cast<float>(guiScale));
  if (ImGui::SliderFloat(
          "Drag coeff",
          &mCoeff,
          1.0f,
          2000.0f,
          "%.0f",
          ImGuiSliderFlags_AlwaysClamp)
      && std::isfinite(mCoeff))
    mCoeff = std::clamp(mCoeff, 1.0f, 2000.0f);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(180.0f * static_cast<float>(guiScale));
  if (ImGui::SliderFloat(
          "Max force",
          &mMaxForce,
          1.0f,
          5000.0f,
          "%.0f N",
          ImGuiSliderFlags_AlwaysClamp)
      && std::isfinite(mMaxForce))
    mMaxForce = std::clamp(mMaxForce, 1.0f, 5000.0f);
  ImGui::TextWrapped(
      "Ctrl+left-drag any body to pull it with a clamped spring force. "
      "Plain left-click selects a body in the Inspector.");
}

} // namespace dart_demos
