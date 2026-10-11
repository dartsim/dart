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

#include "dart/gui/DefaultEventHandler.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Entity.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/SmartPointer.hpp"
#include "dart/gui/InteractiveFrame.hpp"
#include "dart/gui/MouseEventHandler.hpp"
#include "dart/gui/Viewer.hpp"
#include "dart/simulation/World.hpp"

#include <iostream>
#include <map>
#include <stdexcept>

#include <cmath>

namespace dart {
namespace gui {

namespace {

using MouseEventHandlerRegistry
    = std::map<const dart::common::Subject*, MouseEventHandler*>;

MouseEventHandlerRegistry& getMouseEventHandlerRegistry()
{
  static MouseEventHandlerRegistry registry;
  return registry;
}

const dart::common::Subject* getSubject(MouseEventHandler* handler)
{
  return static_cast<const dart::common::Subject*>(handler);
}

std::shared_ptr<dynamics::ShapeFrame> findFrameOwner(
    const std::shared_ptr<dynamics::SimpleFrame>& root,
    dynamics::ShapeFrame* target)
{
  if (root.get() == target)
    return root;

  std::shared_ptr<dynamics::ShapeFrame> child;
  if (const auto* frame = dynamic_cast<const InteractiveFrame*>(root.get())) {
    for (const auto& shapeFrame : frame->getShapeFrameOwners())
      if (shapeFrame.get() == target)
        child = shapeFrame;
    for (int type = 0; type < InteractiveTool::NUM_TYPES && !child; ++type)
      for (std::size_t axis = 0; axis < 3 && !child; ++axis)
        child = findFrameOwner(
            frame->getToolShared(
                static_cast<InteractiveTool::Type>(type), axis),
            target);
  } else if (
      const auto* tool = dynamic_cast<const InteractiveTool*>(root.get())) {
    for (const auto& shapeFrame : tool->getShapeFrameOwners())
      if (shapeFrame.get() == target)
        child = shapeFrame;
  }

  if (!child)
    return nullptr;
  // Child transforms refer to a raw parent. Retain both exact child storage
  // and its parent until removal detaches the child to Frame::World().
  using Owners = std::pair<
      std::shared_ptr<dynamics::ShapeFrame>,
      std::shared_ptr<dynamics::SimpleFrame>>;
  return std::shared_ptr<dynamics::ShapeFrame>(
      std::make_shared<Owners>(child, root), child.get());
}

} // namespace

PickInfo::PickInfo() = default;

PickInfo::PickInfo(
    dynamics::ShapeFrame* pickedFrame,
    std::shared_ptr<dynamics::Shape> pickedShape,
    const Eigen::Vector3d& pickedPosition,
    const Eigen::Vector3d& pickedNormal)
  : frame(pickedFrame),
    shape(std::move(pickedShape)),
    position(pickedPosition),
    normal(pickedNormal)
{
}

void PickInfo::retainFrameOwner(
    const std::vector<std::shared_ptr<simulation::World>>& worlds)
{
  if (getFrameOwner())
    return;
  mFrameOwner.reset();
  if (!frame)
    return;
  if (auto* node = frame->asShapeNode()) {
    auto owner = std::make_shared<dynamics::ShapeNodePtr>(node);
    mFrameOwner = std::shared_ptr<dynamics::ShapeFrame>(owner, node);
    return;
  }
  for (const auto& world : worlds) {
    if (!world)
      continue;
    for (std::size_t i = 0; i < world->getNumSimpleFrames(); ++i) {
      mFrameOwner = findFrameOwner(world->getSimpleFrame(i), frame.get());
      if (mFrameOwner)
        return;
    }
  }
}

void PickInfo::setFrameOwner(std::shared_ptr<dynamics::ShapeFrame> owner)
{
  if (owner && owner->asShapeNode()) {
    auto node = std::make_shared<dynamics::ShapeNodePtr>(owner->asShapeNode());
    owner = std::shared_ptr<dynamics::ShapeFrame>(node, node->get());
  }
  frame = owner.get();
  mFrameOwner = std::move(owner);
}

std::shared_ptr<dynamics::ShapeFrame> PickInfo::getFrameOwner() const
{
  return frame.get() == mFrameOwner.get() ? mFrameOwner : nullptr;
}

DefaultEventHandler::DefaultEventHandler(Viewer* _viewer)
  : mViewer(_viewer),
    mLastCursorPosition(Eigen::Vector2d::Zero()),
    mLastModKeyMask(0)
{
  if (!mViewer)
    throw std::invalid_argument("DefaultEventHandler requires a viewer");
  mViewer->addInstructionText("Spacebar:     Turn simulation on/off\n");
  mViewer->addInstructionText("Ctrl+H:       Turn headlights on/off\n");

  for (std::size_t i = 0; i < NUM_MOUSE_BUTTONS; ++i)
    for (std::size_t j = 0; j < BUTTON_NOTHING; ++j)
      mSuppressButtonPicks[i][j] = false;
  mSuppressMovePicks = false;

  clearButtonEvents();
}

//==============================================================================
DefaultEventHandler::~DefaultEventHandler()
{
  MouseEventHandlerRegistry& registry = getMouseEventHandlerRegistry();
  for (MouseEventHandler* handler : mMouseEventHandlers)
    registry.erase(getSubject(handler));
}

//==============================================================================
MouseButtonEvent DefaultEventHandler::getButtonEvent(MouseButton button) const
{
  if (button < 0 || button >= NUM_MOUSE_BUTTONS)
    throw std::invalid_argument("Mouse button is out of range");
  return mLastButtonEvent[button];
}

//==============================================================================
int DefaultEventHandler::getModKeyMask() const
{
  return mLastModKeyMask;
}

//==============================================================================
double DefaultEventHandler::getWindowCursorX() const
{
  return mLastCursorPosition[0];
}

//==============================================================================
double DefaultEventHandler::getWindowCursorY() const
{
  return mLastCursorPosition[1];
}

//==============================================================================
Eigen::Vector3d DefaultEventHandler::getDeltaCursor(
    const Eigen::Vector3d& _fromPosition,
    ConstraintType _constraint,
    const Eigen::Vector3d& _constraintVector) const
{
  Eigen::Vector3d nearPoint, farPoint;
  getNearAndFarPointUnderCursor(nearPoint, farPoint);
  const Eigen::Vector3d direction = farPoint - nearPoint;
  if (_constraint == LINE_CONSTRAINT) {
    const double denominator
        = direction.squaredNorm() * _constraintVector.squaredNorm()
          - std::pow(direction.dot(_constraintVector), 2);
    if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12)
      return Eigen::Vector3d::Zero();
    const Eigen::Vector3d offset = nearPoint - _fromPosition;
    const double parameter
        = (direction.squaredNorm() * _constraintVector.dot(offset)
           - direction.dot(_constraintVector) * direction.dot(offset))
          / denominator;
    return _constraintVector * parameter;
  }
  const Eigen::Vector3d normal = _constraint == PLANE_CONSTRAINT
                                     ? _constraintVector
                                     : mViewer->getCameraDirection();
  const double denominator = normal.dot(direction);
  if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12)
    return Eigen::Vector3d::Zero();
  return nearPoint - _fromPosition
         + direction * (normal.dot(_fromPosition - nearPoint) / denominator);
}

//==============================================================================
void DefaultEventHandler::getNearAndFarPointUnderCursor(
    Eigen::Vector3d& nearPt, Eigen::Vector3d& farPt, double distance) const
{
  if (!std::isfinite(distance) || distance < 0 || distance > 1)
    throw std::invalid_argument("Window depth must be finite and in [0, 1]");
  mViewer->getRayUnderCursor(
      getWindowCursorX(), getWindowCursorY(), nearPt, farPt);
  if (distance != 1)
    farPt = nearPt
            + mViewer->getWindowDepthRayParameter(distance) * (farPt - nearPt);
}

//==============================================================================
const std::vector<PickInfo>& DefaultEventHandler::getButtonPicks(
    MouseButton button, MouseButtonEvent event) const
{
  if (button < 0 || button >= NUM_MOUSE_BUTTONS || event < 0
      || event > BUTTON_NOTHING)
    throw std::invalid_argument("Mouse button or event is out of range");
  if (BUTTON_NOTHING == event)
    return mMovePicks;

  return mButtonPicks[button][event];
}

//==============================================================================
const std::vector<PickInfo>& DefaultEventHandler::getMovePicks() const
{
  return mMovePicks;
}

//==============================================================================
void DefaultEventHandler::suppressButtonPicks(
    MouseButton button, MouseButtonEvent event)
{
  if (button < 0 || button >= NUM_MOUSE_BUTTONS || event < 0
      || event > BUTTON_NOTHING)
    throw std::invalid_argument("Mouse button or event is out of range");
  if (BUTTON_NOTHING == event)
    mSuppressMovePicks = true;
  else
    mSuppressButtonPicks[button][event] = true;
}

//==============================================================================
void DefaultEventHandler::suppressMovePicks()
{
  mSuppressMovePicks = true;
}

//==============================================================================
void DefaultEventHandler::activateButtonPicks(
    MouseButton button, MouseButtonEvent event)
{
  if (button < 0 || button >= NUM_MOUSE_BUTTONS || event < 0
      || event > BUTTON_NOTHING)
    throw std::invalid_argument("Mouse button or event is out of range");
  if (BUTTON_NOTHING == event)
    mSuppressMovePicks = false;
  else
    mSuppressButtonPicks[button][event] = false;
}

//==============================================================================
void DefaultEventHandler::activateMovePicks()
{
  mSuppressMovePicks = false;
}

//==============================================================================
void DefaultEventHandler::pick(
    std::vector<PickInfo>& infoVector, const GUIEventAdapter& ea)
{
  mViewer->computeIntersections(ea, infoVector);
}

//==============================================================================
void DefaultEventHandler::addMouseEventHandler(MouseEventHandler* handler)
{
  mMouseEventHandlers.insert(handler);
  handler->mEventHandler = this;
  handler->addSubject(this);
  // Observe the handler so handleDestructionNotification() erases it from
  // mMouseEventHandlers when it is destroyed. Without this the set keeps a
  // dangling pointer after a registered handler dies (the header documents
  // automatic removal on deletion), and the next mouse event dereferences it.
  addSubject(handler);
  getMouseEventHandlerRegistry()[getSubject(handler)] = handler;
}

//==============================================================================
const std::set<MouseEventHandler*>& DefaultEventHandler::getMouseEventHandlers()
    const
{
  return mMouseEventHandlers;
}

//==============================================================================
static bool wasActive(MouseButtonEvent event)
{
  return ((event == BUTTON_PUSH) || (event == BUTTON_DRAG));
}

//==============================================================================
static void assignEventToButtons(
    MouseButtonEvent (&mLastButtonEvent)[NUM_MOUSE_BUTTONS],
    const GUIEventAdapter& ea)
{
  MouseButtonEvent event = BUTTON_NOTHING;
  if (ea.getEventType() == GUIEventAdapter::PUSH)
    event = BUTTON_PUSH;
  else if (ea.getEventType() == GUIEventAdapter::DRAG)
    event = BUTTON_DRAG;
  else if (ea.getEventType() == GUIEventAdapter::RELEASE)
    event = BUTTON_RELEASE;

  if (BUTTON_RELEASE == event) {
    if ((ea.getButtonMask() & GUIEventAdapter::LEFT_MOUSE_BUTTON) == 0
        && wasActive(mLastButtonEvent[LEFT_MOUSE]))
      mLastButtonEvent[LEFT_MOUSE] = event;

    if ((ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON) == 0
        && wasActive(mLastButtonEvent[RIGHT_MOUSE]))
      mLastButtonEvent[RIGHT_MOUSE] = event;

    if ((ea.getButtonMask() & GUIEventAdapter::MIDDLE_MOUSE_BUTTON) == 0
        && wasActive(mLastButtonEvent[MIDDLE_MOUSE]))
      mLastButtonEvent[MIDDLE_MOUSE] = event;
  } else {
    if (ea.getButtonMask() & GUIEventAdapter::LEFT_MOUSE_BUTTON)
      mLastButtonEvent[LEFT_MOUSE] = event;

    if (ea.getButtonMask() & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
      mLastButtonEvent[RIGHT_MOUSE] = event;

    if (ea.getButtonMask() & GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
      mLastButtonEvent[MIDDLE_MOUSE] = event;
  }
}

//==============================================================================
bool DefaultEventHandler::handle(const GUIEventAdapter& ea, GUIActionAdapter&)
{
  if (ea.getEventType() != GUIEventAdapter::FRAME)
    mLastModKeyMask = ea.getModKeyMask();

  switch (ea.getEventType()) {
    case GUIEventAdapter::PUSH:
    case GUIEventAdapter::DRAG:
    case GUIEventAdapter::RELEASE:
    case GUIEventAdapter::MOVE:
      mLastCursorPosition[0] = ea.getX();
      mLastCursorPosition[1] = ea.getY();

      break;

    default:
      break;
  }

  switch (ea.getEventType()) {
    case GUIEventAdapter::KEYDOWN: {
      switch (ea.getKey()) {
        case 'h':
          if (!(ea.getModKeyMask() & GUIEventAdapter::MODKEY_CTRL))
            break;
          [[fallthrough]];
        case 8: // ctrl+h
        {
          mViewer->switchHeadlights(!mViewer->checkHeadlights());
          return true;
        }

        case ' ': {
          if (mViewer->isAllowingSimulation()) {
            mViewer->simulate(!mViewer->isSimulating());
            return true;
          }
          break;
        }
      }
      break;
    }

    case GUIEventAdapter::MOVE: {
      if (!mSuppressMovePicks)
        pick(mMovePicks, ea);

      triggerMouseEventHandlers();
      break;
    }

    case GUIEventAdapter::PUSH:
    case GUIEventAdapter::DRAG:
    case GUIEventAdapter::RELEASE:

      eventPick(ea);
      assignEventToButtons(mLastButtonEvent, ea);

      mViewer->updateDragAndDrops();

      triggerMouseEventHandlers();
      break;

    default:
      break;
  }

  return false;
}

//==============================================================================
void DefaultEventHandler::triggerMouseEventHandlers()
{
  const std::vector<MouseEventHandler*> handlers(
      mMouseEventHandlers.begin(), mMouseEventHandlers.end());

  for (MouseEventHandler* handler : handlers) {
    if (mMouseEventHandlers.find(handler) == mMouseEventHandlers.end())
      continue;

    handler->update();
  }
}

//==============================================================================
void DefaultEventHandler::eventPick(const GUIEventAdapter& ea)
{
  MouseButtonEvent mbe;
  switch (ea.getEventType()) {
    case GUIEventAdapter::PUSH:
      mbe = BUTTON_PUSH;
      break;
    case GUIEventAdapter::DRAG:
      mbe = BUTTON_DRAG;
      break;
    case GUIEventAdapter::RELEASE:
      mbe = BUTTON_RELEASE;
      break;
    default:
      return;
  }

  // Release masks omit released buttons; inspect their previous active state.
  const int buttonMask
      = mbe == BUTTON_RELEASE ? ~ea.getButtonMask() : ea.getButtonMask();
  const bool pickLeft
      = (buttonMask & GUIEventAdapter::LEFT_MOUSE_BUTTON)
        && !mSuppressButtonPicks[LEFT_MOUSE][mbe]
        && (mbe != BUTTON_RELEASE || wasActive(mLastButtonEvent[LEFT_MOUSE]));
  const bool pickRight
      = (buttonMask & GUIEventAdapter::RIGHT_MOUSE_BUTTON)
        && !mSuppressButtonPicks[RIGHT_MOUSE][mbe]
        && (mbe != BUTTON_RELEASE || wasActive(mLastButtonEvent[RIGHT_MOUSE]));
  const bool pickMiddle
      = (buttonMask & GUIEventAdapter::MIDDLE_MOUSE_BUTTON)
        && !mSuppressButtonPicks[MIDDLE_MOUSE][mbe]
        && (mbe != BUTTON_RELEASE || wasActive(mLastButtonEvent[MIDDLE_MOUSE]));
  if (pickLeft || pickRight || pickMiddle) {
    pick(mTempPicks, ea);

    if (pickLeft)
      mButtonPicks[LEFT_MOUSE][mbe] = mTempPicks;

    if (pickRight)
      mButtonPicks[RIGHT_MOUSE][mbe] = mTempPicks;

    if (pickMiddle)
      mButtonPicks[MIDDLE_MOUSE][mbe] = mTempPicks;
  }
}

//==============================================================================
void DefaultEventHandler::clearButtonEvents()
{
  for (std::size_t i = 0; i < NUM_MOUSE_BUTTONS; ++i)
    mLastButtonEvent[i] = BUTTON_NOTHING;
}

//==============================================================================
void DefaultEventHandler::handleDestructionNotification(
    const dart::common::Subject* _subject)
{
  MouseEventHandlerRegistry& registry = getMouseEventHandlerRegistry();
  const auto registryIt = registry.find(_subject);
  if (registryIt == registry.end())
    return;

  mMouseEventHandlers.erase(registryIt->second);
  registry.erase(registryIt);
}

} // namespace gui
} // namespace dart
