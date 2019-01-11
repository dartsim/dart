/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_GUI_OSG_DEFAULTEVENTHANDLER_HPP_
#define DART_GUI_OSG_DEFAULTEVENTHANDLER_HPP_

#include <vector>
#include <array>
#include <memory>

#include <Eigen/Core>

#include <osgGA/GUIEventHandler>

#include "dart/common/Subject.hpp"
#include "dart/common/Observer.hpp"

namespace dart {

namespace dynamics {
class Shape;
class ShapeFrame;
class Entity;
} // dynamics

namespace gui {
namespace osg {

struct PickInfo
{
  dart::dynamics::ShapeFrame* frame;
  std::shared_ptr<dart::dynamics::Shape> shape;
  Eigen::Vector3d position;
  Eigen::Vector3d normal;
};

class Viewer;

enum MouseButton {

  LEFT_MOUSE = 0,
  RIGHT_MOUSE,
  MIDDLE_MOUSE,

  NUM_MOUSE_BUTTONS
};

enum MouseButtonEvent {

  BUTTON_PUSH = 0,
  BUTTON_DRAG,
  BUTTON_RELEASE,
  BUTTON_NOTHING

};

enum ConstraintType {

  UNCONSTRAINED = 0,
  LINE_CONSTRAINT,
  PLANE_CONSTRAINT,
  CUSTOM_CONSTRAINT,

  NUM_CONSTRAINT_TYPES
};

class MouseEventHandler;

class DefaultEventHandler : public ::osgGA::GUIEventHandler,
                            public virtual dart::common::Subject,
                            public virtual dart::common::Observer
{
public:

  /// Constructor takes in a pointer to a viewer
  explicit DefaultEventHandler(Viewer* _viewer);

  /// Destructor
  virtual ~DefaultEventHandler();

  /// Returns the last event performed by a mouse button
  MouseButtonEvent getButtonEvent(MouseButton button) const;

  /// Returns the last modkey mask
  int getModKeyMask() const;

  /// Get the last x value of the cursor in Window coordinates
  double getWindowCursorX() const;

  /// Get the last y value of the cursor in Window coordinates
  double getWindowCursorY() const;

  /// Get the change change in the cursor position with respect to some previous
  /// location in the world (_fromPosition). For the unconstrained case, this
  /// change is projected onto a plane parallel to the camera's orientation.
  ///
  /// If _constraint is set to LINE_CONSTRAINT, the change in cursor position
  /// will be constrained to a line that passes through _fromPosition with a
  /// slope of _constraintVector.
  ///
  /// If _constraint is set to PLANE_CONSTRAINT, the change in cursor position
  /// will be constrained to a plane that passes through _fromPosition with a
  /// normal vector of _constraintVector.
  Eigen::Vector3d getDeltaCursor(const Eigen::Vector3d& _fromPosition,
                                 ConstraintType _constraint=UNCONSTRAINED,
                                 const Eigen::Vector3d& _constraintVector =
                                                Eigen::Vector3d::UnitZ()) const;

  /// Get two points that are under the current cursor position. The near point
  /// will be inside the plane of the camera. The far point will have the given
  /// distance from the plane of the camera (default is 1.0).
  void getNearAndFarPointUnderCursor(Eigen::Vector3d& near,
                                     Eigen::Vector3d& far,
                                     double distance=1.0) const;

  /// Get the most recent picks for the specified button and event type
  const std::vector<PickInfo>& getButtonPicks(MouseButton button,
                                              MouseButtonEvent event) const;

  /// Get the most recent picks for a mouse movement (click-and-drag actions do
  /// not qualify as movements)
  const std::vector<PickInfo>& getMovePicks() const;

  /// Suppress pick detection for the specified button event
  void suppressButtonPicks(MouseButton button, MouseButtonEvent event);

  /// Suppress pick detection for mouse movements
  void suppressMovePicks();

  /// Activate pick detection for the specified button event (on by default)
  void activateButtonPicks(MouseButton button, MouseButtonEvent event);

  /// Activate pick detection for mouse movements (on by default)
  void activateMovePicks();

  /// Detect picks
  /// TODO(MXG): Consider putting this functionality in a more accessible place
  void pick(std::vector<PickInfo>& infoVector,
            const ::osgGA::GUIEventAdapter& ea);

  /// Add a MouseEventHandler that will get invoked whenever a mouse event
  /// occurs. You never need to worry about removing a MouseEventHandler from a
  /// DefaultEventHandler, because it will get removed automatically upon
  /// deletion
  void addMouseEventHandler(MouseEventHandler* handler);

  /// Get the list of MouseEventHandlers that are currently held by this
  /// DefaultEventHandler
  const std::set<MouseEventHandler*>& getMouseEventHandlers() const;

  /// Handle incoming user input
  bool handle(const ::osgGA::GUIEventAdapter& ea,
              ::osgGA::GUIActionAdapter&) override;

protected:

  /// Calls update on all MouseEventHandlers
  void triggerMouseEventHandlers();

  /// Gather current picks and assign them to the latest event
  void eventPick(const ::osgGA::GUIEventAdapter& ea);

  /// Clear out the current button events
  void clearButtonEvents();

  void handleDestructionNotification(
      const dart::common::Subject* _subject) override;

  /// dart::gui::osg::Viewer that this event handler is tied to
  Viewer* mViewer;

  /// Set of MouseEventHandlers that are tied to this DefaultEventHandler
  std::set<MouseEventHandler*> mMouseEventHandlers;

  /// The objects that were under the cursor during the last button event
  std::vector<PickInfo> mButtonPicks[NUM_MOUSE_BUTTONS][BUTTON_NOTHING];

  /// Suppress pick detection
  bool mSuppressButtonPicks[NUM_MOUSE_BUTTONS][BUTTON_NOTHING];

  /// The objects that were under the cursor during the last move
  std::vector<PickInfo> mMovePicks;

  /// Suppress pick detection for moves
  bool mSuppressMovePicks;

  /// Cache for pick data
  std::vector<PickInfo> mTempPicks;

  /// The last mouse event that was registered by the event handler
  MouseButtonEvent mLastButtonEvent[NUM_MOUSE_BUTTONS];

  /// X/Y values of the cursor (in the window coordinates) during the last mouse
  /// event
  Eigen::Vector2d mLastCursorPosition;

  /// Storage for the last modkey mask
  int mLastModKeyMask;

};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_DEFAULTEVENTHANDLER_HPP_
