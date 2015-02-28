/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef OSGDART_DEFAULTEVENTHANDLER_H
#define OSGDART_DEFAULTEVENTHANDLER_H

#include <vector>
#include <array>

#include <Eigen/Core>

#include <osgGA/GUIEventHandler>

namespace dart {
namespace dynamics {
class Entity;
class Shape;
} // dynamics
} // dart

namespace osgDart
{

struct PickInfo
{
  dart::dynamics::Entity* entity;
  dart::dynamics::Shape* shape;
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

  NUM_MOUSE_BUTTON_EVENTS
};

class DefaultEventHandler : public osgGA::GUIEventHandler
{
public:

  /// Constructor takes in a pointer to a viewer
  explicit DefaultEventHandler(Viewer* _viewer);

  /// Destructor
  virtual ~DefaultEventHandler();

  /// Handle incoming user input
  virtual bool handle(const osgGA::GUIEventAdapter& ea,
                      osgGA::GUIActionAdapter&);

  /// Detect picks
  /// TODO(MXG): Consider putting this functionality in a more accessible place
  void pick(std::vector<PickInfo>& infoVector,
            const osgGA::GUIEventAdapter& ea);

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

protected:

  void eventPick(const osgGA::GUIEventAdapter& ea);

  /// osgDart::Viewer that this event handler is tied to
  Viewer* mViewer;

  /// The objects that were under the cursor during the last button event
  std::vector<PickInfo> mButtonPicks[NUM_MOUSE_BUTTONS][NUM_MOUSE_BUTTON_EVENTS];

  /// Suppress pick detection
  bool mSuppressButtonPicks[NUM_MOUSE_BUTTONS][NUM_MOUSE_BUTTON_EVENTS];

  /// The objects that were under the cursor during the last move
  std::vector<PickInfo> mMovePicks;

  /// Suppress pick detection for moves
  bool mSuppressMovePicks;

  /// Cache for pick data
  std::vector<PickInfo> mTempPicks;

};

} // namespace osgDart

#endif // OSGDART_DEFAULTEVENTHANDLER_H
