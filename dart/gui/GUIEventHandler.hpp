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

#ifndef DART_GUI_GUIEVENTHANDLER_HPP_
#define DART_GUI_GUIEVENTHANDLER_HPP_

#include <dart/gui/Export.hpp>

namespace dart {
namespace gui {
class Viewer;

/// Input positions are logical window coordinates with the origin at top left.
class DART_GUI_API GUIEventAdapter
{
public:
  enum EventType
  {
    NONE,
    PUSH,
    RELEASE,
    DRAG,
    MOVE,
    KEYDOWN,
    KEYUP,
    SCROLL,
    FRAME,
    CLOSE_WINDOW,
    QUIT_APPLICATION
  };
  enum MouseButtonMask
  {
    LEFT_MOUSE_BUTTON = 1,
    MIDDLE_MOUSE_BUTTON = 2,
    RIGHT_MOUSE_BUTTON = 4
  };
  enum ScrollingMotion
  {
    SCROLL_NONE,
    SCROLL_LEFT,
    SCROLL_RIGHT,
    SCROLL_UP,
    SCROLL_DOWN,
    SCROLL_2D
  };
  enum ModKeyMask
  {
    MODKEY_LEFT_SHIFT = 1,
    MODKEY_RIGHT_SHIFT = 2,
    MODKEY_LEFT_CTRL = 4,
    MODKEY_RIGHT_CTRL = 8,
    MODKEY_LEFT_ALT = 16,
    MODKEY_RIGHT_ALT = 32,
    MODKEY_SHIFT = 3,
    MODKEY_CTRL = 12,
    MODKEY_ALT = 48,
    MODKEY_SUPER = 768
  };
  enum KeySymbol
  {
    KEY_BackSpace = 0xFF08,
    KEY_Tab = 0xFF09,
    KEY_Return = 0xFF0D,
    KEY_Escape = 0xFF1B,
    KEY_Home = 0xFF50,
    KEY_Left = 0xFF51,
    KEY_Up = 0xFF52,
    KEY_Right = 0xFF53,
    KEY_Down = 0xFF54,
    KEY_Page_Up = 0xFF55,
    KEY_Page_Down = 0xFF56,
    KEY_End = 0xFF57,
    KEY_Insert = 0xFF63,
    KEY_F1 = 0xFFBE,
    KEY_F2,
    KEY_F3,
    KEY_F4,
    KEY_F5,
    KEY_F6,
    KEY_F7,
    KEY_F8,
    KEY_F9,
    KEY_F10,
    KEY_F11,
    KEY_F12,
    KEY_Shift_L = 0xFFE1,
    KEY_Shift_R,
    KEY_Control_L,
    KEY_Control_R,
    KEY_Alt_L = 0xFFE9,
    KEY_Alt_R,
    KEY_Delete = 0xFFFF
  };

  explicit GUIEventAdapter(
      EventType type = NONE,
      int key = 0,
      double x = 0,
      double y = 0,
      int buttons = 0,
      int modifiers = 0,
      double scrollX = 0,
      double scrollY = 0)
    : mType(type),
      mKey(key),
      mX(x),
      mY(y),
      mButtons(buttons),
      mModifiers(modifiers),
      mScrollX(scrollX),
      mScrollY(scrollY)
  {
  }
  EventType getEventType() const
  {
    return mType;
  }
  int getKey() const
  {
    return mKey;
  }
  double getX() const
  {
    return mX;
  }
  double getY() const
  {
    return mY;
  }
  int getButtonMask() const
  {
    return mButtons;
  }
  int getModKeyMask() const
  {
    return mModifiers;
  }
  double getScrollingDeltaX() const
  {
    return mScrollX;
  }
  double getScrollingDeltaY() const
  {
    return mScrollY;
  }
  ScrollingMotion getScrollingMotion() const
  {
    if (mScrollX && mScrollY)
      return SCROLL_2D;
    if (mScrollY)
      return mScrollY > 0 ? SCROLL_UP : SCROLL_DOWN;
    if (mScrollX)
      return mScrollX > 0 ? SCROLL_RIGHT : SCROLL_LEFT;
    return SCROLL_NONE;
  }

private:
  EventType mType;
  int mKey;
  double mX, mY;
  int mButtons, mModifiers;
  double mScrollX, mScrollY;
};

class DART_GUI_API GUIActionAdapter
{
public:
  explicit GUIActionAdapter(Viewer& viewer) : mViewer(viewer) {}
  Viewer& getViewer() const
  {
    return mViewer;
  }

private:
  Viewer& mViewer;
};

class DART_GUI_API GUIEventHandler
{
public:
  virtual ~GUIEventHandler();
  virtual bool handle(const GUIEventAdapter& event, GUIActionAdapter& viewer)
      = 0;
};
} // namespace gui
} // namespace dart
#endif
