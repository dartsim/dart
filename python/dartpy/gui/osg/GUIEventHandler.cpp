/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

namespace dart {
namespace python {

class GUIEventHandlerNoRef : public osgGA::GUIEventHandler
{
public:
  using GUIEventHandler::handle;

  virtual bool handle(
      const osgGA::GUIEventAdapter* /*ea*/, osgGA::GUIActionAdapter* /*aa*/)
  {
    return true;
  }

protected:
  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
  {
    return handle(&ea, &aa);
  }
};

class PyGUIEventHandler final : public GUIEventHandlerNoRef
{
public:
  // Inherit the constructors
  using GUIEventHandlerNoRef::GUIEventHandler;

  // Trampoline for virtual function
  bool handle(
      const osgGA::GUIEventAdapter* ea, osgGA::GUIActionAdapter* aa) override
  {
    PYBIND11_OVERLOAD(
        bool,                 // Return type
        GUIEventHandlerNoRef, // Parent class
        handle, // Name of function in C++ (must match Python name)
        ea,
        aa);
  }
};

void GUIEventHandler(py::module& m)
{
  auto ea = ::py::class_<
                osgGA::GUIEventAdapter,
                ::osg::ref_ptr<osgGA::GUIEventAdapter>>(m, "GUIEventAdapter")
                .def(py::init<>())
                .def(
                    "getEventType",
                    +[](const osgGA::GUIEventAdapter* self)
                        -> osgGA::GUIEventAdapter::EventType {
                      return self->getEventType();
                    })
                .def(
                    "getKey", +[](const osgGA::GUIEventAdapter* self) -> int {
                      return self->getKey();
                    });

#define DARTPY_DEFINE_ENUM_MOUSE_BUTTON_MASK(val)                              \
  .value(#val, osgGA::GUIEventAdapter::MouseButtonMask::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::MouseButtonMask>(ea, "MouseButtonMask")
      DARTPY_DEFINE_ENUM_MOUSE_BUTTON_MASK(LEFT_MOUSE_BUTTON)
      DARTPY_DEFINE_ENUM_MOUSE_BUTTON_MASK(MIDDLE_MOUSE_BUTTON)
      DARTPY_DEFINE_ENUM_MOUSE_BUTTON_MASK(RIGHT_MOUSE_BUTTON)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_EVENT_TYPE(val)                                     \
  .value(#val, osgGA::GUIEventAdapter::EventType::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::EventType>(ea, "EventType")
      DARTPY_DEFINE_ENUM_EVENT_TYPE(NONE)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(PUSH)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(RELEASE)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(DOUBLECLICK)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(DRAG)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(MOVE)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(KEYDOWN)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(KEYUP)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(FRAME)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(RESIZE)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(SCROLL)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(PEN_PRESSURE)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(PEN_ORIENTATION)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(PEN_PROXIMITY_ENTER)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(PEN_PROXIMITY_LEAVE)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(CLOSE_WINDOW)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(QUIT_APPLICATION)
      DARTPY_DEFINE_ENUM_EVENT_TYPE(USER)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_KEY_SYMBOL(val)                                     \
  .value(#val, osgGA::GUIEventAdapter::KeySymbol::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::KeySymbol>(ea, "KeySymbol")
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Space)

      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_0)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_1)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_2)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_3)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_4)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_5)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_6)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_7)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_8)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_9)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_A)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_B)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_C)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_D)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_E)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_G)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_H)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_I)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_J)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_K)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_M)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_N)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_O)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_P)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Q)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_R)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_S)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_T)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_U)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_V)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_W)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_X)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Y)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Z)

      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Exclaim     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Quotedbl    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Hash        )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Dollar      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Ampersand   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Quote       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Leftparen   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Rightparen  )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Asterisk    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Plus        )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Comma       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Minus       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Period      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Slash       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Colon       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Semicolon   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Less        )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Equals      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Greater     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Question    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_At          )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Leftbracket )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Backslash   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Rightbracket)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Caret       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Underscore  )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Backquote   )

      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_BackSpace   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Tab         )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Linefeed    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Clear       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Return      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Pause       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Scroll_Lock )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Sys_Req     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Escape      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Delete      )

      /* Cursor control & motion */
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Home     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Left     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Up       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Right    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Down     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Prior    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Page_Up  )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Next     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Page_Down)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_End      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Begin    )

      /* Misc Functions */
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Select       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Print        )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Execute      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Insert       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Undo         )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Redo         )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Menu         )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Find         )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Cancel       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Help         )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Break        )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Mode_switch  )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Script_switch)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Num_Lock     )

      /* Keypad Functions, keypad numbers cleverly chosen to map to ascii */
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Space    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Tab      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Enter    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_F1       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_F2       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_F3       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_F4       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Home     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Left     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Up       )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Right    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Down     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Prior    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Page_Up  )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Next     )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Page_Down)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_End      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Begin    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Insert   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Delete   )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Equal    )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Multiply )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Add      )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Separator)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Subtract )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Decimal  )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_Divide   )

      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_0)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_1)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_2)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_3)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_4)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_5)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_6)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_7)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_8)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_KP_9)

      /*
       * Auxiliary Functions; note the duplicate definitions for left and right
       * function keys;  Sun keyboards and a few other manufactures have such
       * function key groups on the left and/or right sides of the keyboard.
       * We've not found a keyboard with more than 35 function keys total.
       */
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F1 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F2 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F3 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F4 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F5 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F6 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F7 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F8 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F9 )
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F10)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F11)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F12)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F13)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F14)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F15)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F16)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F17)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F18)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F19)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F20)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F21)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F22)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F23)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F24)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F25)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F26)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F27)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F28)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F29)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F30)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F31)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F32)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F33)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F34)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_F35)

      /* Modifiers */
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Shift_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Shift_R)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Control_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Control_R)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Caps_Lock)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Shift_Lock)

      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Meta_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Meta_R)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Alt_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Alt_R)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Super_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Super_R)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Hyper_L)
      DARTPY_DEFINE_ENUM_KEY_SYMBOL(KEY_Hyper_R)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_MOD_KEY_MASK(val)                                   \
  .value(#val, osgGA::GUIEventAdapter::ModKeyMask::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::ModKeyMask>(ea, "ModKeyMask")
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_LEFT_SHIFT)
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_RIGHT_SHIFT)
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_LEFT_CTRL  )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_RIGHT_CTRL )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_LEFT_ALT   )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_RIGHT_ALT  )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_LEFT_META  )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_RIGHT_META )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_LEFT_SUPER )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_RIGHT_SUPER)
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_LEFT_HYPER )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_RIGHT_HYPER)
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_NUM_LOCK   )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_CAPS_LOCK  )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_CTRL       )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_SHIFT      )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_ALT        )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_META       )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_SUPER      )
      DARTPY_DEFINE_ENUM_MOD_KEY_MASK(MODKEY_HYPER      )
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_MOUSE_Y_ORIENTATION(val)                            \
  .value(#val, osgGA::GUIEventAdapter::MouseYOrientation::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::MouseYOrientation>(
      ea, "MouseYOrientation")
      DARTPY_DEFINE_ENUM_MOUSE_Y_ORIENTATION(Y_INCREASING_UPWARDS)
      DARTPY_DEFINE_ENUM_MOUSE_Y_ORIENTATION(Y_INCREASING_DOWNWARDS)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_SCROLLING_MOTION(val)                               \
  .value(#val, osgGA::GUIEventAdapter::ScrollingMotion::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::ScrollingMotion>(ea, "ScrollingMotion")
      DARTPY_DEFINE_ENUM_SCROLLING_MOTION(SCROLL_NONE)
      DARTPY_DEFINE_ENUM_SCROLLING_MOTION(SCROLL_LEFT)
      DARTPY_DEFINE_ENUM_SCROLLING_MOTION(SCROLL_RIGHT)
      DARTPY_DEFINE_ENUM_SCROLLING_MOTION(SCROLL_UP)
      DARTPY_DEFINE_ENUM_SCROLLING_MOTION(SCROLL_DOWN)
      DARTPY_DEFINE_ENUM_SCROLLING_MOTION(SCROLL_2D)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_TABLET_POINTER_TYPE(val)                            \
  .value(#val, osgGA::GUIEventAdapter::TabletPointerType::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::TabletPointerType>(
      ea, "TabletPointerType") DARTPY_DEFINE_ENUM_TABLET_POINTER_TYPE(UNKNOWN)
      DARTPY_DEFINE_ENUM_TABLET_POINTER_TYPE(PEN)
      DARTPY_DEFINE_ENUM_TABLET_POINTER_TYPE(PUCK)
      DARTPY_DEFINE_ENUM_TABLET_POINTER_TYPE(ERASER)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_ENUM_TOUCH_PHASE(val)                                    \
  .value(#val, osgGA::GUIEventAdapter::TouchPhase::val)

  // clang-format off
  ::py::enum_<osgGA::GUIEventAdapter::TouchPhase>(ea, "TouchPhase")
      DARTPY_DEFINE_ENUM_TOUCH_PHASE(TOUCH_UNKNOWN)
      DARTPY_DEFINE_ENUM_TOUCH_PHASE(TOUCH_BEGAN)
      DARTPY_DEFINE_ENUM_TOUCH_PHASE(TOUCH_MOVED)
      DARTPY_DEFINE_ENUM_TOUCH_PHASE(TOUCH_STATIONERY)
      DARTPY_DEFINE_ENUM_TOUCH_PHASE(TOUCH_ENDED)
      .export_values();
  // clang-format on

  ::py::class_<osgGA::GUIActionAdapter>(m, "GUIActionAdapter");

  ::py::class_<osgGA::GUIEventHandler, ::osg::ref_ptr<osgGA::GUIEventHandler>>(
      m, "__GUIEventHandler__")
      .def(py::init<>());

  ::py::class_<
      GUIEventHandlerNoRef,
      osgGA::GUIEventHandler,
      PyGUIEventHandler,
      ::osg::ref_ptr<GUIEventHandlerNoRef>>(m, "GUIEventHandler")
      .def(py::init<>());
}

} // namespace python
} // namespace dart
