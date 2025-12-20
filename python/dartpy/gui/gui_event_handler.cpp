#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

class GUIEventHandlerNoRef : public osgGA::GUIEventHandler
{
public:
  using osgGA::GUIEventHandler::GUIEventHandler;
  using osgGA::GUIEventHandler::handle;

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
  NB_TRAMPOLINE(GUIEventHandlerNoRef, 1);

  bool handle(
      const osgGA::GUIEventAdapter* ea, osgGA::GUIActionAdapter* aa) override
  {
    NB_OVERRIDE(handle, ea, aa);
  }
};

} // namespace

void defGuiEventHandler(nb::module_& m)
{
  auto ea = nb::class_<osgGA::GUIEventAdapter>(m, "GUIEventAdapter")
                .def(nb::init<>())
                .def("getEventType", &osgGA::GUIEventAdapter::getEventType)
                .def("getKey", &osgGA::GUIEventAdapter::getKey);

  nb::enum_<osgGA::GUIEventAdapter::MouseButtonMask>(ea, "MouseButtonMask")
      .value(
          "LEFT_MOUSE_BUTTON",
          osgGA::GUIEventAdapter::MouseButtonMask::LEFT_MOUSE_BUTTON)
      .value(
          "MIDDLE_MOUSE_BUTTON",
          osgGA::GUIEventAdapter::MouseButtonMask::MIDDLE_MOUSE_BUTTON)
      .value(
          "RIGHT_MOUSE_BUTTON",
          osgGA::GUIEventAdapter::MouseButtonMask::RIGHT_MOUSE_BUTTON)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::EventType>(ea, "EventType")
      .value("NONE", osgGA::GUIEventAdapter::EventType::NONE)
      .value("PUSH", osgGA::GUIEventAdapter::EventType::PUSH)
      .value("RELEASE", osgGA::GUIEventAdapter::EventType::RELEASE)
      .value("DOUBLECLICK", osgGA::GUIEventAdapter::EventType::DOUBLECLICK)
      .value("DRAG", osgGA::GUIEventAdapter::EventType::DRAG)
      .value("MOVE", osgGA::GUIEventAdapter::EventType::MOVE)
      .value("KEYDOWN", osgGA::GUIEventAdapter::EventType::KEYDOWN)
      .value("KEYUP", osgGA::GUIEventAdapter::EventType::KEYUP)
      .value("FRAME", osgGA::GUIEventAdapter::EventType::FRAME)
      .value("RESIZE", osgGA::GUIEventAdapter::EventType::RESIZE)
      .value("SCROLL", osgGA::GUIEventAdapter::EventType::SCROLL)
      .value("PEN_PRESSURE", osgGA::GUIEventAdapter::EventType::PEN_PRESSURE)
      .value(
          "PEN_ORIENTATION", osgGA::GUIEventAdapter::EventType::PEN_ORIENTATION)
      .value(
          "PEN_PROXIMITY_ENTER",
          osgGA::GUIEventAdapter::EventType::PEN_PROXIMITY_ENTER)
      .value(
          "PEN_PROXIMITY_LEAVE",
          osgGA::GUIEventAdapter::EventType::PEN_PROXIMITY_LEAVE)
      .value("CLOSE_WINDOW", osgGA::GUIEventAdapter::EventType::CLOSE_WINDOW)
      .value(
          "QUIT_APPLICATION",
          osgGA::GUIEventAdapter::EventType::QUIT_APPLICATION)
      .value("USER", osgGA::GUIEventAdapter::EventType::USER)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::KeySymbol>(ea, "KeySymbol")
      .value("KEY_Space", osgGA::GUIEventAdapter::KeySymbol::KEY_Space)
      .value("KEY_0", osgGA::GUIEventAdapter::KeySymbol::KEY_0)
      .value("KEY_1", osgGA::GUIEventAdapter::KeySymbol::KEY_1)
      .value("KEY_2", osgGA::GUIEventAdapter::KeySymbol::KEY_2)
      .value("KEY_3", osgGA::GUIEventAdapter::KeySymbol::KEY_3)
      .value("KEY_4", osgGA::GUIEventAdapter::KeySymbol::KEY_4)
      .value("KEY_5", osgGA::GUIEventAdapter::KeySymbol::KEY_5)
      .value("KEY_6", osgGA::GUIEventAdapter::KeySymbol::KEY_6)
      .value("KEY_7", osgGA::GUIEventAdapter::KeySymbol::KEY_7)
      .value("KEY_8", osgGA::GUIEventAdapter::KeySymbol::KEY_8)
      .value("KEY_9", osgGA::GUIEventAdapter::KeySymbol::KEY_9)
      .value("KEY_A", osgGA::GUIEventAdapter::KeySymbol::KEY_A)
      .value("KEY_B", osgGA::GUIEventAdapter::KeySymbol::KEY_B)
      .value("KEY_C", osgGA::GUIEventAdapter::KeySymbol::KEY_C)
      .value("KEY_D", osgGA::GUIEventAdapter::KeySymbol::KEY_D)
      .value("KEY_E", osgGA::GUIEventAdapter::KeySymbol::KEY_E)
      .value("KEY_F", osgGA::GUIEventAdapter::KeySymbol::KEY_F)
      .value("KEY_G", osgGA::GUIEventAdapter::KeySymbol::KEY_G)
      .value("KEY_H", osgGA::GUIEventAdapter::KeySymbol::KEY_H)
      .value("KEY_I", osgGA::GUIEventAdapter::KeySymbol::KEY_I)
      .value("KEY_J", osgGA::GUIEventAdapter::KeySymbol::KEY_J)
      .value("KEY_K", osgGA::GUIEventAdapter::KeySymbol::KEY_K)
      .value("KEY_L", osgGA::GUIEventAdapter::KeySymbol::KEY_L)
      .value("KEY_M", osgGA::GUIEventAdapter::KeySymbol::KEY_M)
      .value("KEY_N", osgGA::GUIEventAdapter::KeySymbol::KEY_N)
      .value("KEY_O", osgGA::GUIEventAdapter::KeySymbol::KEY_O)
      .value("KEY_P", osgGA::GUIEventAdapter::KeySymbol::KEY_P)
      .value("KEY_Q", osgGA::GUIEventAdapter::KeySymbol::KEY_Q)
      .value("KEY_R", osgGA::GUIEventAdapter::KeySymbol::KEY_R)
      .value("KEY_S", osgGA::GUIEventAdapter::KeySymbol::KEY_S)
      .value("KEY_T", osgGA::GUIEventAdapter::KeySymbol::KEY_T)
      .value("KEY_U", osgGA::GUIEventAdapter::KeySymbol::KEY_U)
      .value("KEY_V", osgGA::GUIEventAdapter::KeySymbol::KEY_V)
      .value("KEY_W", osgGA::GUIEventAdapter::KeySymbol::KEY_W)
      .value("KEY_X", osgGA::GUIEventAdapter::KeySymbol::KEY_X)
      .value("KEY_Y", osgGA::GUIEventAdapter::KeySymbol::KEY_Y)
      .value("KEY_Z", osgGA::GUIEventAdapter::KeySymbol::KEY_Z)
      .value("KEY_Exclaim", osgGA::GUIEventAdapter::KeySymbol::KEY_Exclaim)
      .value("KEY_Quotedbl", osgGA::GUIEventAdapter::KeySymbol::KEY_Quotedbl)
      .value("KEY_Hash", osgGA::GUIEventAdapter::KeySymbol::KEY_Hash)
      .value("KEY_Dollar", osgGA::GUIEventAdapter::KeySymbol::KEY_Dollar)
      .value("KEY_Ampersand", osgGA::GUIEventAdapter::KeySymbol::KEY_Ampersand)
      .value("KEY_Quote", osgGA::GUIEventAdapter::KeySymbol::KEY_Quote)
      .value("KEY_Leftparen", osgGA::GUIEventAdapter::KeySymbol::KEY_Leftparen)
      .value(
          "KEY_Rightparen", osgGA::GUIEventAdapter::KeySymbol::KEY_Rightparen)
      .value("KEY_Asterisk", osgGA::GUIEventAdapter::KeySymbol::KEY_Asterisk)
      .value("KEY_Plus", osgGA::GUIEventAdapter::KeySymbol::KEY_Plus)
      .value("KEY_Comma", osgGA::GUIEventAdapter::KeySymbol::KEY_Comma)
      .value("KEY_Minus", osgGA::GUIEventAdapter::KeySymbol::KEY_Minus)
      .value("KEY_Period", osgGA::GUIEventAdapter::KeySymbol::KEY_Period)
      .value("KEY_Slash", osgGA::GUIEventAdapter::KeySymbol::KEY_Slash)
      .value("KEY_Colon", osgGA::GUIEventAdapter::KeySymbol::KEY_Colon)
      .value("KEY_Semicolon", osgGA::GUIEventAdapter::KeySymbol::KEY_Semicolon)
      .value("KEY_Less", osgGA::GUIEventAdapter::KeySymbol::KEY_Less)
      .value("KEY_Equals", osgGA::GUIEventAdapter::KeySymbol::KEY_Equals)
      .value("KEY_Greater", osgGA::GUIEventAdapter::KeySymbol::KEY_Greater)
      .value("KEY_Question", osgGA::GUIEventAdapter::KeySymbol::KEY_Question)
      .value("KEY_At", osgGA::GUIEventAdapter::KeySymbol::KEY_At)
      .value(
          "KEY_Leftbracket", osgGA::GUIEventAdapter::KeySymbol::KEY_Leftbracket)
      .value("KEY_Backslash", osgGA::GUIEventAdapter::KeySymbol::KEY_Backslash)
      .value(
          "KEY_Rightbracket",
          osgGA::GUIEventAdapter::KeySymbol::KEY_Rightbracket)
      .value("KEY_Caret", osgGA::GUIEventAdapter::KeySymbol::KEY_Caret)
      .value(
          "KEY_Underscore", osgGA::GUIEventAdapter::KeySymbol::KEY_Underscore)
      .value("KEY_Backquote", osgGA::GUIEventAdapter::KeySymbol::KEY_Backquote)
      .value("KEY_BackSpace", osgGA::GUIEventAdapter::KeySymbol::KEY_BackSpace)
      .value("KEY_Tab", osgGA::GUIEventAdapter::KeySymbol::KEY_Tab)
      .value("KEY_Linefeed", osgGA::GUIEventAdapter::KeySymbol::KEY_Linefeed)
      .value("KEY_Clear", osgGA::GUIEventAdapter::KeySymbol::KEY_Clear)
      .value("KEY_Return", osgGA::GUIEventAdapter::KeySymbol::KEY_Return)
      .value("KEY_Pause", osgGA::GUIEventAdapter::KeySymbol::KEY_Pause)
      .value(
          "KEY_Scroll_Lock", osgGA::GUIEventAdapter::KeySymbol::KEY_Scroll_Lock)
      .value("KEY_Sys_Req", osgGA::GUIEventAdapter::KeySymbol::KEY_Sys_Req)
      .value("KEY_Escape", osgGA::GUIEventAdapter::KeySymbol::KEY_Escape)
      .value("KEY_Delete", osgGA::GUIEventAdapter::KeySymbol::KEY_Delete)
      .value("KEY_Home", osgGA::GUIEventAdapter::KeySymbol::KEY_Home)
      .value("KEY_Left", osgGA::GUIEventAdapter::KeySymbol::KEY_Left)
      .value("KEY_Up", osgGA::GUIEventAdapter::KeySymbol::KEY_Up)
      .value("KEY_Right", osgGA::GUIEventAdapter::KeySymbol::KEY_Right)
      .value("KEY_Down", osgGA::GUIEventAdapter::KeySymbol::KEY_Down)
      .value("KEY_Prior", osgGA::GUIEventAdapter::KeySymbol::KEY_Prior)
      .value("KEY_Page_Up", osgGA::GUIEventAdapter::KeySymbol::KEY_Page_Up)
      .value("KEY_Next", osgGA::GUIEventAdapter::KeySymbol::KEY_Next)
      .value("KEY_Page_Down", osgGA::GUIEventAdapter::KeySymbol::KEY_Page_Down)
      .value("KEY_End", osgGA::GUIEventAdapter::KeySymbol::KEY_End)
      .value("KEY_Begin", osgGA::GUIEventAdapter::KeySymbol::KEY_Begin)
      .value("KEY_Select", osgGA::GUIEventAdapter::KeySymbol::KEY_Select)
      .value("KEY_Print", osgGA::GUIEventAdapter::KeySymbol::KEY_Print)
      .value("KEY_Execute", osgGA::GUIEventAdapter::KeySymbol::KEY_Execute)
      .value("KEY_Insert", osgGA::GUIEventAdapter::KeySymbol::KEY_Insert)
      .value("KEY_Undo", osgGA::GUIEventAdapter::KeySymbol::KEY_Undo)
      .value("KEY_Redo", osgGA::GUIEventAdapter::KeySymbol::KEY_Redo)
      .value("KEY_Menu", osgGA::GUIEventAdapter::KeySymbol::KEY_Menu)
      .value("KEY_Find", osgGA::GUIEventAdapter::KeySymbol::KEY_Find)
      .value("KEY_Cancel", osgGA::GUIEventAdapter::KeySymbol::KEY_Cancel)
      .value("KEY_Help", osgGA::GUIEventAdapter::KeySymbol::KEY_Help)
      .value("KEY_Break", osgGA::GUIEventAdapter::KeySymbol::KEY_Break)
      .value(
          "KEY_Mode_switch", osgGA::GUIEventAdapter::KeySymbol::KEY_Mode_switch)
      .value(
          "KEY_Script_switch",
          osgGA::GUIEventAdapter::KeySymbol::KEY_Script_switch)
      .value("KEY_Num_Lock", osgGA::GUIEventAdapter::KeySymbol::KEY_Num_Lock)
      .value("KEY_KP_Space", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Space)
      .value("KEY_KP_Tab", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Tab)
      .value("KEY_KP_Enter", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Enter)
      .value("KEY_KP_F1", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_F1)
      .value("KEY_KP_F2", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_F2)
      .value("KEY_KP_F3", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_F3)
      .value("KEY_KP_F4", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_F4)
      .value("KEY_KP_Home", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Home)
      .value("KEY_KP_Left", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Left)
      .value("KEY_KP_Up", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Up)
      .value("KEY_KP_Right", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Right)
      .value("KEY_KP_Down", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Down)
      .value("KEY_KP_Prior", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Prior)
      .value(
          "KEY_KP_Page_Up", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Page_Up)
      .value("KEY_KP_Next", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Next)
      .value(
          "KEY_KP_Page_Down",
          osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Page_Down)
      .value("KEY_KP_End", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_End)
      .value("KEY_KP_Begin", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Begin)
      .value("KEY_KP_Insert", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Insert)
      .value("KEY_KP_Delete", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Delete)
      .value("KEY_KP_Equal", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Equal)
      .value(
          "KEY_KP_Multiply", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Multiply)
      .value("KEY_KP_Add", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Add)
      .value(
          "KEY_KP_Separator",
          osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Separator)
      .value(
          "KEY_KP_Subtract", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Subtract)
      .value(
          "KEY_KP_Decimal", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Decimal)
      .value("KEY_KP_Divide", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_Divide)
      .value("KEY_KP_0", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_0)
      .value("KEY_KP_1", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_1)
      .value("KEY_KP_2", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_2)
      .value("KEY_KP_3", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_3)
      .value("KEY_KP_4", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_4)
      .value("KEY_KP_5", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_5)
      .value("KEY_KP_6", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_6)
      .value("KEY_KP_7", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_7)
      .value("KEY_KP_8", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_8)
      .value("KEY_KP_9", osgGA::GUIEventAdapter::KeySymbol::KEY_KP_9)
      .value("KEY_F1", osgGA::GUIEventAdapter::KeySymbol::KEY_F1)
      .value("KEY_F2", osgGA::GUIEventAdapter::KeySymbol::KEY_F2)
      .value("KEY_F3", osgGA::GUIEventAdapter::KeySymbol::KEY_F3)
      .value("KEY_F4", osgGA::GUIEventAdapter::KeySymbol::KEY_F4)
      .value("KEY_F5", osgGA::GUIEventAdapter::KeySymbol::KEY_F5)
      .value("KEY_F6", osgGA::GUIEventAdapter::KeySymbol::KEY_F6)
      .value("KEY_F7", osgGA::GUIEventAdapter::KeySymbol::KEY_F7)
      .value("KEY_F8", osgGA::GUIEventAdapter::KeySymbol::KEY_F8)
      .value("KEY_F9", osgGA::GUIEventAdapter::KeySymbol::KEY_F9)
      .value("KEY_F10", osgGA::GUIEventAdapter::KeySymbol::KEY_F10)
      .value("KEY_F11", osgGA::GUIEventAdapter::KeySymbol::KEY_F11)
      .value("KEY_F12", osgGA::GUIEventAdapter::KeySymbol::KEY_F12)
      .value("KEY_F13", osgGA::GUIEventAdapter::KeySymbol::KEY_F13)
      .value("KEY_F14", osgGA::GUIEventAdapter::KeySymbol::KEY_F14)
      .value("KEY_F15", osgGA::GUIEventAdapter::KeySymbol::KEY_F15)
      .value("KEY_F16", osgGA::GUIEventAdapter::KeySymbol::KEY_F16)
      .value("KEY_F17", osgGA::GUIEventAdapter::KeySymbol::KEY_F17)
      .value("KEY_F18", osgGA::GUIEventAdapter::KeySymbol::KEY_F18)
      .value("KEY_F19", osgGA::GUIEventAdapter::KeySymbol::KEY_F19)
      .value("KEY_F20", osgGA::GUIEventAdapter::KeySymbol::KEY_F20)
      .value("KEY_F21", osgGA::GUIEventAdapter::KeySymbol::KEY_F21)
      .value("KEY_F22", osgGA::GUIEventAdapter::KeySymbol::KEY_F22)
      .value("KEY_F23", osgGA::GUIEventAdapter::KeySymbol::KEY_F23)
      .value("KEY_F24", osgGA::GUIEventAdapter::KeySymbol::KEY_F24)
      .value("KEY_F25", osgGA::GUIEventAdapter::KeySymbol::KEY_F25)
      .value("KEY_F26", osgGA::GUIEventAdapter::KeySymbol::KEY_F26)
      .value("KEY_F27", osgGA::GUIEventAdapter::KeySymbol::KEY_F27)
      .value("KEY_F28", osgGA::GUIEventAdapter::KeySymbol::KEY_F28)
      .value("KEY_F29", osgGA::GUIEventAdapter::KeySymbol::KEY_F29)
      .value("KEY_F30", osgGA::GUIEventAdapter::KeySymbol::KEY_F30)
      .value("KEY_F31", osgGA::GUIEventAdapter::KeySymbol::KEY_F31)
      .value("KEY_F32", osgGA::GUIEventAdapter::KeySymbol::KEY_F32)
      .value("KEY_F33", osgGA::GUIEventAdapter::KeySymbol::KEY_F33)
      .value("KEY_F34", osgGA::GUIEventAdapter::KeySymbol::KEY_F34)
      .value("KEY_F35", osgGA::GUIEventAdapter::KeySymbol::KEY_F35)
      .value("KEY_Shift_L", osgGA::GUIEventAdapter::KeySymbol::KEY_Shift_L)
      .value("KEY_Shift_R", osgGA::GUIEventAdapter::KeySymbol::KEY_Shift_R)
      .value("KEY_Control_L", osgGA::GUIEventAdapter::KeySymbol::KEY_Control_L)
      .value("KEY_Control_R", osgGA::GUIEventAdapter::KeySymbol::KEY_Control_R)
      .value("KEY_Caps_Lock", osgGA::GUIEventAdapter::KeySymbol::KEY_Caps_Lock)
      .value(
          "KEY_Shift_Lock", osgGA::GUIEventAdapter::KeySymbol::KEY_Shift_Lock)
      .value("KEY_Meta_L", osgGA::GUIEventAdapter::KeySymbol::KEY_Meta_L)
      .value("KEY_Meta_R", osgGA::GUIEventAdapter::KeySymbol::KEY_Meta_R)
      .value("KEY_Alt_L", osgGA::GUIEventAdapter::KeySymbol::KEY_Alt_L)
      .value("KEY_Alt_R", osgGA::GUIEventAdapter::KeySymbol::KEY_Alt_R)
      .value("KEY_Super_L", osgGA::GUIEventAdapter::KeySymbol::KEY_Super_L)
      .value("KEY_Super_R", osgGA::GUIEventAdapter::KeySymbol::KEY_Super_R)
      .value("KEY_Hyper_L", osgGA::GUIEventAdapter::KeySymbol::KEY_Hyper_L)
      .value("KEY_Hyper_R", osgGA::GUIEventAdapter::KeySymbol::KEY_Hyper_R)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::ModKeyMask>(ea, "ModKeyMask")
      .value(
          "MODKEY_LEFT_SHIFT",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_LEFT_SHIFT)
      .value(
          "MODKEY_RIGHT_SHIFT",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_RIGHT_SHIFT)
      .value(
          "MODKEY_LEFT_CTRL",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_LEFT_CTRL)
      .value(
          "MODKEY_RIGHT_CTRL",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_RIGHT_CTRL)
      .value(
          "MODKEY_LEFT_ALT",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_LEFT_ALT)
      .value(
          "MODKEY_RIGHT_ALT",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_RIGHT_ALT)
      .value(
          "MODKEY_LEFT_META",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_LEFT_META)
      .value(
          "MODKEY_RIGHT_META",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_RIGHT_META)
      .value(
          "MODKEY_LEFT_SUPER",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_LEFT_SUPER)
      .value(
          "MODKEY_RIGHT_SUPER",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_RIGHT_SUPER)
      .value(
          "MODKEY_LEFT_HYPER",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_LEFT_HYPER)
      .value(
          "MODKEY_RIGHT_HYPER",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_RIGHT_HYPER)
      .value(
          "MODKEY_NUM_LOCK",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_NUM_LOCK)
      .value(
          "MODKEY_CAPS_LOCK",
          osgGA::GUIEventAdapter::ModKeyMask::MODKEY_CAPS_LOCK)
      .value("MODKEY_CTRL", osgGA::GUIEventAdapter::ModKeyMask::MODKEY_CTRL)
      .value("MODKEY_SHIFT", osgGA::GUIEventAdapter::ModKeyMask::MODKEY_SHIFT)
      .value("MODKEY_ALT", osgGA::GUIEventAdapter::ModKeyMask::MODKEY_ALT)
      .value("MODKEY_META", osgGA::GUIEventAdapter::ModKeyMask::MODKEY_META)
      .value("MODKEY_SUPER", osgGA::GUIEventAdapter::ModKeyMask::MODKEY_SUPER)
      .value("MODKEY_HYPER", osgGA::GUIEventAdapter::ModKeyMask::MODKEY_HYPER)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::MouseYOrientation>(ea, "MouseYOrientation")
      .value(
          "Y_INCREASING_UPWARDS",
          osgGA::GUIEventAdapter::MouseYOrientation::Y_INCREASING_UPWARDS)
      .value(
          "Y_INCREASING_DOWNWARDS",
          osgGA::GUIEventAdapter::MouseYOrientation::Y_INCREASING_DOWNWARDS)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::ScrollingMotion>(ea, "ScrollingMotion")
      .value(
          "SCROLL_NONE", osgGA::GUIEventAdapter::ScrollingMotion::SCROLL_NONE)
      .value(
          "SCROLL_LEFT", osgGA::GUIEventAdapter::ScrollingMotion::SCROLL_LEFT)
      .value(
          "SCROLL_RIGHT", osgGA::GUIEventAdapter::ScrollingMotion::SCROLL_RIGHT)
      .value("SCROLL_UP", osgGA::GUIEventAdapter::ScrollingMotion::SCROLL_UP)
      .value(
          "SCROLL_DOWN", osgGA::GUIEventAdapter::ScrollingMotion::SCROLL_DOWN)
      .value("SCROLL_2D", osgGA::GUIEventAdapter::ScrollingMotion::SCROLL_2D)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::TabletPointerType>(ea, "TabletPointerType")
      .value("UNKNOWN", osgGA::GUIEventAdapter::TabletPointerType::UNKNOWN)
      .value("PEN", osgGA::GUIEventAdapter::TabletPointerType::PEN)
      .value("PUCK", osgGA::GUIEventAdapter::TabletPointerType::PUCK)
      .value("ERASER", osgGA::GUIEventAdapter::TabletPointerType::ERASER)
      .export_values();

  nb::enum_<osgGA::GUIEventAdapter::TouchPhase>(ea, "TouchPhase")
      .value("TOUCH_UNKNOWN", osgGA::GUIEventAdapter::TouchPhase::TOUCH_UNKNOWN)
      .value("TOUCH_BEGAN", osgGA::GUIEventAdapter::TouchPhase::TOUCH_BEGAN)
      .value("TOUCH_MOVED", osgGA::GUIEventAdapter::TouchPhase::TOUCH_MOVED)
      .value(
          "TOUCH_STATIONERY",
          osgGA::GUIEventAdapter::TouchPhase::TOUCH_STATIONERY)
      .value("TOUCH_ENDED", osgGA::GUIEventAdapter::TouchPhase::TOUCH_ENDED)
      .export_values();

  nb::class_<osgGA::GUIActionAdapter>(m, "GUIActionAdapter");

  nb::class_<osgGA::GUIEventHandler>(m, "__GUIEventHandler__")
      .def(nb::init<>());

  nb::class_<GUIEventHandlerNoRef, osgGA::GUIEventHandler, PyGUIEventHandler>(
      m, "GUIEventHandler")
      .def(nb::init<>());
}

} // namespace dart::python_nb
