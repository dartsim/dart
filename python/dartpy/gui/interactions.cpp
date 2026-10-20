// clang-format off
#include "detail/dart_nb.hpp"
#include "detail/gil.hpp"
// clang-format on
#include "detail/eigen.hpp"
#include "gui/ownership.hpp"

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

#include <dart/gui/DefaultEventHandler.hpp>
#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/GUIEventHandler.hpp>
#include <dart/gui/InteractiveFrame.hpp>
#include <dart/gui/Viewer.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>

#include <dart/common/Observer.hpp>
#include <dart/common/Subject.hpp>

#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include <type_traits>
#include <unordered_map>

namespace dartnb {
template <>
inline std::vector<BaseEdge> extra_base_edges<dart::gui::DragAndDrop>()
{
  return {
      base_edge<dart::gui::DragAndDrop, dart::common::Subject>(),
      base_edge<dart::gui::DragAndDrop, dart::common::Observer>()};
}
template <>
inline std::vector<BaseEdge> extra_base_edges<dart::gui::DefaultEventHandler>()
{
  return {
      base_edge<dart::gui::DefaultEventHandler, dart::common::Subject>(),
      base_edge<dart::gui::DefaultEventHandler, dart::common::Observer>()};
}
} // namespace dartnb

namespace dart {
namespace python {
namespace {

template <class Extra>
const Extra& factoryArgument(const Extra& extra)
{
  return extra;
}

template <std::size_t Nurse, std::size_t Patient>
auto factoryArgument(const nb::keep_alive<Nurse, Patient>&)
{
  static_assert(Nurse == 1 && Patient > 1);
  return nb::keep_alive<0, Patient - 1>();
}

template <class... Args>
struct RetainingInit : nb::def_visitor<RetainingInit<Args...>>
{
  template <class Cls, class... Extra>
  void execute(Cls& cls, const Extra&... extra) const
  {
    using T = typename Cls::Type;
    static_assert(std::is_same_v<T, typename Cls::Alias>);
    // Static factories count arguments from zero's result, initializers from
    // self.
    dartnb::defHybridNew<T>(
        cls,
        [](Args... args) {
          return dartnb::nativeInstance(
              nb::type<T>(), std::make_shared<T>(args...));
        },
        factoryArgument(extra)...);
    cls.def(
        "__init__",
        [](nb::handle self, Args... args) {
          dartnb::initializeInPlace<T>(self, args...);
        },
        extra...);
  }
};

class IdentityWeakCache : public std::enable_shared_from_this<IdentityWeakCache>
{
public:
  ~IdentityWeakCache()
  {
    if (nb::detail::cleanup_guard guard{})
      mEntries.clear();
    else {
      // The interpreter is finalizing; intentionally leak the Python handles.
      for (auto& entry : mEntries) {
        entry.second.key.release();
        entry.second.value.release();
      }
    }
  }

  nb::object get(const nb::handle& object) const
  {
    const auto entry = mEntries.find(object.ptr());
    if (entry == mEntries.end() || !entry->second.key().is(object))
      return nb::none();
    return entry->second.value;
  }

  void set(const nb::handle& object, nb::object value)
  {
    auto* identity = object.ptr();
    const std::weak_ptr<IdentityWeakCache> cache = shared_from_this();
    auto callback = nb::cpp_function([cache,
                                      identity](const nb::weakref& reference) {
      if (auto owners = cache.lock()) {
        const auto entry = owners->mEntries.find(identity);
        if (entry != owners->mEntries.end() && entry->second.key.is(reference))
          owners->mEntries.erase(entry);
      }
    });
    // Pointer keys avoid subclass hash/equality; the weakref verifies identity
    // and releases each value when its wrapper dies, without retaining it.
    mEntries.insert_or_assign(
        identity, Entry{nb::weakref(object, callback), std::move(value)});
  }

private:
  struct Entry
  {
    nb::weakref key;
    nb::object value;
  };
  std::unordered_map<PyObject*, Entry> mEntries;
};

using OwnerCache = std::shared_ptr<IdentityWeakCache>;

nb::capsule cacheCapsule(const OwnerCache& cache)
{
  return nb::capsule(new OwnerCache(cache), [](void* pointer) noexcept {
    delete static_cast<OwnerCache*>(pointer);
  });
}

OwnerCache cacheFromCapsule(const nb::handle& capsule)
{
  return *static_cast<OwnerCache*>(nb::cast<nb::capsule>(capsule).data());
}

struct DragOwners
{
  std::weak_ptr<dynamics::Entity> entity;
  std::weak_ptr<dynamics::Shape> shape;
};

template <typename Native>
nb::object ownedNative(const std::shared_ptr<Native>& owner)
{
  if (!owner)
    return nb::none();
  nb::object object;
  if constexpr (std::is_base_of_v<dynamics::Entity, Native>) {
    // Core body bindings use DART's node holders, which retain bodies across
    // skeleton changes and differ from the shared holders of other entities.
    // Entity is a virtual base. Keep exact graph holders and native shared
    // owners distinct when returning the most-derived frame wrapper.
    if (auto* soft = dynamic_cast<dynamics::SoftBodyNode*>(owner.get()))
      object = nb::cast(dynamics::SoftBodyNodePtr(soft));
    else if (auto* body = dynamic_cast<dynamics::BodyNode*>(owner.get()))
      object = nb::cast(dynamics::BodyNodePtr(body));
    else if (
        auto frame = std::dynamic_pointer_cast<gui::InteractiveFrame>(owner))
      object = nb::cast(frame);
    else if (
        auto frame = std::dynamic_pointer_cast<dynamics::SimpleFrame>(owner))
      object = nb::cast(frame);
    else if (auto node = std::dynamic_pointer_cast<dynamics::ShapeNode>(owner))
      object = nb::cast(node);
    else
      object = nb::cast(owner);
  } else {
    object = nb::cast(owner);
  }
  // DART's shared caster retains native owners on new or borrowed wrappers.
  // Python-owned subclasses already keep their storage and must not pin
  // themselves.
  return object;
}

template <typename Native>
std::shared_ptr<Native> existingOwner(Native* pointer)
{
  if (!pointer)
    return nullptr;
  const nb::object object = nb::cast(pointer, nb::rv_policy::reference);
  try {
    return nb::cast<std::shared_ptr<Native>>(object);
  } catch (const nb::cast_error&) {
    return nullptr;
  }
}

std::shared_ptr<dynamics::Entity> dragEntityOwner(
    const nb::object& self, const OwnerCache& dragOwners)
{
  auto* entity = nb::cast<const gui::DragAndDrop&>(self).getEntity();
  if (!entity)
    return nullptr;
  if (auto* body = dynamic_cast<dynamics::BodyNode*>(entity)) {
    auto owner = std::make_shared<dynamics::BodyNodePtr>(body);
    return std::shared_ptr<dynamics::Entity>(owner, body);
  }
  const auto cached = dragOwners->get(self);
  if (!cached.is_none()) {
    auto* owners
        = static_cast<DragOwners*>(nb::cast<nb::capsule>(cached).data());
    auto owner = owners->entity.lock();
    if (owner.get() == entity)
      return owner;
  }
  // Direct constructors keep their Python arguments alive. Retrieve the
  // original holder, rejecting borrowed wrappers without shared storage.
  auto owner = existingOwner(entity);
  if (!owner)
    throw std::runtime_error(
        "Dragged entity has no shared owner; its lifetime cannot be retained "
        "safely by Python");
  return owner;
}

std::shared_ptr<dynamics::Shape> dragShapeOwner(
    const nb::object& self, const OwnerCache& dragOwners)
{
  const auto& drag = nb::cast<const gui::SimpleFrameShapeDnD&>(self);
  auto* shape = drag.getShape();
  if (!shape)
    return nullptr;
  if (auto* frame = drag.getSimpleFrame()) {
    auto owner = frame->getShape();
    if (owner.get() == shape)
      return owner;
  }
  const auto cached = dragOwners->get(self);
  if (!cached.is_none()) {
    auto* owners
        = static_cast<DragOwners*>(nb::cast<nb::capsule>(cached).data());
    auto owner = owners->shape.lock();
    if (owner.get() == shape)
      return owner;
  }
  auto owner = existingOwner(shape);
  if (!owner)
    throw std::runtime_error(
        "Dragged shape has no shared owner; its lifetime cannot be retained "
        "safely by Python");
  return owner;
}

class PyGUIEventHandler : public gui::GUIEventHandler
{
public:
  NB_TRAMPOLINE(gui::GUIEventHandler);

  bool handle(
      const gui::GUIEventAdapter& event, gui::GUIActionAdapter& viewer) override
  {
    dartnb::GilScopedAcquire gil;
    nb::detail::ticket ticket(
        nb_trampoline, "handle", nb::detail::str_hash("handle"), true);
    // Native input and action adapters are stack-owned. Python handlers may
    // retain them, so pass an event copy and an action with Viewer keep-alive.
    auto action = nb::type<gui::GUIActionAdapter>()(
        nb::cast(&viewer.getViewer(), nb::rv_policy::reference));
    return nb::cast<bool>(nb_trampoline.base().attr(ticket.key)(
        nb::cast(gui::GUIEventAdapter(event)), action));
  }
};

nb::object ownedChild(
    nb::object child, const nb::object& parent, const OwnerCache& parentOwners)
{
  if (!child.is_none())
    parentOwners->set(child, parent);
  return child;
}

template <typename Owner>
nb::list ownedShapeFrames(nb::object self, const OwnerCache& parentOwners)
{
  nb::list result;
  for (const auto& frame : nb::cast<Owner&>(self).getShapeFrameOwners()) {
    auto child = nb::cast(frame);
    // A child has shared storage, but its kinematic parent is a raw pointer.
    // Keep the parent wrapper alive separately for each returned child.
    result.append(ownedChild(std::move(child), self, parentOwners));
  }
  return result;
}

} // namespace

nb::object dart_gui_owned_drag(
    const std::shared_ptr<gui::DragAndDrop>& drag, const nb::tuple& ownership)
{
  if (!drag)
    return nb::none();
  const auto dragOwners = cacheFromCapsule(ownership[0]);
  auto object = nb::cast(drag);
  auto owners = std::make_unique<DragOwners>();
  // BodyNode wrappers hold BodyNodePtr, not shared_ptr; dragEntityOwner()
  // creates their owner on demand instead.
  if (!dynamic_cast<dynamics::BodyNode*>(drag->getEntity()))
    owners->entity = existingOwner(drag->getEntity());
  if (const auto shapeDrag
      = std::dynamic_pointer_cast<gui::SimpleFrameShapeDnD>(drag))
    owners->shape = existingOwner(shapeDrag->getShape());
  // Observers remain weak. Returned wrappers acquire immutable strong owners
  // only when a getter is called, independently of later DnD detachment.
  dragOwners->set(
      object, nb::capsule(owners.release(), [](void* pointer) noexcept {
        delete static_cast<DragOwners*>(pointer);
      }));
  return object;
}

nb::tuple dart_gui_interactions(nb::module_& m)
{
  // Binding closures own the caches; weak callbacks never retain those caches.
  const auto parentOwners = std::make_shared<IdentityWeakCache>();
  const auto dragOwners = std::make_shared<IdentityWeakCache>();
  using Event = gui::GUIEventAdapter;
  auto event = dartnb::dart_class<Event>(m, "GUIEventAdapter");
  nb::enum_<Event::EventType>(event, "EventType", nb::is_arithmetic())
      .value("NONE", Event::NONE)
      .value("PUSH", Event::PUSH)
      .value("RELEASE", Event::RELEASE)
      .value("DRAG", Event::DRAG)
      .value("MOVE", Event::MOVE)
      .value("KEYDOWN", Event::KEYDOWN)
      .value("KEYUP", Event::KEYUP)
      .value("SCROLL", Event::SCROLL)
      .value("FRAME", Event::FRAME)
      .value("CLOSE_WINDOW", Event::CLOSE_WINDOW)
      .value("QUIT_APPLICATION", Event::QUIT_APPLICATION)
      .export_values();
  nb::enum_<Event::MouseButtonMask>(
      event, "MouseButtonMask", nb::is_arithmetic(), nb::is_flag())
      .value("LEFT_MOUSE_BUTTON", Event::LEFT_MOUSE_BUTTON)
      .value("MIDDLE_MOUSE_BUTTON", Event::MIDDLE_MOUSE_BUTTON)
      .value("RIGHT_MOUSE_BUTTON", Event::RIGHT_MOUSE_BUTTON)
      .export_values();
  nb::enum_<Event::ScrollingMotion>(
      event, "ScrollingMotion", nb::is_arithmetic())
      .value("SCROLL_NONE", Event::SCROLL_NONE)
      .value("SCROLL_LEFT", Event::SCROLL_LEFT)
      .value("SCROLL_RIGHT", Event::SCROLL_RIGHT)
      .value("SCROLL_UP", Event::SCROLL_UP)
      .value("SCROLL_DOWN", Event::SCROLL_DOWN)
      .value("SCROLL_2D", Event::SCROLL_2D)
      .export_values();
  nb::enum_<Event::ModKeyMask>(
      event, "ModKeyMask", nb::is_arithmetic(), nb::is_flag())
      .value("MODKEY_LEFT_SHIFT", Event::MODKEY_LEFT_SHIFT)
      .value("MODKEY_RIGHT_SHIFT", Event::MODKEY_RIGHT_SHIFT)
      .value("MODKEY_LEFT_CTRL", Event::MODKEY_LEFT_CTRL)
      .value("MODKEY_RIGHT_CTRL", Event::MODKEY_RIGHT_CTRL)
      .value("MODKEY_LEFT_ALT", Event::MODKEY_LEFT_ALT)
      .value("MODKEY_RIGHT_ALT", Event::MODKEY_RIGHT_ALT)
      .value("MODKEY_SHIFT", Event::MODKEY_SHIFT)
      .value("MODKEY_CTRL", Event::MODKEY_CTRL)
      .value("MODKEY_ALT", Event::MODKEY_ALT)
      .value("MODKEY_SUPER", Event::MODKEY_SUPER)
      .export_values();
  nb::enum_<Event::KeySymbol>(event, "KeySymbol", nb::is_arithmetic())
      .value("KEY_BackSpace", Event::KEY_BackSpace)
      .value("KEY_Tab", Event::KEY_Tab)
      .value("KEY_Return", Event::KEY_Return)
      .value("KEY_Escape", Event::KEY_Escape)
      .value("KEY_Home", Event::KEY_Home)
      .value("KEY_Left", Event::KEY_Left)
      .value("KEY_Up", Event::KEY_Up)
      .value("KEY_Right", Event::KEY_Right)
      .value("KEY_Down", Event::KEY_Down)
      .value("KEY_Page_Up", Event::KEY_Page_Up)
      .value("KEY_Page_Down", Event::KEY_Page_Down)
      .value("KEY_End", Event::KEY_End)
      .value("KEY_Insert", Event::KEY_Insert)
      .value("KEY_F1", Event::KEY_F1)
      .value("KEY_F2", Event::KEY_F2)
      .value("KEY_F3", Event::KEY_F3)
      .value("KEY_F4", Event::KEY_F4)
      .value("KEY_F5", Event::KEY_F5)
      .value("KEY_F6", Event::KEY_F6)
      .value("KEY_F7", Event::KEY_F7)
      .value("KEY_F8", Event::KEY_F8)
      .value("KEY_F9", Event::KEY_F9)
      .value("KEY_F10", Event::KEY_F10)
      .value("KEY_F11", Event::KEY_F11)
      .value("KEY_F12", Event::KEY_F12)
      .value("KEY_Shift_L", Event::KEY_Shift_L)
      .value("KEY_Shift_R", Event::KEY_Shift_R)
      .value("KEY_Control_L", Event::KEY_Control_L)
      .value("KEY_Control_R", Event::KEY_Control_R)
      .value("KEY_Alt_L", Event::KEY_Alt_L)
      .value("KEY_Alt_R", Event::KEY_Alt_R)
      .value("KEY_Delete", Event::KEY_Delete)
      .export_values();

  event
      .def(
          dartnb::init<
              Event::EventType,
              int,
              double,
              double,
              int,
              int,
              double,
              double>(),
          nb::arg("type") = Event::NONE,
          nb::arg("key") = 0,
          nb::arg("x") = 0.0,
          nb::arg("y") = 0.0,
          nb::arg("buttons") = 0,
          nb::arg("modifiers") = 0,
          nb::arg("scrollX") = 0.0,
          nb::arg("scrollY") = 0.0)
      .def("getEventType", &Event::getEventType)
      .def("getKey", &Event::getKey)
      .def("getX", &Event::getX)
      .def("getY", &Event::getY)
      .def("getButtonMask", &Event::getButtonMask)
      .def("getModKeyMask", &Event::getModKeyMask)
      .def("getScrollingDeltaX", &Event::getScrollingDeltaX)
      .def("getScrollingDeltaY", &Event::getScrollingDeltaY)
      .def("getScrollingMotion", &Event::getScrollingMotion);

  dartnb::dart_class<gui::GUIActionAdapter>(m, "GUIActionAdapter")
      .def(
          dartnb::init<gui::Viewer&>(),
          nb::arg("viewer"),
          nb::keep_alive<1, 2>())
      .def(
          "getViewer",
          &gui::GUIActionAdapter::getViewer,
          nb::rv_policy::reference_internal);
  dartnb::dart_class<gui::GUIEventHandler, PyGUIEventHandler>(
      m, "GUIEventHandler")
      .def(dartnb::init<>())
      .def("handle", &gui::GUIEventHandler::handle);

  nb::enum_<gui::MouseButton>(m, "MouseButton", nb::is_arithmetic())
      .value("LEFT_MOUSE", gui::LEFT_MOUSE)
      .value("RIGHT_MOUSE", gui::RIGHT_MOUSE)
      .value("MIDDLE_MOUSE", gui::MIDDLE_MOUSE)
      .value("NUM_MOUSE_BUTTONS", gui::NUM_MOUSE_BUTTONS)
      .export_values();
  nb::enum_<gui::MouseButtonEvent>(m, "MouseButtonEvent", nb::is_arithmetic())
      .value("BUTTON_PUSH", gui::BUTTON_PUSH)
      .value("BUTTON_DRAG", gui::BUTTON_DRAG)
      .value("BUTTON_RELEASE", gui::BUTTON_RELEASE)
      .value("BUTTON_NOTHING", gui::BUTTON_NOTHING)
      .export_values();
  nb::enum_<gui::ConstraintType>(m, "ConstraintType", nb::is_arithmetic())
      .value("UNCONSTRAINED", gui::UNCONSTRAINED)
      .value("LINE_CONSTRAINT", gui::LINE_CONSTRAINT)
      .value("PLANE_CONSTRAINT", gui::PLANE_CONSTRAINT)
      .value("CUSTOM_CONSTRAINT", gui::CUSTOM_CONSTRAINT)
      .value("NUM_CONSTRAINT_TYPES", gui::NUM_CONSTRAINT_TYPES)
      .export_values();
  dartnb::dart_class<gui::PickInfo>(m, "PickInfo")
      .def(dartnb::init<>())
      .def_prop_rw(
          "frame",
          [](nb::object self) -> nb::object {
            const auto& pick = nb::cast<const gui::PickInfo&>(self);
            if (!pick.frame)
              return nb::none();
            const auto owner = pick.getFrameOwner();
            if (!owner)
              throw std::runtime_error(
                  "Picked frame has no shared owner; its lifetime cannot be "
                  "retained safely by Python");
            // Existing borrowed core wrappers need an immutable exact owner
            // independently of later PickInfo.frame mutations.
            return ownedNative(owner);
          },
          &gui::PickInfo::setFrameOwner,
          nb::for_setter(nb::arg("value").none()))
      .def_rw(
          "shape",
          &gui::PickInfo::shape,
          nb::for_setter(nb::arg("value").none()))
      .def_rw("position", &gui::PickInfo::position)
      .def_rw("normal", &gui::PickInfo::normal);
  // Each Viewer owns its default handler; Python reaches it through
  // Viewer.getDefaultEventHandler() so it cannot outlive that viewer.
  dartnb::dart_class<gui::DefaultEventHandler, gui::GUIEventHandler>(
      m, "DefaultEventHandler")
      .def(
          "getButtonEvent",
          [](const gui::DefaultEventHandler& self, int button) {
            return self.getButtonEvent(static_cast<gui::MouseButton>(button));
          })
      .def("getModKeyMask", &gui::DefaultEventHandler::getModKeyMask)
      .def("getWindowCursorX", &gui::DefaultEventHandler::getWindowCursorX)
      .def("getWindowCursorY", &gui::DefaultEventHandler::getWindowCursorY)
      .def(
          "getDeltaCursor",
          &gui::DefaultEventHandler::getDeltaCursor,
          nb::arg("fromPosition"),
          nb::arg("constraint") = gui::UNCONSTRAINED,
          nb::arg("constraintVector")
          = Eigen::Vector3d(Eigen::Vector3d::UnitZ()))
      .def(
          "getNearAndFarPointUnderCursor",
          [](const gui::DefaultEventHandler& self, double distance) {
            Eigen::Vector3d nearPoint, farPoint;
            self.getNearAndFarPointUnderCursor(nearPoint, farPoint, distance);
            return nb::make_tuple(nearPoint, farPoint);
          },
          nb::arg("distance") = 1.0)
      .def(
          "getButtonPicks",
          [](const gui::DefaultEventHandler& self, int button, int event) {
            return self.getButtonPicks(
                static_cast<gui::MouseButton>(button),
                static_cast<gui::MouseButtonEvent>(event));
          })
      .def("getMovePicks", &gui::DefaultEventHandler::getMovePicks)
      .def(
          "suppressButtonPicks",
          [](gui::DefaultEventHandler& self, int button, int event) {
            self.suppressButtonPicks(
                static_cast<gui::MouseButton>(button),
                static_cast<gui::MouseButtonEvent>(event));
          })
      .def("suppressMovePicks", &gui::DefaultEventHandler::suppressMovePicks)
      .def(
          "activateButtonPicks",
          [](gui::DefaultEventHandler& self, int button, int event) {
            self.activateButtonPicks(
                static_cast<gui::MouseButton>(button),
                static_cast<gui::MouseButtonEvent>(event));
          })
      .def("activateMovePicks", &gui::DefaultEventHandler::activateMovePicks)
      .def(
          "pick",
          [](gui::DefaultEventHandler& self, const Event& event) {
            std::vector<gui::PickInfo> picks;
            self.pick(picks, event);
            return picks;
          })
      .def("handle", &gui::DefaultEventHandler::handle);

  auto tool
      = dartnb::dart_class<gui::InteractiveTool, dynamics::SimpleFrame>(
            m, "InteractiveTool")
            .def(
                RetainingInit<
                    gui::InteractiveFrame*,
                    double,
                    const std::string&>(),
                nb::arg("frame"),
                nb::arg("defaultAlpha"),
                nb::arg("name"),
                nb::keep_alive<1, 2>())
            .def("setEnabled", &gui::InteractiveTool::setEnabled)
            .def("getEnabled", &gui::InteractiveTool::getEnabled)
            .def("setAlpha", &gui::InteractiveTool::setAlpha)
            .def("resetAlpha", &gui::InteractiveTool::resetAlpha)
            .def(
                "setDefaultAlpha",
                &gui::InteractiveTool::setDefaultAlpha,
                nb::arg("alpha"),
                nb::arg("reset") = true)
            .def("getDefaultAlpha", &gui::InteractiveTool::getDefaultAlpha)
            .def(
                "getInteractiveFrame",
                nb::overload_cast<>(&gui::InteractiveTool::getInteractiveFrame),
                nb::rv_policy::reference_internal)
            .def(
                "addShapeFrame",
                [parentOwners](
                    nb::object self, const dynamics::ShapePtr& shape) {
                  return ownedChild(
                      nb::cast(nb::cast<gui::InteractiveTool&>(self)
                                   .addShapeFrameShared(shape)),
                      self,
                      parentOwners);
                })
            .def(
                "getShapeFrames",
                [parentOwners](nb::object self) {
                  return ownedShapeFrames<gui::InteractiveTool>(
                      self, parentOwners);
                })
            .def(
                "removeAllShapeFrames",
                &gui::InteractiveTool::removeAllShapeFrames);
  nb::enum_<gui::InteractiveTool::Type>(tool, "Type", nb::is_arithmetic())
      .value("LINEAR", gui::InteractiveTool::LINEAR)
      .value("ANGULAR", gui::InteractiveTool::ANGULAR)
      .value("PLANAR", gui::InteractiveTool::PLANAR)
      .value("NUM_TYPES", gui::InteractiveTool::NUM_TYPES)
      .export_values();
  dartnb::dart_class<gui::InteractiveFrame, dynamics::SimpleFrame>(
      m, "InteractiveFrame")
      .def(
          RetainingInit<
              dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double,
              double>(),
          nb::arg("referenceFrame"),
          nb::arg("name") = "interactive_frame",
          nb::arg("relativeTransform") = Eigen::Isometry3d::Identity(),
          nb::arg("sizeScale") = 0.2,
          nb::arg("thicknessScale") = 2.0,
          nb::keep_alive<1, 2>())
      .def(
          "resizeStandardVisuals",
          &gui::InteractiveFrame::resizeStandardVisuals,
          nb::arg("sizeScale") = 0.2,
          nb::arg("thicknessScale") = 2.0)
      .def(
          "getTool",
          [parentOwners](
              nb::object self,
              gui::InteractiveTool::Type type,
              std::size_t coordinate) {
            return ownedChild(
                nb::cast(nb::cast<gui::InteractiveFrame&>(self).getToolShared(
                    type, coordinate)),
                self,
                parentOwners);
          })
      .def(
          "addShapeFrame",
          [parentOwners](nb::object self, const dynamics::ShapePtr& shape) {
            return ownedChild(
                nb::cast(
                    nb::cast<gui::InteractiveFrame&>(self).addShapeFrameShared(
                        shape)),
                self,
                parentOwners);
          })
      .def(
          "getShapeFrames",
          [parentOwners](nb::object self) {
            return ownedShapeFrames<gui::InteractiveFrame>(self, parentOwners);
          })
      .def(
          "removeAllShapeFrames", &gui::InteractiveFrame::removeAllShapeFrames);

  auto drag
      = dartnb::dart_class<gui::DragAndDrop>(m, "DragAndDrop")
            .def(
                "getEntity",
                [dragOwners](const nb::object& self) {
                  return ownedNative(dragEntityOwner(self, dragOwners));
                })
            .def("update", &gui::DragAndDrop::update)
            .def("setObstructable", &gui::DragAndDrop::setObstructable)
            .def("isObstructable", &gui::DragAndDrop::isObstructable)
            .def("move", &gui::DragAndDrop::move)
            .def("saveState", &gui::DragAndDrop::saveState)
            .def("release", &gui::DragAndDrop::release)
            .def("getConstrainedDx", &gui::DragAndDrop::getConstrainedDx)
            .def(
                "getConstrainedRotation",
                &gui::DragAndDrop::getConstrainedRotation)
            .def("unconstrain", &gui::DragAndDrop::unconstrain)
            .def("constrainToLine", &gui::DragAndDrop::constrainToLine)
            .def("constrainToPlane", &gui::DragAndDrop::constrainToPlane)
            .def("isMoving", &gui::DragAndDrop::isMoving)
            .def("setRotationOption", &gui::DragAndDrop::setRotationOption)
            .def("getRotationOption", &gui::DragAndDrop::getRotationOption)
            .def("setRotationModKey", &gui::DragAndDrop::setRotationModKey)
            .def("getRotationModKey", &gui::DragAndDrop::getRotationModKey);
  nb::enum_<gui::DragAndDrop::RotationOption>(
      drag, "RotationOption", nb::is_arithmetic())
      .value("HOLD_MODKEY", gui::DragAndDrop::RotationOption::HOLD_MODKEY)
      .value("ALWAYS_ON", gui::DragAndDrop::RotationOption::ALWAYS_ON)
      .value("ALWAYS_OFF", gui::DragAndDrop::RotationOption::ALWAYS_OFF)
      .export_values();
  dartnb::dart_class<gui::SimpleFrameDnD, gui::DragAndDrop>(m, "SimpleFrameDnD")
      .def(
          RetainingInit<gui::Viewer*, dynamics::SimpleFrame*>(),
          nb::arg("viewer"),
          nb::arg("frame"),
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>())
      .def(
          "getSimpleFrame",
          [dragOwners](const nb::object& self) {
            return ownedNative(dragEntityOwner(self, dragOwners));
          })
      .def("move", &gui::SimpleFrameDnD::move)
      .def("saveState", &gui::SimpleFrameDnD::saveState);
  dartnb::dart_class<gui::SimpleFrameShapeDnD, gui::SimpleFrameDnD>(
      m, "SimpleFrameShapeDnD")
      .def(
          RetainingInit<
              gui::Viewer*,
              dynamics::SimpleFrame*,
              dynamics::Shape*>(),
          nb::arg("viewer"),
          nb::arg("frame"),
          nb::arg("shape"),
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>(),
          nb::keep_alive<1, 4>())
      .def(
          "getShape",
          [dragOwners](const nb::object& self) {
            return ownedNative(dragShapeOwner(self, dragOwners));
          })
      .def("update", &gui::SimpleFrameShapeDnD::update);
  dartnb::dart_class<gui::InteractiveFrameDnD, gui::DragAndDrop>(
      m, "InteractiveFrameDnD")
      .def(
          RetainingInit<gui::Viewer*, gui::InteractiveFrame*>(),
          nb::arg("viewer"),
          nb::arg("frame"),
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>())
      .def(
          "getFrame",
          [dragOwners](const nb::object& self) {
            return ownedNative(dragEntityOwner(self, dragOwners));
          })
      .def("update", &gui::InteractiveFrameDnD::update)
      .def("move", &gui::InteractiveFrameDnD::move)
      .def("saveState", &gui::InteractiveFrameDnD::saveState);
  dartnb::dart_class<gui::BodyNodeDnD, gui::DragAndDrop>(m, "BodyNodeDnD")
      .def(
          RetainingInit<gui::Viewer*, dynamics::BodyNode*, bool, bool>(),
          nb::arg("viewer"),
          nb::arg("bn"),
          nb::arg("useExternalIK") = true,
          nb::arg("useWholeBody") = false,
          nb::keep_alive<1, 2>(),
          nb::keep_alive<1, 3>())
      .def(
          "getBodyNode",
          [dragOwners](const nb::object& self) {
            return ownedNative(dragEntityOwner(self, dragOwners));
          })
      .def("update", &gui::BodyNodeDnD::update)
      .def("move", &gui::BodyNodeDnD::move)
      .def("saveState", &gui::BodyNodeDnD::saveState)
      .def("release", &gui::BodyNodeDnD::release)
      .def("useExternalIK", &gui::BodyNodeDnD::useExternalIK)
      .def("isUsingExternalIK", &gui::BodyNodeDnD::isUsingExternalIK)
      .def("useWholeBody", &gui::BodyNodeDnD::useWholeBody)
      .def("isUsingWholeBody", &gui::BodyNodeDnD::isUsingWholeBody)
      .def(
          "setPreserveOrientationModKey",
          &gui::BodyNodeDnD::setPreserveOrientationModKey)
      .def(
          "getPreserveOrientationModKey",
          &gui::BodyNodeDnD::getPreserveOrientationModKey)
      .def(
          "setJointRestrictionModKey",
          &gui::BodyNodeDnD::setJointRestrictionModKey)
      .def(
          "getJointRestrictionModKey",
          &gui::BodyNodeDnD::getJointRestrictionModKey);
  return nb::make_tuple(cacheCapsule(dragOwners));
}

} // namespace python
} // namespace dart
