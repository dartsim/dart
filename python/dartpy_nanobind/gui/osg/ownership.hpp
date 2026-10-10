#pragma once

#include <dart/gui/osg/DefaultEventHandler.hpp>
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/RealTimeWorldNode.hpp>
#include <dart/gui/osg/ShapeFrameNode.hpp>
#include <dart/gui/osg/WorldNode.hpp>
#include <dart/gui/osg/detail/CameraModeCallback.hpp>

#include <dart/simulation/World.hpp>

#include <osg/ref_ptr>
#include <osgShadow/ShadowedScene>

namespace dartnb::gui {
struct Ownership
{
  ::osg::Referenced* owner;
  struct Edge
  {
    ::osg::Referenced* pointer;
    nb::object wrapper;
    std::function<void()> detach;
  };
  std::vector<Edge> active;
  std::vector<Edge> retired;
  explicit Ownership(::osg::Referenced* value) : owner(value) {}
  ~Ownership();
  void prune();
  void clear();
};
std::shared_ptr<Ownership> ownership(::osg::Referenced* owner);
void retainWrapper(
    ::osg::Referenced* owner,
    ::osg::Referenced* child,
    nb::object wrapper,
    std::function<void()> detach);
template <class T>
void retain(::osg::Referenced* owner, T* child, std::function<void()> detach)
{
  if (child)
    retainWrapper(
        owner,
        child,
        nb::cast(child, nb::rv_policy::reference),
        std::move(detach));
}
void retire(::osg::Referenced* owner, ::osg::Referenced* child);
int traverse(PyObject* self, visitproc visit, void* arg);
int clear(PyObject* self);
const PyType_Slot* slots();
const PyType_Slot* nodeSlots();

template <class T, class... Args>
std::shared_ptr<T> make(Args&&... args)
{
  auto* pointer = new T(std::forward<Args>(args)...);
  pointer->ref();
  auto state = ownership(pointer);
  return std::shared_ptr<T>(pointer, [state](T* value) { value->unref(); });
}

template <class... Args>
struct init : nb::def_visitor<init<Args...>>
{
  template <class Cls, class... Extra>
  void execute(Cls& cls, const Extra&... extra) const
  {
    using T = typename Cls::Type;
    if constexpr (std::is_same_v<T, typename Cls::Alias>)
      cls.def(
          dartnb::factory([](Args... args) { return make<T>(args...); }),
          extra...);
    else
      cls.def(dartnb::init<Args...>(), extra...);
  }
};
} // namespace dartnb::gui

namespace nanobind::detail {
template <class T>
struct type_caster<::osg::ref_ptr<T>>
{
  using Caster = make_caster<T>;
  static constexpr bool dart_osg_caster = true;
  NB_TYPE_CASTER(::osg::ref_ptr<T>, Caster::Name)
  bool from_python(handle src, uint32_t flags, cleanup_list* cleanup) noexcept
  {
    Caster caster;
    if (!caster.from_python(src, flags & ~cast_flags::convert, cleanup))
      return false;
    value = caster.operator T*();
    return true;
  }
  static handle from_cpp(
      const Value& value, rv_policy, cleanup_list* cleanup) noexcept
  {
    auto* pointer = value.get();
    if (!pointer)
      return nb::none().release();
    try {
      bool isNew = false;
      auto result = dartnb::wrap(
          typeid(T),
          typeid(*pointer),
          dartnb::complete_address(pointer),
          pointer,
          rv_policy::reference,
          cleanup ? cleanup->self() : handle(),
          &isNew);
      if (isNew) {
        auto* holder = new Value(value);
        nb::keep_alive_cb(result, holder, [](void* payload) noexcept {
          delete static_cast<Value*>(payload);
        });
      }
      return result;
    } catch (...) {
      return {};
    }
  }
};
} // namespace nanobind::detail

namespace dartnb {
template <>
inline std::vector<BaseEdge> extra_base_edges<osgViewer::View>()
{
  return {base_edge<osgViewer::View, osgGA::GUIActionAdapter>()};
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::osg::WorldNode>()
{
  return gui::nodeSlots();
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::osg::RealTimeWorldNode>()
{
  return gui::nodeSlots();
}
template <>
inline const PyType_Slot* gc_slots<osgViewer::View>()
{
  return gui::slots();
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::osg::Viewer>()
{
  return gui::slots();
}
template <>
inline const PyType_Slot* gc_slots<dart::gui::osg::ImGuiViewer>()
{
  return gui::slots();
}
static_assert(nb::detail::make_caster<
              ::osg::ref_ptr<::osg::Referenced>>::dart_osg_caster);
} // namespace dartnb
