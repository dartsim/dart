#pragma once

#include <dart/gui/osg/DragAndDrop.hpp>

#include <dart/common/Observer.hpp>
#include <dart/common/sub_ptr.hpp>

namespace dartnb::gui {
// Subject notifications cover disable(), entity destruction, and delete this.
class DndWatch : public dart::common::Observer
{
public:
  nb::object wrapper;
  DndWatch(dart::gui::osg::DragAndDrop* value, nb::handle object)
    : wrapper(nb::steal(PyWeakref_NewRef(object.ptr(), nullptr)))
  {
    addSubject(value);
    addSubject(value->getEntity());
  }
  void handleDestructionNotification(const dart::common::Subject*) override
  {
    nb::gil_scoped_acquire guard;
    auto object = wrapper();
    if (!object.is_none())
      nb::inst_set_state(object, false, false);
  }
};
template <class T>
nb::typed<nb::object, T> watchDnd(T* pointer, nb::handle parent)
{
  if (!pointer)
    return nb::borrow<nb::typed<nb::object, T>>(nb::none());
  bool isNew = false;
  auto result = nb::steal(dartnb::wrap(
      typeid(T),
      typeid(*pointer),
      complete_address(pointer),
      pointer,
      nb::rv_policy::reference_internal,
      parent,
      &isNew));
  if (isNew) {
    auto* watch = new DndWatch(pointer, result);
    nb::keep_alive_cb(result, watch, [](void* payload) noexcept {
      delete static_cast<DndWatch*>(payload);
    });
  }
  return nb::borrow<nb::typed<nb::object, T>>(result);
}

template <class... Args>
struct dnd_init : nb::def_visitor<dnd_init<Args...>>
{
  template <class Cls, class... Extra>
  void execute(Cls& cls, const Extra&... extra) const
  {
    using T = typename Cls::Type;
    auto prepare = [](Args... args) {
      auto* pointer = new T(args...);
      auto lifetime = std::make_shared<dart::sub_ptr<T>>(pointer);
      auto owner = std::shared_ptr<T>(pointer, [lifetime](T* value) {
        if (lifetime->get())
          delete value;
      });
      auto result = nativeInstance(nb::type<T>(), owner);
      auto* watch = new DndWatch(pointer, result);
      nb::keep_alive_cb(result, watch, [](void* payload) noexcept {
        delete static_cast<DndWatch*>(payload);
      });
      return result;
    };
    defHybridNew<T>(cls, prepare, extra...);
    cls.def(
        "__init__",
        [prepare](nb::handle self, Args... args) {
          if (nb::inst_ready(self))
            return;
          auto prepared = prepare(args...);
          attachFactoryInstance(self, prepared);
          nb::inst_set_state(self, true, false);
          auto* pointer = nb::inst_ptr<T>(self);
          remember_wrapper(typeid(T), complete_address(pointer), self);
          auto* watch = new DndWatch(pointer, self);
          nb::keep_alive_cb(self, watch, [](void* payload) noexcept {
            delete static_cast<DndWatch*>(payload);
          });
        },
        extra...);
  }
};
} // namespace dartnb::gui
