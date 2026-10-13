#pragma once

// Include after detail/dart_nb.hpp, as every binding translation unit does.
#include <dart/gui/app/App.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// dartpy.viewer keeps every Python reference where the garbage collector can
// see it: in a Python object that accepted it, or, while run() executes, in
// its Session. Host objects only ever hold weak handles to slots, so they can
// be destroyed with the GIL released and never form hidden cycles.
namespace dartnb::viewer {

/// A Python callable that host code may call.
struct Slot
{
  Slot(nb::object fn, std::string what);
  ~Slot();
  Slot(const Slot&) = delete;
  Slot& operator=(const Slot&) = delete;

  nb::object fn;
  std::string what;
};

/// A slot for `fn`, or none for None; raises TypeError unless `fn` is callable.
std::shared_ptr<Slot> makeSlot(nb::handle fn, std::string what);

/// The callable in `slot`, or None.
nb::object slotFunction(const std::shared_ptr<Slot>& slot);

/// The state of the run() in progress: the Python objects the installed scene
/// needs until its teardown, and the first BaseException, which stops the run.
struct Session
{
  nb::object scenes;
  nb::object activeSetup;
  nb::object activeContext;
  std::optional<nb::python_error> pending;
};

/// The session of the run() in progress, or null.
std::shared_ptr<Session>& currentSession();

/// Calls `slot` for the host. An ordinary exception becomes a C++ exception
/// for the host's guards, which log it and disable the callback; a
/// BaseException such as KeyboardInterrupt or SystemExit is stashed in the
/// session, which stops the run. Once one is stashed, only teardowns run.
void callSlot(const std::weak_ptr<Slot>& slot, bool teardown = false);

/// A host callback that calls `slot`, or an empty one for no slot.
std::function<void()> hostCallback(
    const std::shared_ptr<Slot>& slot, bool teardown = false);

/// Visits the callable in `slot` for the garbage collector.
int visitSlot(const std::shared_ptr<Slot>& slot, visitproc visit, void* arg);

/// tp_traverse and tp_clear for a bound struct with traverse() and clear().
template <class T>
const PyType_Slot* gcSlots()
{
  struct Functions
  {
    static int traverse(PyObject* self, visitproc visit, void* arg)
    {
      Py_VISIT(Py_TYPE(self));
      if (!nb::inst_ready(self))
        return 0;
      return nb::inst_ptr<T>(self)->traverse(visit, arg);
    }
    static int clear(PyObject* self)
    {
      if (nb::inst_ready(self))
        nb::inst_ptr<T>(self)->clear();
      return 0;
    }
  };
  static const PyType_Slot slots[]
      = {{Py_tp_traverse, reinterpret_cast<void*>(&Functions::traverse)},
         {Py_tp_clear, reinterpret_cast<void*>(&Functions::clear)},
         {0, nullptr}};
  return slots;
}

/// The Python face of a dart::gui::app::Control. It owns the change callback
/// and the wrappers of its children, so callbacks set on a child survive the
/// child's wrapper going out of scope in Python.
struct PyControl
{
  std::shared_ptr<dart::gui::app::Control> native;
  std::shared_ptr<Slot> onChange;
  std::vector<nb::object> children;

  int traverse(visitproc visit, void* arg) const;
  void clear();

  /// Reconnects the native callbacks of this tree to its Python callables.
  /// The host clears a panel's callbacks at teardown, so a panel that a
  /// factory returns again is rewired when its scene activates.
  void rewire();
};

void bindPanel(nb::module_& m);

} // namespace dartnb::viewer
