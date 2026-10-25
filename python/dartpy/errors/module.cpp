// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/eval.h>

#include <exception>
#include <stdexcept>
#include <string_view>
#include <typeinfo>

namespace dart::python {

void dart_errors(nb::module_& m)
{
  auto sm = m.def_submodule("errors", "Exception types raised by dartpy.");
  nb::exec(
      R"(
class WorldBusyError(RuntimeError):
    """Another thread is stepping the World."""
)",
      sm.attr("__dict__"));
  nb::register_exception_translator(
      [](const std::exception_ptr& error, void* worldBusyError) {
        try {
          std::rethrow_exception(error);
        } catch (const std::runtime_error& e) {
          // Only a plain std::runtime_error carries the busy rule; its
          // subclasses keep nanobind's mappings whatever their message.
          if (typeid(e) != typeid(std::runtime_error)
              || std::string_view(e.what()).rfind("WorldBusyError: ", 0) != 0)
            throw;
          PyErr_SetString(static_cast<PyObject*>(worldBusyError), e.what());
        }
      },
      // The translator keeps this reference for the life of the module.
      nb::object(sm.attr("WorldBusyError")).release().ptr());
}

} // namespace dart::python
