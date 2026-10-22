// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/errors.hpp"

#include <nanobind/eval.h>

#include <exception>
#include <stdexcept>

namespace dartnb {
namespace {
PyObject* sOperationError = nullptr;
PyObject* sValidationError = nullptr;

// Runs before nanobind's default translator, which maps the standard
// exceptions to the builtins. nanobind handles python_error and its own
// builtin_exception types before any translator. Rethrowing passes an
// exception to the next translator, which keeps the deliberate builtin
// mappings; everything else derived from std::exception becomes a DartError.
void translate(const std::exception_ptr& error, void*)
{
  try {
    std::rethrow_exception(error);
  } catch (const std::bad_alloc&) {
    throw; // MemoryError
  } catch (const std::out_of_range&) {
    throw; // IndexError
  } catch (const std::domain_error&) {
    throw; // ValueError
  } catch (const std::invalid_argument& e) {
    PyErr_SetString(sValidationError, e.what());
  } catch (const std::length_error& e) {
    PyErr_SetString(sValidationError, e.what());
  } catch (const std::range_error&) {
    throw; // ValueError
  } catch (const std::overflow_error&) {
    throw; // OverflowError
  } catch (const std::exception& e) {
    // std::logic_error, std::runtime_error (including std::underflow_error),
    // and every other std::exception subclass.
    PyErr_SetString(sOperationError, e.what());
  } catch (...) {
    // Not a std::exception: leave it to the next translator. Returning here
    // would end the chain with no Python error set.
    throw;
  }
}
} // namespace

nb::handle operationError()
{
  return sOperationError;
}
} // namespace dartnb

namespace dart::python {

void dart_errors(nb::module_& m)
{
  auto sm = m.def_submodule("errors", "Exception types raised by dartpy.");
  nb::exec(
      R"(
class DartError(Exception):
    """Base class of the errors dartpy defines.

    Not every exception dartpy raises derives from it: argument conversion
    failures raise plain ``TypeError``, and C++ ``std::out_of_range``,
    ``std::domain_error``, ``std::range_error``, ``std::overflow_error`` and
    ``std::bad_alloc`` keep their ``IndexError``, ``ValueError``,
    ``OverflowError`` and ``MemoryError`` translations.
    """


class OperationError(DartError, RuntimeError):
    """An operation failed or ran in the wrong state."""


class ValidationError(DartError, ValueError):
    """An argument or input is invalid."""


class NotFoundError(DartError, KeyError):
    """A name lookup failed. The message lists the valid names."""

    def __str__(self):
        # KeyError quotes a lone message argument; keep it readable.
        return str(self.args[0]) if len(self.args) == 1 else super().__str__()


class WorldBusyError(OperationError):
    """Another thread is stepping the World."""


class ModelFrozenError(OperationError):
    """A structural edit on a compiled World outside ``world.edit()``."""


class CheckpointMismatchError(ValidationError):
    """A checkpoint does not match this World's topology, parameters, build
    or solver."""


class NameConflictError(ValidationError):
    """The name is already in use."""
)",
      sm.attr("__dict__"));
  // Module-lifetime references.
  dartnb::sOperationError
      = nb::object(sm.attr("OperationError")).release().ptr();
  dartnb::sValidationError
      = nb::object(sm.attr("ValidationError")).release().ptr();
  nb::register_exception_translator(&dartnb::translate);
}

} // namespace dart::python
