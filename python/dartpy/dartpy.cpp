#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

PYBIND11_MODULE(dartpy, m)
{
  py::module::import("numpy");

  m.doc() = "DART python bindings";

  void dart_common(pybind11::module& m);
  dart_common(m);

  void dart_dynamics(pybind11::module& m);
  dart_dynamics(m);

  void dart_simulation(pybind11::module& m);
  dart_simulation(m);

  void dart_utils(pybind11::module& m);
  dart_utils(m);
}

} // namespace python
} // namespace dart
