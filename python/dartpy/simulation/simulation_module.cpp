#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void dart_simulation(pybind11::module& m)
{
  auto sm = m.def_submodule("simulation");

  void dart_simulation_World(pybind11::module& sm);
  dart_simulation_World(sm);
}

} // namespace python
} // namespace dart
