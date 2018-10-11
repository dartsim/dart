#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void dart_dynamics(pybind11::module& m)
{
  auto sm = m.def_submodule("dynamics");

  void Skeleton(pybind11::module& sm);
  Skeleton(sm);
}

} // namespace python
} // namespace dart
