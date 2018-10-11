#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void dart_utils(pybind11::module& m)
{
  auto sm = m.def_submodule("utils");

  void DartLoader(pybind11::module& sm);
  DartLoader(sm);
}

} // namespace python
} // namespace dart
