#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void dart_common(pybind11::module& m)
{
  auto sm = m.def_submodule("common");

  void Uri(pybind11::module& sm);
  Uri(sm);
}

} // namespace python
} // namespace dart
