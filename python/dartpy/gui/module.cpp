// Nanobind bindings for DART's GUI.

#include "gui/module.hpp"

#if defined(DARTPY_HAS_EXPERIMENTAL_GUI)
  #include "gui/experimental.hpp"
#endif

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defGuiModule(nanobind::module_& m)
{
#if defined(DARTPY_HAS_EXPERIMENTAL_GUI)
  auto experimental = m.def_submodule(
      "experimental", "Compatibility namespace for promoted GUI descriptors");
  defGuiExperimentalModule(experimental);
#endif
}

} // namespace dart::python_nb
