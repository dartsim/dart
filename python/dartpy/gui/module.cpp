// Nanobind bindings for DART's GUI.

#include "gui/module.hpp"

#if defined(DARTPY_HAS_GUI)
  #include "gui/descriptors.hpp"
  #include "gui/offscreen.hpp"
  #include "gui/panel.hpp"
  #include "gui/viewer.hpp"
#endif

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defGuiModule(nanobind::module_& m)
{
#if defined(DARTPY_HAS_GUI)
  defGuiDescriptors(m);
  defGuiOffscreen(m);
  defGuiPanels(m);
  defGuiViewer(m);
#endif
}

} // namespace dart::python_nb
