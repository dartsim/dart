// Nanobind bindings for DART's GUI.

#include "gui/module.hpp"

#include "gui/gui.hpp"

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defGuiModule(nanobind::module_& m)
{
  defGuiEventHandler(m);
  defViewerAttachment(m);
  defGridVisual(m);
  defInteractiveFrame(m);
  defWorldNode(m);
  defRealTimeWorldNode(m);
  defViewer(m);
  defImGuiApi(m);
  defImGuiWidget(m);
  defImGuiHandler(m);
  defImGuiViewer(m);
}

} // namespace dart::python_nb
