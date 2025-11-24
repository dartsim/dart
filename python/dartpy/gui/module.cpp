// Nanobind bindings for DART's OSG GUI.

#include "gui/module.hpp"

#include "gui/osg/osg.hpp"

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
}

} // namespace dart::python_nb
