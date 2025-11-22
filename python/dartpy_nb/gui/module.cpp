// Nanobind bindings for DART's OSG GUI.

#include "gui/module.hpp"

#include "gui/osg/osg.hpp"

#include <nanobind/nanobind.h>

namespace dart::python_nb {

void defGuiModule(nanobind::module_& m)
{
  std::fprintf(stderr, "[dartpy_nb][gui] gui_event_handler\\n");
  defGuiEventHandler(m);
  std::fprintf(stderr, "[dartpy_nb][gui] viewer_attachment\\n");
  defViewerAttachment(m);
  std::fprintf(stderr, "[dartpy_nb][gui] grid_visual\\n");
  defGridVisual(m);
  std::fprintf(stderr, "[dartpy_nb][gui] interactive_frame\\n");
  defInteractiveFrame(m);
  std::fprintf(stderr, "[dartpy_nb][gui] world_node\\n");
  defWorldNode(m);
  std::fprintf(stderr, "[dartpy_nb][gui] realtime_world_node\\n");
  defRealTimeWorldNode(m);
  std::fprintf(stderr, "[dartpy_nb][gui] viewer\\n");
  defViewer(m);
}

} // namespace dart::python_nb
