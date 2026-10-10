#include "probe.hpp"

#include <osg/ref_ptr>
#include <pybind11/pybind11.h>

PYBIND11_DECLARE_HOLDER_TYPE(T, ::osg::ref_ptr<T>, true);

namespace py = pybind11;

PYBIND11_MODULE(_dartpy_gui_probe, m)
{
  namespace probe = dart::python::gui_probe;
  m.def("accept_action_adapter", [](osgGA::GUIActionAdapter* action) {
    return dynamic_cast<dart::gui::osg::Viewer*>(action) != nullptr;
  });
  m.def("base_view", [](dart::gui::osg::Viewer* viewer) -> osgViewer::View* { return viewer; });
  m.def("base_subject", [](dart::gui::osg::Viewer* viewer) -> dart::common::Subject* { return viewer; });
  m.def("refresh", &probe::refresh);
  m.def("refresh_viewer", &probe::refreshViewer);
  m.def("shadow_ref_count", [](osgShadow::ShadowTechnique* technique) {
    return technique->referenceCount();
  });
  m.def(
      "handle",
      &probe::handle,
      py::arg("handler"),
      py::arg("viewer"),
      py::arg("key") = 65);
  m.def(
      "dispatch_viewer_handlers",
      &probe::dispatchViewerHandlers,
      py::arg("viewer"),
      py::arg("key") = 65);
  m.def(
      "remove_handler",
      [](osgViewer::View* viewer, osgGA::GUIEventHandler* handler) {
        viewer->removeEventHandler(handler);
      });
}
