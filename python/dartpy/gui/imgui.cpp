#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <dart/gui/ImGuiHandler.hpp>
#include <dart/gui/ImGuiViewer.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/Utils.hpp>
#include <dart/gui/Viewer.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

class PyImGuiWidget : public dart::gui::ImGuiWidget
{
public:
  NB_TRAMPOLINE(dart::gui::ImGuiWidget, 1);

  void render() override
  {
    NB_OVERRIDE_PURE(render);
  }
};

} // namespace

void defImGuiWidget(nb::module_& m)
{
  nb::class_<
      dart::gui::ImGuiWidget,
      PyImGuiWidget,
      std::shared_ptr<dart::gui::ImGuiWidget>>(m, "ImGuiWidget")
      .def(nb::init<>())
      .def("render", &dart::gui::ImGuiWidget::render)
      .def(
          "setVisible",
          [](dart::gui::ImGuiWidget& self, bool visible) {
            self.setVisible(visible);
          },
          nb::arg("visible"))
      .def("toggleVisible", &dart::gui::ImGuiWidget::toggleVisible)
      .def("show", &dart::gui::ImGuiWidget::show)
      .def("hide", &dart::gui::ImGuiWidget::hide)
      .def("isVisible", &dart::gui::ImGuiWidget::isVisible);
}

void defImGuiHandler(nb::module_& m)
{
  using dart::gui::ImGuiHandler;

  nb::class_<ImGuiHandler>(m, "ImGuiHandler")
      .def(nb::init<>())
      .def(
          "newFrame",
          [](ImGuiHandler& self, osg::RenderInfo& info) {
            self.newFrame(info);
          },
          nb::arg("renderInfo"))
      .def(
          "render",
          [](ImGuiHandler& self, osg::RenderInfo& info) { self.render(info); },
          nb::arg("renderInfo"))
      .def(
          "setCameraCallbacks",
          [](ImGuiHandler& self, osg::Camera* camera) {
            self.setCameraCallbacks(camera);
          },
          nb::arg("camera"))
      .def(
          "hasWidget",
          [](const ImGuiHandler& self,
             const std::shared_ptr<dart::gui::ImGuiWidget>& widget) {
            return self.hasWidget(widget);
          },
          nb::arg("widget"))
      .def(
          "addWidget",
          [](ImGuiHandler& self,
             const std::shared_ptr<dart::gui::ImGuiWidget>& widget) {
            self.addWidget(widget);
          },
          nb::arg("widget"))
      .def(
          "addWidget",
          [](ImGuiHandler& self,
             const std::shared_ptr<dart::gui::ImGuiWidget>& widget,
             bool visible) { self.addWidget(widget, visible); },
          nb::arg("widget"),
          nb::arg("visible"))
      .def(
          "removeWidget",
          [](ImGuiHandler& self,
             const std::shared_ptr<dart::gui::ImGuiWidget>& widget) {
            self.removeWidget(widget);
          },
          nb::arg("widget"))
      .def("removeAllWidget", &ImGuiHandler::removeAllWidget)
      .def(
          "handle",
          [](ImGuiHandler& self,
             const osgGA::GUIEventAdapter& eventAdapter,
             osgGA::GUIActionAdapter& actionAdapter,
             osg::Object* object,
             osg::NodeVisitor* nodeVisitor) {
            return self.handle(
                eventAdapter, actionAdapter, object, nodeVisitor);
          },
          nb::arg("eventAdapter"),
          nb::arg("actionAdapter"),
          nb::arg("object"),
          nb::arg("nodeVisitor"));
}

void defImGuiViewer(nb::module_& m)
{
  using dart::gui::ImGuiViewer;

  nb::class_<ImGuiViewer, dart::gui::Viewer>(m, "ImGuiViewer")
      .def(nb::init<>())
      .def(
          "__init__",
          [](ImGuiViewer* self, const Eigen::Vector4d& clearColor) {
            new (self) ImGuiViewer(dart::gui::eigToOsgVec4f(clearColor));
          },
          nb::arg("clearColor"))
      .def(nb::init<const osg::Vec4&>(), nb::arg("clearColor"))
      .def(
          "getImGuiHandler",
          [](ImGuiViewer& self) { return self.getImGuiHandler(); },
          nb::rv_policy::reference_internal)
      .def("showAbout", &ImGuiViewer::showAbout)
      .def("hideAbout", &ImGuiViewer::hideAbout);
}

} // namespace dart::python_nb
