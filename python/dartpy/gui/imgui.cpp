#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/im_gui_viewer.hpp>
#include <dart/gui/im_gui_widget.hpp>
#include <dart/gui/include_im_gui.hpp>
#include <dart/gui/utils.hpp>
#include <dart/gui/viewer.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

#include <memory>
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
  nb::class_<dart::gui::ImGuiWidget, PyImGuiWidget>(m, "ImGuiWidget")
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

void defImGuiApi(nb::module_& m)
{
  auto imgui = m.def_submodule("imgui", "Bindings for Dear ImGui (DART).");

  imgui.attr("FIRST_USE_EVER") = static_cast<int>(ImGuiCond_FirstUseEver);

  // ImGuiKey is a typedef to int in some ImGui releases; use a shim enum for
  // nanobind's enum bindings.
  enum class ImGuiKeyShim : int
  {
    Escape = ImGuiKey_Escape
  };
  nb::enum_<ImGuiKeyShim>(imgui, "Key").value("Escape", ImGuiKeyShim::Escape);
  imgui.def(
      "is_key_down",
      [](ImGuiKeyShim key) {
        return ImGui::IsKeyDown(static_cast<ImGuiKey>(key));
      },
      nb::arg("key"));

  nb::class_<ImGuiIO>(imgui, "IO")
      .def_prop_ro("framerate", [](const ImGuiIO& io) { return io.Framerate; });

  imgui.def(
      "get_io",
      []() -> ImGuiIO& { return ImGui::GetIO(); },
      nb::rv_policy::reference);
  imgui.def(
      "set_next_window_size",
      [](float width, float height, int cond) {
        ImGui::SetNextWindowSize(
            ImVec2(width, height), static_cast<ImGuiCond>(cond));
      },
      nb::arg("width"),
      nb::arg("height"),
      nb::arg("cond") = static_cast<int>(ImGuiCond_Always));
  imgui.def(
      "set_next_window_bg_alpha",
      [](float alpha) { ImGui::SetNextWindowBgAlpha(alpha); },
      nb::arg("alpha"));
  imgui.def(
      "begin",
      [](const std::string& name, int flags) {
        return ImGui::Begin(
            name.c_str(), nullptr, static_cast<ImGuiWindowFlags>(flags));
      },
      nb::arg("name"),
      nb::arg("flags") = 0);
  imgui.def("end", []() { ImGui::End(); });
  imgui.def(
      "text",
      [](const std::string& text) { ImGui::TextUnformatted(text.c_str()); },
      nb::arg("text"));
  imgui.def(
      "button",
      [](const std::string& label) { return ImGui::Button(label.c_str()); },
      nb::arg("label"));
  imgui.def("same_line", []() { ImGui::SameLine(); });
  imgui.def("separator", []() { ImGui::Separator(); });
  imgui.def(
      "slider_float",
      [](const std::string& label, float value, float min, float max) {
        float current = value;
        const bool changed
            = ImGui::SliderFloat(label.c_str(), &current, min, max);
        return std::make_pair(changed, current);
      },
      nb::arg("label"),
      nb::arg("value"),
      nb::arg("min"),
      nb::arg("max"));
  imgui.def(
      "is_key_down",
      [](int key) { return ImGui::IsKeyDown(static_cast<ImGuiKey>(key)); },
      nb::arg("key"));
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
          nb::arg("render_info"))
      .def(
          "render",
          [](ImGuiHandler& self, osg::RenderInfo& info) { self.render(info); },
          nb::arg("render_info"))
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
          nb::arg("event_adapter"),
          nb::arg("action_adapter"),
          nb::arg("object"),
          nb::arg("node_visitor"));
}

void defImGuiViewer(nb::module_& m)
{
  using dart::gui::ImGuiViewer;

  nb::class_<ImGuiViewer, dart::gui::Viewer>(m, "ImGuiViewer")
      .def(nb::new_([]() { return makeOsgShared<ImGuiViewer>(); }))
      .def(
          nb::new_([](const Eigen::Vector4d& clearColor) {
            return makeOsgShared<ImGuiViewer>(
                dart::gui::eigToOsgVec4f(clearColor));
          }),
          nb::arg("clear_color"))
      .def(
          nb::new_([](const osg::Vec4& clearColor) {
            return makeOsgShared<ImGuiViewer>(clearColor);
          }),
          nb::arg("clear_color"))
      .def(
          "getImGuiHandler",
          [](ImGuiViewer& self) { return self.getImGuiHandler(); },
          nb::rv_policy::reference_internal)
      .def("showAbout", &ImGuiViewer::showAbout)
      .def("hideAbout", &ImGuiViewer::hideAbout)
      .def("run", [](ImGuiViewer& self) {
        int result = 0;
        {
          nb::gil_scoped_release release;
          result = self.run();
        }
        self.getImGuiHandler()->removeAllWidget();
        return result;
      });
}

} // namespace dart::python_nb
