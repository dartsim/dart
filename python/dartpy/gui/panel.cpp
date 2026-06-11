// Python bindings for DART's renderer-neutral GUI panel abstraction.

#include "gui/panel.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <array>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstdio>

namespace nb = nanobind;

namespace dart::python_nb {
namespace {

class PanelBuilderView
{
public:
  explicit PanelBuilderView(dart::gui::PanelBuilder* builder)
    : mBuilder(builder)
  {
  }

  dart::gui::PanelBuilder& builder() const
  {
    return *mBuilder;
  }

private:
  dart::gui::PanelBuilder* mBuilder = nullptr;
};

class PanelContextView
{
public:
  explicit PanelContextView(dart::gui::PanelContext* context)
    : mContext(context)
  {
  }

  const dart::gui::PanelContext& context() const
  {
    return *mContext;
  }

  dart::gui::PanelContext& context()
  {
    return *mContext;
  }

private:
  dart::gui::PanelContext* mContext = nullptr;
};

std::optional<std::array<double, 2>> optionalPair(nb::handle value)
{
  if (value.is_none()) {
    return std::nullopt;
  }

  const auto values = nb::cast<std::vector<double>>(value);
  if (values.size() != 2) {
    throw std::runtime_error("panel size/position must contain two values");
  }

  return std::array<double, 2>{values[0], values[1]};
}

std::optional<double> optionalDouble(nb::handle value)
{
  if (value.is_none()) {
    return std::nullopt;
  }

  return nb::cast<double>(value);
}

dart::gui::DockSide dockSideFromPython(nb::handle value)
{
  if (value.is_none()) {
    return dart::gui::DockSide::None;
  }

  const auto text = nb::cast<std::string>(value);
  if (text == "none" || text == "None" || text.empty()) {
    return dart::gui::DockSide::None;
  }
  if (text == "left" || text == "Left") {
    return dart::gui::DockSide::Left;
  }
  if (text == "right" || text == "Right") {
    return dart::gui::DockSide::Right;
  }
  if (text == "top" || text == "Top") {
    return dart::gui::DockSide::Top;
  }
  if (text == "bottom" || text == "Bottom") {
    return dart::gui::DockSide::Bottom;
  }
  if (text == "center" || text == "Center") {
    return dart::gui::DockSide::Center;
  }

  throw std::runtime_error("unknown panel dock side: " + text);
}

std::vector<std::string_view> toStringViews(
    const std::vector<std::string>& strings)
{
  std::vector<std::string_view> views;
  views.reserve(strings.size());
  for (const auto& value : strings) {
    views.emplace_back(value);
  }
  return views;
}

} // namespace

void defGuiPanels(nb::module_& m)
{
  nb::class_<PanelBuilderView>(m, "PanelBuilder")
      .def(
          "text",
          [](PanelBuilderView& self, const std::string& value) {
            self.builder().text(value);
          })
      .def(
          "separator",
          [](PanelBuilderView& self) { self.builder().separator(); })
      .def(
          "same_line",
          [](PanelBuilderView& self) { self.builder().sameLine(); })
      .def(
          "button",
          [](PanelBuilderView& self, const std::string& label) {
            return self.builder().button(label);
          },
          nb::arg("label"))
      .def(
          "selectable",
          [](PanelBuilderView& self, const std::string& label, bool selected) {
            return self.builder().selectable(label, selected);
          },
          nb::arg("label"),
          nb::arg("selected") = false)
      .def(
          "item_tooltip",
          [](PanelBuilderView& self, const std::string& text) {
            self.builder().itemTooltip(text);
          },
          nb::arg("text"))
      .def(
          "indent",
          [](PanelBuilderView& self, double width) {
            self.builder().indent(width);
          },
          nb::arg("width") = 12.0)
      .def(
          "unindent",
          [](PanelBuilderView& self, double width) {
            self.builder().unindent(width);
          },
          nb::arg("width") = 12.0)
      .def(
          "checkbox",
          [](PanelBuilderView& self, const std::string& label, bool value) {
            bool next = value;
            const bool changed = self.builder().checkbox(label, next);
            return nb::make_tuple(changed, next);
          },
          nb::arg("label"),
          nb::arg("value"))
      .def(
          "text_input",
          [](PanelBuilderView& self,
             const std::string& label,
             const std::string& value) {
            std::string next = value;
            const bool changed = self.builder().textInput(label, next);
            return nb::make_tuple(changed, next);
          },
          nb::arg("label"),
          nb::arg("value"))
      .def(
          "slider",
          [](PanelBuilderView& self,
             const std::string& label,
             double value,
             double minimum,
             double maximum) {
            double next = value;
            const bool changed
                = self.builder().slider(label, next, minimum, maximum);
            return nb::make_tuple(changed, next);
          },
          nb::arg("label"),
          nb::arg("value"),
          nb::arg("minimum"),
          nb::arg("maximum"))
      .def(
          "timeline",
          [](PanelBuilderView& self,
             const std::string& label,
             double value,
             double minimum,
             double maximum,
             const std::vector<double>& valueTrack,
             const std::vector<double>& markerTrack,
             const std::vector<double>& cursorTrack,
             const std::string& valueTrackLabel) {
            double next = value;
            const bool changed = self.builder().timeline(
                label,
                next,
                minimum,
                maximum,
                valueTrack,
                markerTrack,
                cursorTrack,
                valueTrackLabel);
            return nb::make_tuple(changed, next);
          },
          nb::arg("label"),
          nb::arg("value"),
          nb::arg("minimum"),
          nb::arg("maximum"),
          nb::arg("value_track") = std::vector<double>{},
          nb::arg("marker_track") = std::vector<double>{},
          nb::arg("cursor_track") = std::vector<double>{},
          nb::arg("value_track_label") = "Values")
      .def(
          "color_edit",
          [](PanelBuilderView& self,
             const std::string& label,
             const Eigen::Vector4d& rgba) {
            Eigen::Vector4d next = rgba;
            const bool changed = self.builder().colorEdit(label, next);
            return nb::make_tuple(changed, next);
          },
          nb::arg("label"),
          nb::arg("rgba"))
      .def(
          "select",
          [](PanelBuilderView& self,
             const std::string& label,
             int selectedIndex,
             const std::vector<std::string>& choices) {
            int next = selectedIndex;
            const auto views = toStringViews(choices);
            const bool changed = self.builder().select(label, next, views);
            return nb::make_tuple(changed, next);
          },
          nb::arg("label"),
          nb::arg("selected_index"),
          nb::arg("choices"))
      .def(
          "color_swatch",
          [](PanelBuilderView& self,
             const std::string& label,
             const Eigen::Vector4d& rgba) {
            self.builder().colorSwatch(label, rgba);
          },
          nb::arg("label"),
          nb::arg("rgba"))
      .def(
          "plot_lines",
          [](PanelBuilderView& self,
             const std::string& label,
             const std::vector<double>& values) {
            self.builder().plotLines(label, values);
          },
          nb::arg("label"),
          nb::arg("values"))
      .def(
          "collapsing_header",
          [](PanelBuilderView& self,
             const std::string& label,
             bool defaultOpen) {
            return self.builder().collapsingHeader(label, defaultOpen);
          },
          nb::arg("label"),
          nb::arg("default_open") = false)
      .def(
          "begin_table",
          [](PanelBuilderView& self,
             const std::string& label,
             const std::vector<std::string>& columns) {
            const auto views = toStringViews(columns);
            return self.builder().beginTable(label, views);
          },
          nb::arg("label"),
          nb::arg("columns"))
      .def(
          "table_next_row",
          [](PanelBuilderView& self) { self.builder().tableNextRow(); })
      .def(
          "table_next_column",
          [](PanelBuilderView& self) {
            return self.builder().tableNextColumn();
          })
      .def("end_table", [](PanelBuilderView& self) {
        self.builder().endTable();
      });

  nb::class_<PanelContextView>(m, "PanelContext")
      .def_prop_ro(
          "selected_label",
          [](const PanelContextView& self) {
            return self.context().selectedLabel;
          })
      .def_prop_ro(
          "selected_point",
          [](const PanelContextView& self) {
            if (!self.context().selectedPoint.has_value()) {
              return nb::none();
            }
            return nb::cast(*self.context().selectedPoint);
          })
      .def_prop_ro(
          "selected_normal",
          [](const PanelContextView& self) {
            if (!self.context().selectedNormal.has_value()) {
              return nb::none();
            }
            return nb::cast(*self.context().selectedNormal);
          })
      .def_prop_ro(
          "simulation_time",
          [](const PanelContextView& self) {
            return self.context().simulationTime;
          })
      .def_prop_ro(
          "contact_count",
          [](const PanelContextView& self) {
            return self.context().contactCount;
          })
      .def_prop_ro(
          "display_size",
          [](const PanelContextView& self) {
            return self.context().ui.displaySize;
          })
      .def_prop_ro(
          "font_size",
          [](const PanelContextView& self) {
            return self.context().ui.fontSize;
          })
      .def_prop_ro(
          "rendered_frames",
          [](const PanelContextView& self) {
            const auto* lifecycle = self.context().lifecycle;
            return lifecycle != nullptr ? lifecycle->renderedFrames : 0;
          })
      .def_prop_ro(
          "skipped_frames",
          [](const PanelContextView& self) {
            const auto* lifecycle = self.context().lifecycle;
            return lifecycle != nullptr ? lifecycle->skippedFrames : 0;
          })
      .def_prop_ro(
          "paused",
          [](const PanelContextView& self) {
            const auto* lifecycle = self.context().lifecycle;
            return lifecycle != nullptr && lifecycle->paused;
          })
      .def_prop_ro(
          "frame_output_enabled",
          [](const PanelContextView& self) {
            const auto* lifecycle = self.context().lifecycle;
            return lifecycle != nullptr && lifecycle->frameOutputEnabled;
          })
      .def(
          "set_paused",
          [](PanelContextView& self, bool paused) {
            auto* lifecycle = self.context().lifecycle;
            if (lifecycle != nullptr) {
              lifecycle->paused = paused;
            }
          },
          nb::arg("paused"))
      .def(
          "request_single_step",
          [](PanelContextView& self) {
            auto* lifecycle = self.context().lifecycle;
            if (lifecycle != nullptr) {
              dart::gui::requestSingleStep(*lifecycle);
            }
          })
      .def(
          "request_scene_switch",
          [](PanelContextView& self, const std::string& sceneId) {
            auto* lifecycle = self.context().lifecycle;
            if (lifecycle != nullptr) {
              dart::gui::requestSceneSwitch(*lifecycle, sceneId);
            }
          },
          nb::arg("scene_id"))
      .def(
          "request_scene_replay",
          [](PanelContextView& self, const std::string& sceneId) {
            auto* lifecycle = self.context().lifecycle;
            if (lifecycle != nullptr) {
              dart::gui::requestSceneReplay(*lifecycle, sceneId);
            }
          },
          nb::arg("scene_id"));
}

dart::gui::Panel makeGuiPanelFromPython(nb::handle panelLike)
{
  dart::gui::Panel panel;
  panel.title = nb::cast<std::string>(panelLike.attr("title"));
  panel.initialPosition = optionalPair(panelLike.attr("initial_position"));
  panel.initialSize = optionalPair(panelLike.attr("initial_size"));
  panel.backgroundAlpha = optionalDouble(panelLike.attr("background_alpha"));
  panel.autoResize = nb::cast<bool>(panelLike.attr("auto_resize"));
  panel.horizontalScrollbar
      = nb::cast<bool>(panelLike.attr("horizontal_scrollbar"));
  panel.menuBar = nb::cast<bool>(panelLike.attr("menu_bar"));
  panel.dockSide = dockSideFromPython(panelLike.attr("dock_side"));

  auto buildHolder = std::make_shared<nb::callable>(
      nb::cast<nb::callable>(panelLike.attr("build")));
  const std::string panelTitle = panel.title;
  panel.buildWithContext = [buildHolder, panelTitle](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    nb::gil_scoped_acquire gil;
    try {
      PanelBuilderView builderView(&builder);
      PanelContextView contextView(&context);
      (*buildHolder)(builderView, contextView);
    } catch (const std::exception& e) {
      std::fprintf(
          stderr,
          "py-demos panel '%s' error: %s\n",
          panelTitle.c_str(),
          e.what());
      builder.text("Panel callback failed; see stderr.");
    }
  };

  return panel;
}

} // namespace dart::python_nb
