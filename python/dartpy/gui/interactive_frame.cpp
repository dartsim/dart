#include "common/type_casters.hpp"
#include "gui/gui.hpp"

#include <dart/gui/interactive_frame.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defInteractiveFrame(nb::module_& m)
{
  using dart::gui::InteractiveFrame;
  using dart::gui::InteractiveTool;

  auto it
      = nb::class_<InteractiveTool, dart::dynamics::SimpleFrame>(
            m, "InteractiveTool")
            .def(
                nb::init<InteractiveFrame*, double, const std::string&>(),
                nb::arg("frame"),
                nb::arg("default_alpha"),
                nb::arg("name"))
            .def(
                "setEnabled",
                [](InteractiveTool& self, bool enabled) {
                  self.setEnabled(enabled);
                },
                nb::arg("enabled"))
            .def(
                "getEnabled",
                [](const InteractiveTool& self) { return self.getEnabled(); })
            .def(
                "setAlpha",
                [](InteractiveTool& self, double alpha) {
                  self.setAlpha(alpha);
                },
                nb::arg("alpha"))
            .def("resetAlpha", &InteractiveTool::resetAlpha)
            .def(
                "setDefaultAlpha",
                [](InteractiveTool& self, double alpha) {
                  self.setDefaultAlpha(alpha);
                },
                nb::arg("alpha"))
            .def(
                "setDefaultAlpha",
                [](InteractiveTool& self, double alpha, bool reset) {
                  self.setDefaultAlpha(alpha, reset);
                },
                nb::arg("alpha"),
                nb::arg("reset"))
            .def(
                "getDefaultAlpha",
                [](const InteractiveTool& self) {
                  return self.getDefaultAlpha();
                })
            .def(
                "getShapeFrames",
                [](InteractiveTool& self) { return self.getShapeFrames(); })
            .def(
                "getShapeFrames",
                [](const InteractiveTool& self) {
                  return self.getShapeFrames();
                })
            .def(
                "removeAllShapeFrames", &InteractiveTool::removeAllShapeFrames);

  nb::enum_<InteractiveTool::Type>(it, "Type")
      .value("LINEAR", InteractiveTool::Type::LINEAR)
      .value("ANGULAR", InteractiveTool::Type::ANGULAR)
      .value("PLANAR", InteractiveTool::Type::PLANAR)
      .value("NUM_TYPES", InteractiveTool::Type::NUM_TYPES);

  nb::class_<InteractiveFrame, dart::dynamics::SimpleFrame>(
      m, "InteractiveFrame")
      .def(nb::init<dart::dynamics::Frame*>(), nb::arg("reference_frame"))
      .def(
          nb::init<dart::dynamics::Frame*, const std::string&>(),
          nb::arg("reference_frame"),
          nb::arg("name"))
      .def(
          nb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&>(),
          nb::arg("reference_frame"),
          nb::arg("name"),
          nb::arg("relative_transform"))
      .def(
          nb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double>(),
          nb::arg("reference_frame"),
          nb::arg("name"),
          nb::arg("relative_transform"),
          nb::arg("size_scale"))
      .def(
          nb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double,
              double>(),
          nb::arg("reference_frame"),
          nb::arg("name"),
          nb::arg("relative_transform"),
          nb::arg("size_scale"),
          nb::arg("thickness_scale"))
      .def(
          "resizeStandardVisuals",
          [](InteractiveFrame& self) { self.resizeStandardVisuals(); })
      .def(
          "resizeStandardVisuals",
          [](InteractiveFrame& self, double sizeScale) {
            self.resizeStandardVisuals(sizeScale);
          },
          nb::arg("size_scale"))
      .def(
          "resizeStandardVisuals",
          [](InteractiveFrame& self, double sizeScale, double thicknessScale) {
            self.resizeStandardVisuals(sizeScale, thicknessScale);
          },
          nb::arg("size_scale"),
          nb::arg("thickness_scale"))
      .def(
          "getShapeFrames",
          [](InteractiveFrame& self) { return self.getShapeFrames(); })
      .def(
          "getShapeFrames",
          [](const InteractiveFrame& self) { return self.getShapeFrames(); })
      .def("removeAllShapeFrames", &InteractiveFrame::removeAllShapeFrames);
}

} // namespace dart::python_nb
