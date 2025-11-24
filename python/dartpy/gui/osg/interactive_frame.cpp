#include "common/type_casters.hpp"
#include "gui/osg/osg.hpp"

#include <dart/gui/osg/All.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defInteractiveFrame(nb::module_& m)
{
  using dart::gui::osg::InteractiveFrame;
  using dart::gui::osg::InteractiveTool;

  auto it
      = nb::class_<InteractiveTool, dart::dynamics::SimpleFrame>(
            m, "InteractiveTool")
            .def(
                nb::init<InteractiveFrame*, double, const std::string&>(),
                nb::arg("frame"),
                nb::arg("defaultAlpha"),
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
      .def(nb::init<dart::dynamics::Frame*>(), nb::arg("referenceFrame"))
      .def(
          nb::init<dart::dynamics::Frame*, const std::string&>(),
          nb::arg("referenceFrame"),
          nb::arg("name"))
      .def(
          nb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&>(),
          nb::arg("referenceFrame"),
          nb::arg("name"),
          nb::arg("relativeTransform"))
      .def(
          nb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double>(),
          nb::arg("referenceFrame"),
          nb::arg("name"),
          nb::arg("relativeTransform"),
          nb::arg("sizeScale"))
      .def(
          nb::init<
              dart::dynamics::Frame*,
              const std::string&,
              const Eigen::Isometry3d&,
              double,
              double>(),
          nb::arg("referenceFrame"),
          nb::arg("name"),
          nb::arg("relativeTransform"),
          nb::arg("sizeScale"),
          nb::arg("thicknessScale"))
      .def(
          "resizeStandardVisuals",
          [](InteractiveFrame& self) { self.resizeStandardVisuals(); })
      .def(
          "resizeStandardVisuals",
          [](InteractiveFrame& self, double sizeScale) {
            self.resizeStandardVisuals(sizeScale);
          },
          nb::arg("sizeScale"))
      .def(
          "resizeStandardVisuals",
          [](InteractiveFrame& self, double sizeScale, double thicknessScale) {
            self.resizeStandardVisuals(sizeScale, thicknessScale);
          },
          nb::arg("sizeScale"),
          nb::arg("thicknessScale"))
      .def(
          "getShapeFrames",
          [](InteractiveFrame& self) { return self.getShapeFrames(); })
      .def(
          "getShapeFrames",
          [](const InteractiveFrame& self) { return self.getShapeFrames(); })
      .def("removeAllShapeFrames", &InteractiveFrame::removeAllShapeFrames);
}

} // namespace dart::python_nb
