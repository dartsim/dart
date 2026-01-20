#include "gui/support_polygon_visual.hpp"

#include "common/type_casters.hpp"
#include "gui/utils.hpp"

#include <dart/gui/support_polygon_visual.hpp>

#include <dart/dynamics/skeleton.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <utility>

namespace nb = nanobind;

namespace dart::python_nb {

void defSupportPolygonVisual(nb::module_& m)
{
  using dart::gui::SupportPolygonVisual;

  nb::class_<SupportPolygonVisual, dart::gui::ViewerAttachment>(
      m, "SupportPolygonVisual")
      .def(
          nb::new_([](dart::dynamics::SkeletonPtr skeleton, double elevation) {
            return makeOsgShared<SupportPolygonVisual>(
                std::move(skeleton), elevation);
          }),
          nb::arg("skeleton") = nullptr,
          nb::arg("elevation") = 0.02)
      .def(
          nb::new_([](dart::dynamics::SkeletonPtr skeleton,
                      std::size_t treeIndex,
                      double elevation) {
            return makeOsgShared<SupportPolygonVisual>(
                std::move(skeleton), treeIndex, elevation);
          }),
          nb::arg("skeleton"),
          nb::arg("tree_index"),
          nb::arg("elevation") = 0.02)
      .def(
          "setSkeleton",
          &SupportPolygonVisual::setSkeleton,
          nb::arg("skeleton"))
      .def("getSkeleton", &SupportPolygonVisual::getSkeleton)
      .def(
          "visualizeWholeSkeleton",
          &SupportPolygonVisual::visualizeWholeSkeleton)
      .def(
          "visualizeTree",
          &SupportPolygonVisual::visualizeTree,
          nb::arg("tree_index"))
      .def(
          "setDisplayElevation",
          &SupportPolygonVisual::setDisplayElevation,
          nb::arg("elevation"))
      .def("getDisplayElevation", &SupportPolygonVisual::getDisplayElevation)
      .def(
          "displayPolygon",
          &SupportPolygonVisual::displayPolygon,
          nb::arg("display"))
      .def(
          "setPolygonColor",
          [](SupportPolygonVisual& self, const Eigen::Vector4d& color) {
            self.setPolygonColor(color);
          },
          nb::arg("color"))
      .def("getPolygonColor", &SupportPolygonVisual::getPolygonColor)
      .def(
          "displayCentroid",
          &SupportPolygonVisual::displayCentroid,
          nb::arg("display"))
      .def("isCentroidDisplayed", &SupportPolygonVisual::isCentroidDisplayed)
      .def(
          "setCentroidRadius",
          &SupportPolygonVisual::setCentroidRadius,
          nb::arg("radius"))
      .def("getCentroidRadius", &SupportPolygonVisual::getCentroidRadius)
      .def(
          "displayCenterOfMass",
          &SupportPolygonVisual::displayCenterOfMass,
          nb::arg("display"))
      .def(
          "isCenterOfMassDisplayed",
          &SupportPolygonVisual::isCenterOfMassDisplayed)
      .def(
          "setCenterOfMassRadius",
          &SupportPolygonVisual::setCenterOfMassRadius,
          nb::arg("radius"))
      .def(
          "getCenterOfMassRadius", &SupportPolygonVisual::getCenterOfMassRadius)
      .def(
          "setValidCOMColor",
          [](SupportPolygonVisual& self, const Eigen::Vector4d& color) {
            self.setValidCOMColor(color);
          },
          nb::arg("color"))
      .def(
          "getValidCOMColor",
          [](const SupportPolygonVisual& self) {
            return self.getValidCOMColor();
          })
      .def(
          "setInvalidCOMColor",
          [](SupportPolygonVisual& self, const Eigen::Vector4d& color) {
            self.setInvalidCOMColor(color);
          },
          nb::arg("color"))
      .def(
          "getInvalidCOMColor",
          [](const SupportPolygonVisual& self) {
            return self.getInvalidCOMColor();
          })
      .def("refresh", &SupportPolygonVisual::refresh);
}

} // namespace dart::python_nb
