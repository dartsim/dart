#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <dart/gui/GridVisual.hpp>
#include <dart/gui/PolyhedronVisual.hpp>
#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/Viewer.hpp>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <cstddef>

namespace nb = nanobind;

namespace {

Eigen::Vector3d toVec3(const nb::handle& h)
{
  try {
    return nb::cast<Eigen::Vector3d>(h);
  } catch (const nb::cast_error&) {
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) != 3)
      throw nb::type_error("Expected a length-3 sequence");
    Eigen::Vector3d vec;
    for (nb::ssize_t i = 0; i < 3; ++i)
      vec[i] = nb::cast<double>(seq[i]);
    return vec;
  }
}

} // namespace

namespace dart::python_nb {

void defViewerAttachment(nb::module_& m)
{
  nb::class_<dart::gui::ViewerAttachment>(m, "ViewerAttachment")
      .def("refresh", &dart::gui::ViewerAttachment::refresh);
}

void defGridVisual(nb::module_& m)
{
  using dart::gui::GridVisual;

  auto grid
      = nb::class_<GridVisual, dart::gui::ViewerAttachment>(m, "GridVisual")
            .def(nb::new_([]() { return makeOsgShared<GridVisual>(); }))
            .def("setNumCells", &GridVisual::setNumCells)
            .def("setMinorLineStepSize", &GridVisual::setMinorLineStepSize)
            .def(
                "setNumMinorLinesPerMajorLine",
                &GridVisual::setNumMinorLinesPerMajorLine)
            .def("setPlaneType", &GridVisual::setPlaneType)
            .def("getPlaneType", &GridVisual::getPlaneType)
            .def(
                "setOffset",
                [](GridVisual& self, const nb::handle& offset) {
                  self.setOffset(toVec3(offset));
                })
            .def("getOffset", &GridVisual::getOffset);

  nb::enum_<GridVisual::PlaneType>(grid, "PlaneType")
      .value("XY", GridVisual::PlaneType::XY)
      .value("YZ", GridVisual::PlaneType::YZ)
      .value("ZX", GridVisual::PlaneType::ZX);
}

void defPolyhedronVisual(nb::module_& m)
{
  using dart::gui::PolyhedronVisual;

  nb::class_<PolyhedronVisual, dart::gui::ViewerAttachment>(
      m, "PolyhedronVisual")
      .def(nb::new_([]() { return makeOsgShared<PolyhedronVisual>(); }))
      .def(
          "setVertices",
          [](PolyhedronVisual& self,
             const std::vector<Eigen::Vector3d>& vertices) {
            self.setVertices(vertices);
          })
      .def(
          "setVerticesMatrix",
          [](PolyhedronVisual& self,
             const Eigen::Ref<const Eigen::MatrixXd>& vertices) {
            self.setVertices(vertices);
          },
          nb::arg("vertices"))
      .def("getVertices", &PolyhedronVisual::getVertices)
      .def("clear", &PolyhedronVisual::clear)
      .def("setSurfaceColor", &PolyhedronVisual::setSurfaceColor)
      .def("getSurfaceColor", &PolyhedronVisual::getSurfaceColor)
      .def("setWireframeColor", &PolyhedronVisual::setWireframeColor)
      .def("getWireframeColor", &PolyhedronVisual::getWireframeColor)
      .def("setWireframeWidth", &PolyhedronVisual::setWireframeWidth)
      .def("getWireframeWidth", &PolyhedronVisual::getWireframeWidth)
      .def("display", &PolyhedronVisual::display)
      .def("isDisplayed", &PolyhedronVisual::isDisplayed)
      .def("displaySurface", &PolyhedronVisual::displaySurface)
      .def("isSurfaceDisplayed", &PolyhedronVisual::isSurfaceDisplayed)
      .def("displayWireframe", &PolyhedronVisual::displayWireframe)
      .def("isWireframeDisplayed", &PolyhedronVisual::isWireframeDisplayed);
}

} // namespace dart::python_nb
