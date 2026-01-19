#include "simulation_experimental/frame.hpp"

#include "dart/simulation/experimental/frame/fixed_frame.hpp"
#include "dart/simulation/experimental/frame/frame.hpp"
#include "dart/simulation/experimental/frame/free_frame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defExpFrame(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<Frame>(m, "Frame")
      .def("getLocalTransform", &Frame::getLocalTransform)
      .def("getParentFrame", &Frame::getParentFrame)
      .def("setParentFrame", &Frame::setParentFrame, nb::arg("parent"))
      .def(
          "getTransform", nb::overload_cast<>(&Frame::getTransform, nb::const_))
      .def(
          "getTransform",
          nb::overload_cast<const Frame&>(&Frame::getTransform, nb::const_),
          nb::arg("relative_to"))
      .def(
          "getTransform",
          nb::overload_cast<const Frame&, const Frame&>(
              &Frame::getTransform, nb::const_),
          nb::arg("relative_to"),
          nb::arg("expressed_in"))
      .def("getTranslation", &Frame::getTranslation)
      .def("getRotation", &Frame::getRotation)
      .def("getQuaternion", &Frame::getQuaternion)
      .def("getTransformMatrix", &Frame::getTransformMatrix)
      .def_static("world", &Frame::world)
      .def("isValid", &Frame::isValid)
      .def("isWorld", &Frame::isWorld)
      .def("isSameInstanceAs", &Frame::isSameInstanceAs, nb::arg("other"))
      .def("__eq__", &Frame::operator==)
      .def("__ne__", &Frame::operator!=);

  nb::class_<FreeFrame, Frame>(m, "FreeFrame")
      .def("setLocalTransform", &FreeFrame::setLocalTransform, nb::arg("tf"))
      .def("getLocalTransform", &FreeFrame::getLocalTransform);

  nb::class_<FixedFrame, Frame>(m, "FixedFrame")
      .def("setLocalTransform", &FixedFrame::setLocalTransform, nb::arg("tf"))
      .def("getLocalTransform", &FixedFrame::getLocalTransform);
}

} // namespace dart::python_nb
