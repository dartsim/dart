#include "dynamics/frame.hpp"

#include "dart/dynamics/Frame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "common/type_casters.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defFrame(nb::module_& m)
{
  using Frame = dart::dynamics::Frame;

  nb::class_<Frame>(m, "Frame")
      .def(
          "getRelativeTransform",
          &Frame::getRelativeTransform,
          nb::rv_policy::reference_internal)
      .def(
          "getWorldTransform",
          &Frame::getWorldTransform,
          nb::rv_policy::reference_internal)
      .def(
          "getTransform", [](const Frame& self) { return self.getTransform(); })
      .def(
          "getTransform",
          [](const Frame& self, const Frame* withRespectTo) {
            const Frame* target
                = withRespectTo ? withRespectTo : Frame::World();
            return self.getTransform(target);
          },
          nb::arg("withRespectTo"))
      .def(
          "getTransform",
          [](const Frame& self,
             const Frame* withRespectTo,
             const Frame* inCoordinatesOf) {
            const Frame* wrt = withRespectTo ? withRespectTo : Frame::World();
            const Frame* coords
                = inCoordinatesOf ? inCoordinatesOf : Frame::World();
            return self.getTransform(wrt, coords);
          },
          nb::arg("withRespectTo"),
          nb::arg("inCoordinatesOf"))
      .def(
          "getParentFrame",
          [](Frame* self) -> Frame* { return self->getParentFrame(); },
          nb::rv_policy::reference_internal)
      .def(
          "descendsFrom",
          [](const Frame& self, const Frame* someFrame) {
            return self.descendsFrom(someFrame);
          },
          nb::arg("someFrame") = nullptr)
      .def("isShapeFrame", &Frame::isShapeFrame)
      .def("isWorld", &Frame::isWorld)
      .def_static("World", []() { return Frame::WorldShared(); });

  registerPolymorphicCaster<Frame, Frame>();
}

} // namespace dart::python_nb
