#include "dynamics/shape.hpp"

#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defShape(nb::module_& m)
{
  using Shape = dart::dynamics::Shape;

  nb::class_<Shape>(m, "Shape")
      .def(
          "getType",
          [](const Shape& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("computeInertia", &Shape::computeInertia, nb::arg("mass"));

  nb::class_<dart::dynamics::SphereShape, Shape>(m, "SphereShape")
      .def(nb::init<double>(), nb::arg("radius"))
      .def(
          "setRadius",
          &dart::dynamics::SphereShape::setRadius,
          nb::arg("radius"))
      .def("getRadius", &dart::dynamics::SphereShape::getRadius)
      .def(
          "computeInertia",
          [](const dart::dynamics::SphereShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          []() -> const std::string& {
            return dart::dynamics::SphereShape::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          [](double radius) {
            return dart::dynamics::SphereShape::computeVolume(radius);
          },
          nb::arg("radius"))
      .def_static(
          "computeInertiaOf",
          [](double radius, double mass) {
            return dart::dynamics::SphereShape::computeInertia(radius, mass);
          },
          nb::arg("radius"),
          nb::arg("mass"));

  nb::class_<dart::dynamics::BoxShape, Shape>(m, "BoxShape")
      .def(nb::init<const Eigen::Vector3d&>(), nb::arg("size"))
      .def("setSize", &dart::dynamics::BoxShape::setSize, nb::arg("size"))
      .def(
          "getSize",
          [](const dart::dynamics::BoxShape& self) { return self.getSize(); })
      .def(
          "computeInertia",
          [](const dart::dynamics::BoxShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"))
      .def_static(
          "computeVolumeOf",
          [](const Eigen::Vector3d& size) {
            return dart::dynamics::BoxShape::computeVolume(size);
          },
          nb::arg("size"))
      .def_static(
          "computeInertiaOf",
          [](const Eigen::Vector3d& size, double mass) {
            return dart::dynamics::BoxShape::computeInertia(size, mass);
          },
          nb::arg("size"),
          nb::arg("mass"));
}

} // namespace dart::python_nb
