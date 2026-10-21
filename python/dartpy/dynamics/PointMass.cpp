// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 * See LICENSE for the BSD-style license governing this file.
 */

#include "detail/eigen.hpp"

#include <dart/dynamics/PointMass.hpp>

#include <cmath>

namespace dart {
namespace python {
namespace {

nb::object readonlyCopy(const Eigen::Vector3d& value)
{
  nb::object result = nb::cast(value, nb::rv_policy::copy);
  result.attr("setflags")(nb::arg("write") = false);
  return result;
}

} // namespace

void PointMass(nb::module_& m)
{
  using Point = dart::dynamics::PointMass;

  dartnb::dart_class<Point>(m, "PointMass")
      .def_prop_rw(
          "mass",
          &Point::getMass,
          [](Point& self, double mass) {
            if (!std::isfinite(mass) || mass <= 0)
              throw nb::value_error("mass must be finite and positive");
            self.setMass(mass);
          })
      .def_prop_ro(
          "rest_position",
          [](const Point& self) {
            return readonlyCopy(self.getRestingPosition());
          })
      .def_prop_rw("is_colliding", &Point::isColliding, &Point::setColliding)
      .def_prop_rw(
          "positions",
          [](const Point& self) { return readonlyCopy(self.getPositions()); },
          &Point::setPositions)
      .def_prop_rw(
          "velocities",
          [](const Point& self) { return readonlyCopy(self.getVelocities()); },
          &Point::setVelocities)
      .def_prop_ro(
          "accelerations",
          [](const Point& self) {
            return readonlyCopy(self.getAccelerations());
          })
      .def_prop_ro(
          "world_position",
          [](const Point& self) {
            return readonlyCopy(self.getWorldPosition());
          })
      .def_prop_ro(
          "world_velocity",
          [](const Point& self) {
            return readonlyCopy(self.getWorldVelocity());
          })
      .def_prop_ro(
          "forces",
          [](const Point& self) { return readonlyCopy(self.getForces()); })
      .def("reset_positions", &Point::resetPositions)
      .def("reset_velocities", &Point::resetVelocities)
      .def("reset_accelerations", &Point::resetAccelerations)
      .def("reset_forces", &Point::resetForces);
}

} // namespace python
} // namespace dart
