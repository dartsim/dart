#include "dynamics/ball_joint.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/ball_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defBallJoint(nb::module_& m)
{
  using BallJoint = dart::dynamics::BallJoint;

  nb::class_<BallJoint, dart::dynamics::Joint>(m, "BallJoint")
      .def_static(
          "convertToPositions",
          [](const Eigen::Matrix3d& transform) {
            return BallJoint::convertToPositions(transform);
          },
          nb::arg("transform"))
      .def_static(
          "convertToRotation",
          [](const Eigen::Vector3d& positions) {
            return BallJoint::convertToRotation(positions);
          },
          nb::arg("positions"))
      .def_static(
          "convertToTransform",
          [](const Eigen::Vector3d& positions) {
            return BallJoint::convertToTransform(positions);
          },
          nb::arg("positions"));

  registerPolymorphicCaster<dart::dynamics::Joint, BallJoint>();
}

} // namespace dart::python_nb
