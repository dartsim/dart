#include "dynamics/ball_joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/dynamics/BallJoint.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defBallJoint(nb::module_& m)
{
  using BallJoint = dart::dynamics::BallJoint;

  nb::class_<BallJoint, dart::dynamics::Joint, std::shared_ptr<BallJoint>>(m, "BallJoint")
      .def_static("convertToPositions", &BallJoint::convertToPositions, nb::arg("transform"))
      .def_static("convertToRotation", &BallJoint::convertToRotation, nb::arg("positions"))
      .def_static("convertToTransform", &BallJoint::convertToTransform, nb::arg("positions"));
}

} // namespace dart::python_nb
