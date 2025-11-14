#include "dynamics/joint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "dart/dynamics/Joint.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defJoint(nb::module_& m)
{
  using Joint = dart::dynamics::Joint;

  nb::class_<Joint, std::shared_ptr<Joint>>(m, "Joint")
      .def("getName",
          [](const Joint& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def("setName", &Joint::setName, nb::arg("name"))
      .def("getType",
          [](const Joint& self) -> const std::string& {
            return self.getType();
          },
          nb::rv_policy::reference_internal)
      .def("getNumDofs", &Joint::getNumDofs)
      .def("setPositions",
          [](Joint& self, const Eigen::VectorXd& positions) {
            self.setPositions(positions);
          },
          nb::arg("positions"))
      .def("getPositions",
          [](const Joint& self) {
            return self.getPositions();
          })
      .def("setVelocities",
          [](Joint& self, const Eigen::VectorXd& velocities) {
            self.setVelocities(velocities);
          },
          nb::arg("velocities"))
      .def("getVelocities",
          [](const Joint& self) {
            return self.getVelocities();
          })
      .def("getRelativeTransform", &Joint::getRelativeTransform)
      .def("setRelativeTransform",
          [](Joint& self, const Eigen::Isometry3d& tf) {
            self.setRelativeTransform(tf);
          },
          nb::arg("transform"))
      .def("getRelativeJacobian",
          [](Joint& self, const Eigen::VectorXd& positions) {
            return self.getRelativeJacobian(positions);
          },
          nb::arg("positions"))
      .def("getRelativeJacobian", [](Joint& self) { return self.getRelativeJacobian(); })
      .def("getRelativeJacobianTimeDeriv", &Joint::getRelativeJacobianTimeDeriv)
      .def("getTransformFromChildBodyNode", &Joint::getTransformFromChildBodyNode)
      .def("getTransformFromParentBodyNode", &Joint::getTransformFromParentBodyNode)
      .def("setTransformFromChildBodyNode", &Joint::setTransformFromChildBodyNode, nb::arg("transform"))
      .def("setTransformFromParentBodyNode", &Joint::setTransformFromParentBodyNode, nb::arg("transform"))
      .def("getWrenchToParentBodyNode",
          [](Joint& self, const dart::dynamics::Frame& frame) {
            return self.getWrenchToParentBodyNode(frame);
          },
          nb::arg("frame"))
      .def("getWrenchToChildBodyNode",
          [](Joint& self, const dart::dynamics::Frame& frame) {
            return self.getWrenchToChildBodyNode(frame);
          },
          nb::arg("frame"))
      .def("setLimitEnforcement", &Joint::setLimitEnforcement, nb::arg("enforce"));
}

} // namespace dart::python_nb
