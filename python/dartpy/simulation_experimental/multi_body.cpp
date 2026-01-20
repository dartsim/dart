#include "simulation_experimental/multi_body.hpp"

#include "dart/simulation/experimental/multi_body/joint.hpp"
#include "dart/simulation/experimental/multi_body/link.hpp"
#include "dart/simulation/experimental/multi_body/multi_body.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defExpJoint(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<Joint>(m, "Joint")
      .def("getName", &Joint::getName)
      .def("getType", &Joint::getType)
      .def("getAxis", &Joint::getAxis)
      .def("getAxis2", &Joint::getAxis2)
      .def("getPitch", &Joint::getPitch)
      .def("getParentLink", &Joint::getParentLink)
      .def("getChildLink", &Joint::getChildLink)
      .def("isValid", &Joint::isValid)
      .def("getDOF", &Joint::getDOF)
      .def("getPosition", &Joint::getPosition)
      .def("setPosition", &Joint::setPosition, nb::arg("position"))
      .def("getVelocity", &Joint::getVelocity)
      .def("setVelocity", &Joint::setVelocity, nb::arg("velocity"))
      .def("getAcceleration", &Joint::getAcceleration)
      .def("setAcceleration", &Joint::setAcceleration, nb::arg("acceleration"))
      .def("getTorque", &Joint::getTorque)
      .def("setTorque", &Joint::setTorque, nb::arg("torque"))
      .def("getPositionLowerLimits", &Joint::getPositionLowerLimits)
      .def(
          "setPositionLowerLimits",
          &Joint::setPositionLowerLimits,
          nb::arg("limits"))
      .def("getPositionUpperLimits", &Joint::getPositionUpperLimits)
      .def(
          "setPositionUpperLimits",
          &Joint::setPositionUpperLimits,
          nb::arg("limits"))
      .def("getVelocityLimits", &Joint::getVelocityLimits)
      .def("setVelocityLimits", &Joint::setVelocityLimits, nb::arg("limits"))
      .def("getEffortLimits", &Joint::getEffortLimits)
      .def("setEffortLimits", &Joint::setEffortLimits, nb::arg("limits"));
}

void defExpLink(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<Link, Frame>(m, "Link")
      .def("getName", &Link::getName)
      .def("getParentJoint", &Link::getParentJoint)
      .def("getLocalTransform", &Link::getLocalTransform)
      .def("getWorldTransform", &Link::getWorldTransform)
      .def(
          "createShapeNode",
          &Link::createShapeNode,
          nb::arg("shape"),
          nb::arg("name") = "",
          nb::arg("options") = ShapeNodeOptions{});
}

void defExpMultiBody(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<MultiBody>(m, "MultiBody")
      .def("getName", &MultiBody::getName)
      .def("setName", &MultiBody::setName, nb::arg("name"))
      .def("getLinkCount", &MultiBody::getLinkCount)
      .def("getJointCount", &MultiBody::getJointCount)
      .def("getDOFCount", &MultiBody::getDOFCount)
      .def("getLink", &MultiBody::getLink, nb::arg("name"))
      .def("getJoint", &MultiBody::getJoint, nb::arg("name"))
      .def(
          "addLink",
          nb::overload_cast<std::string_view>(&MultiBody::addLink),
          nb::arg("name") = "")
      .def(
          "addLinkWithJoint",
          [](MultiBody& self,
             std::string_view name,
             const Link& parentLink,
             const std::string& jointName,
             comps::JointType jointType,
             const Eigen::Vector3d& axis) {
            return self.addLink(
                name, LinkOptions{parentLink, jointName, jointType, axis});
          },
          nb::arg("name"),
          nb::arg("parent_link"),
          nb::arg("joint_name"),
          nb::arg("joint_type") = comps::JointType::Revolute,
          nb::arg("axis") = Eigen::Vector3d::UnitZ());
}

} // namespace dart::python_nb
