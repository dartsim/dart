#include "simulation_experimental/rigid_body.hpp"

#include "dart/simulation/experimental/body/rigid_body.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defRigidBody(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<RigidBody, Frame>(m, "RigidBody")
      .def("getName", &RigidBody::getName)
      .def("getMass", &RigidBody::getMass)
      .def("setMass", &RigidBody::setMass, nb::arg("mass"))
      .def("getInertia", &RigidBody::getInertia)
      .def("setInertia", &RigidBody::setInertia, nb::arg("inertia"))
      .def("getPosition", &RigidBody::getPosition)
      .def("setPosition", &RigidBody::setPosition, nb::arg("position"))
      .def("getOrientation", &RigidBody::getOrientation)
      .def("setOrientation", &RigidBody::setOrientation, nb::arg("orientation"))
      .def("getLinearVelocity", &RigidBody::getLinearVelocity)
      .def(
          "setLinearVelocity",
          &RigidBody::setLinearVelocity,
          nb::arg("velocity"))
      .def("getAngularVelocity", &RigidBody::getAngularVelocity)
      .def(
          "setAngularVelocity",
          &RigidBody::setAngularVelocity,
          nb::arg("velocity"))
      .def("getForce", &RigidBody::getForce)
      .def("addForce", &RigidBody::addForce, nb::arg("force"))
      .def("getTorque", &RigidBody::getTorque)
      .def("addTorque", &RigidBody::addTorque, nb::arg("torque"))
      .def("clearForces", &RigidBody::clearForces);
}

} // namespace dart::python_nb
