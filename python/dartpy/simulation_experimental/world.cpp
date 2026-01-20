#include "simulation_experimental/world.hpp"

#include "dart/collision/CollisionResult.hpp"
#include "dart/simulation/experimental/body/rigid_body.hpp"
#include "dart/simulation/experimental/body/rigid_body_options.hpp"
#include "dart/simulation/experimental/frame/fixed_frame.hpp"
#include "dart/simulation/experimental/frame/free_frame.hpp"
#include "dart/simulation/experimental/multi_body/multi_body.hpp"
#include "dart/simulation/experimental/world.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defExperimentalWorld(nb::module_& m)
{
  using namespace dart::simulation::experimental;

  nb::class_<RigidBodyOptions>(m, "RigidBodyOptions")
      .def(nb::init<>())
      .def_rw("mass", &RigidBodyOptions::mass)
      .def_rw("inertia", &RigidBodyOptions::inertia)
      .def_rw("position", &RigidBodyOptions::position)
      .def_rw("orientation", &RigidBodyOptions::orientation)
      .def_rw("linearVelocity", &RigidBodyOptions::linearVelocity)
      .def_rw("angularVelocity", &RigidBodyOptions::angularVelocity);

  nb::class_<World>(m, "World")
      .def(nb::init<>())
      .def(
          "addFreeFrame",
          nb::overload_cast<>(&World::addFreeFrame),
          nb::rv_policy::reference_internal)
      .def(
          "addFreeFrame",
          nb::overload_cast<std::string_view>(&World::addFreeFrame),
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "addFreeFrame",
          nb::overload_cast<std::string_view, const Frame&>(
              &World::addFreeFrame),
          nb::arg("name"),
          nb::arg("parent"),
          nb::rv_policy::reference_internal)
      .def(
          "addFixedFrame",
          nb::overload_cast<std::string_view, const Frame&>(
              &World::addFixedFrame),
          nb::arg("name"),
          nb::arg("parent"),
          nb::rv_policy::reference_internal)
      .def(
          "addFixedFrame",
          nb::overload_cast<
              std::string_view,
              const Frame&,
              const Eigen::Isometry3d&>(&World::addFixedFrame),
          nb::arg("name"),
          nb::arg("parent"),
          nb::arg("offset"),
          nb::rv_policy::reference_internal)
      .def(
          "addMultiBody",
          &World::addMultiBody,
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def(
          "getMultiBody",
          &World::getMultiBody,
          nb::arg("name"),
          nb::rv_policy::reference_internal)
      .def("getMultiBodyCount", &World::getMultiBodyCount)
      .def(
          "addRigidBody",
          &World::addRigidBody,
          nb::arg("name"),
          nb::arg("options") = RigidBodyOptions{},
          nb::rv_policy::reference_internal)
      .def("hasRigidBody", &World::hasRigidBody, nb::arg("name"))
      .def("getRigidBodyCount", &World::getRigidBodyCount)
      .def("isSimulationMode", &World::isSimulationMode)
      .def("enterSimulationMode", &World::enterSimulationMode)
      .def("updateKinematics", &World::updateKinematics)
      .def(
          "step",
          &World::step,
          nb::arg("clear_forces") = true,
          nb::call_guard<nb::gil_scoped_release>())
      .def("getTimeStep", &World::getTimeStep)
      .def("setTimeStep", &World::setTimeStep, nb::arg("time_step"))
      .def("getGravity", &World::getGravity)
      .def("setGravity", &World::setGravity, nb::arg("gravity"))
      .def("getTime", &World::getTime)
      .def("setTime", &World::setTime, nb::arg("time"))
      .def("getFrame", &World::getFrame)
      .def(
          "detectCollisions",
          [](World& self) -> dart::collision::CollisionResult {
            return self.detectCollisions();
          },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "getLastCollisionResult",
          [](World& self) -> const dart::collision::CollisionResult& {
            return self.getLastCollisionResult();
          },
          nb::rv_policy::reference_internal)
      .def("clear", &World::clear);
}

} // namespace dart::python_nb
