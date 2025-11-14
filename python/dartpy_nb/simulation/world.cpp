#include "simulation/world.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

#include "simulation/constraint_solver.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defWorld(nb::module_& m)
{
  using World = dart::simulation::World;

  nb::class_<World, std::shared_ptr<World>>(m, "World")
      .def(nb::init<>())
      .def(nb::init<const std::string&>(), nb::arg("name"))
      .def("getNumSkeletons", &World::getNumSkeletons)
      .def("getNumSimpleFrames", &World::getNumSimpleFrames)
      .def("getSkeleton",
          [](World& self, std::size_t index) {
            return self.getSkeleton(index);
          },
          nb::arg("index"))
      .def("getSkeleton",
          [](World& self, const std::string& name) {
            return self.getSkeleton(name);
          },
          nb::arg("name"))
      .def("setGravity",
          [](World& self, const Eigen::Vector3d& gravity) {
            self.setGravity(gravity);
          },
          nb::arg("gravity"))
      .def("getGravity",
          &World::getGravity,
          nb::rv_policy::reference_internal)
      .def("addSkeleton",
          [](World& self, const std::shared_ptr<dart::dynamics::Skeleton>& skeleton) {
            return self.addSkeleton(skeleton);
          },
          nb::arg("skeleton"))
      .def("setTimeStep", &World::setTimeStep, nb::arg("timeStep"))
      .def("getTimeStep", &World::getTimeStep)
      .def("step", &World::step)
      .def("getTime", &World::getTime)
      .def("getConstraintSolver",
          [](World& self) -> dart::constraint::ConstraintSolver* {
            return self.getConstraintSolver();
          },
          nb::rv_policy::reference_internal);
}

} // namespace dart::python_nb
