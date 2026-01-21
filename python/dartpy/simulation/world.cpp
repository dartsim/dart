#include "simulation/world.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"
#include "common/type_casters.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/collision/collision_result.hpp"
#include "dart/constraint/constraint_solver.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/simulation/world.hpp"
#include "simulation/constraint_solver.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <memory>

namespace nb = nanobind;

namespace dart::python_nb {

void defWorld(nb::module_& m)
{
  using World = dart::simulation::World;

  nb::class_<World>(m, "World")
      .def(nb::init<>())
      .def(nb::init<const std::string&>(), nb::arg("name"))
      .def("clone", [](const World& self) { return self.clone(); })
      .def(
          "setName",
          [](World& self, const std::string& new_name) -> const std::string& {
            return self.setName(new_name);
          },
          nb::arg("new_name"),
          nb::rv_policy::reference_internal)
      .def(
          "getName",
          [](const World& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setGravity",
          nb::overload_cast<const Eigen::Vector3d&>(&World::setGravity),
          nb::arg("gravity"))
      .def(
          "setGravity",
          [](World& self, const nb::handle& gravity) {
            self.setGravity(toVector3(gravity));
          },
          nb::arg("gravity"))
      .def(
          "setGravity",
          nb::overload_cast<double, double, double>(&World::setGravity),
          nb::arg("x"),
          nb::arg("y"),
          nb::arg("z"))
      .def("getGravity", &World::getGravity, nb::rv_policy::reference_internal)
      .def("setTimeStep", &World::setTimeStep, nb::arg("time_step"))
      .def("getTimeStep", &World::getTimeStep)
      .def(
          "getSkeleton",
          [](World& self, std::size_t index) {
            return self.getSkeleton(index);
          },
          nb::arg("index"))
      .def(
          "getSkeleton",
          [](World& self, const std::string& name) {
            return self.getSkeleton(name);
          },
          nb::arg("name"))
      .def("getNumSkeletons", &World::getNumSkeletons)
      .def(
          "addSkeleton",
          [](World& self,
             const std::shared_ptr<dart::dynamics::Skeleton>& skeleton) {
            return self.addSkeleton(skeleton);
          },
          nb::arg("skeleton"))
      .def(
          "removeSkeleton",
          [](World& self,
             const std::shared_ptr<dart::dynamics::Skeleton>& skeleton) {
            self.removeSkeleton(skeleton);
          },
          nb::arg("skeleton"))
      .def(
          "removeAllSkeletons",
          [](World& self) { return self.removeAllSkeletons(); })
      .def(
          "hasSkeleton",
          [](World& self,
             const std::shared_ptr<const dart::dynamics::Skeleton>& skeleton) {
            return self.hasSkeleton(skeleton);
          },
          nb::arg("skeleton"))
      .def(
          "hasSkeleton",
          [](World& self, const std::string& skeleton_name) {
            return self.hasSkeleton(skeleton_name);
          },
          nb::arg("skeleton_name"))
      .def(
          "getIndex",
          [](World& self, int index) { return self.getIndex(index); },
          nb::arg("index"))
      .def("getNumSimpleFrames", &World::getNumSimpleFrames)
      .def(
          "getSimpleFrame",
          [](World& self, std::size_t index) {
            return self.getSimpleFrame(index);
          },
          nb::arg("index"))
      .def(
          "getSimpleFrame",
          [](World& self, const std::string& name) {
            return self.getSimpleFrame(name);
          },
          nb::arg("name"))
      .def(
          "addSimpleFrame",
          [](World& self,
             const std::shared_ptr<dart::dynamics::SimpleFrame>& frame) {
            return self.addSimpleFrame(frame);
          },
          nb::arg("frame"))
      .def(
          "removeSimpleFrame",
          [](World& self,
             const std::shared_ptr<dart::dynamics::SimpleFrame>& frame) {
            self.removeSimpleFrame(frame);
          },
          nb::arg("frame"))
      .def(
          "removeAllSimpleFrames",
          [](World& self) { return self.removeAllSimpleFrames(); })
      .def(
          "checkCollision",
          [](World& self) { return self.checkCollision(); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "checkCollision",
          [](World& self, const dart::collision::CollisionOption& option) {
            return self.checkCollision(option);
          },
          nb::arg("option"),
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "checkCollision",
          [](World& self,
             const dart::collision::CollisionOption& option,
             dart::collision::CollisionResult* result) {
            return self.checkCollision(option, result);
          },
          nb::arg("option"),
          nb::arg("result"),
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "checkCollisionResult",
          [](World& self, const dart::collision::CollisionOption& option)
              -> dart::collision::CollisionResult {
            dart::collision::CollisionResult result;
            self.checkCollision(option, &result);
            return result;
          },
          nb::arg("option") = dart::collision::CollisionOption(),
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "getLastCollisionResult",
          [](World& self) -> const dart::collision::CollisionResult& {
            return self.getLastCollisionResult();
          },
          nb::rv_policy::reference_internal)
      .def("reset", &World::reset)
      .def(
          "step",
          [](World& self) { self.step(); },
          nb::call_guard<nb::gil_scoped_release>())
      .def(
          "step",
          [](World& self, bool resetCommand) { self.step(resetCommand); },
          nb::arg("reset_command"),
          nb::call_guard<nb::gil_scoped_release>())
      .def("setTime", &World::setTime, nb::arg("time"))
      .def("getTime", &World::getTime)
      .def("getSimFrames", &World::getSimFrames)
      .def(
          "getConstraintSolver",
          [](World& self) -> dart::constraint::ConstraintSolver* {
            return self.getConstraintSolver();
          },
          nb::rv_policy::reference_internal)
      .def("bake", &World::bake)
      .def("__repr__", [](const World& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("name", repr_string(self.getName()));
        fields.emplace_back(
            "skeletons", std::to_string(self.getNumSkeletons()));
        fields.emplace_back("time", repr_double(self.getTime()));
        fields.emplace_back("time_step", repr_double(self.getTimeStep()));
        return format_repr("World", fields);
      });
}

} // namespace dart::python_nb
