// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

#include <nanobind/stl/set.h>

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/DeactivationOptions.hpp>
#include <dart/simulation/Recording.hpp>
#include <dart/simulation/World.hpp>
#include <dart/simulation/WorldState.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionResult.hpp>

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <Eigen/Core>

#include <memory>
#include <set>
#include <string>

#include <cstddef>

namespace dart {
namespace python {

namespace {

// Independent copy of a state channel with NumPy's writeable flag cleared.
template <typename Array>
nb::object readOnlyCopy(Array array)
{
  nb::object result = nb::cast(std::move(array));
  result.attr("setflags")(nb::arg("write") = false);
  return result;
}

} // namespace

void World(nb::module_& m)
{
  nb::enum_<dart::simulation::CollisionDetectorType>(
      m, "CollisionDetectorType", nb::is_arithmetic())
      .value("DART", dart::simulation::CollisionDetectorType::Dart)
      .value("FCL", dart::simulation::CollisionDetectorType::Fcl)
      .value("BULLET", dart::simulation::CollisionDetectorType::Bullet)
      .value("ODE", dart::simulation::CollisionDetectorType::Ode)
      .export_values();

  nb::class_<dart::simulation::WorldState>(
      m,
      "WorldState",
      "Dynamic snapshot of a World: generalized coordinates and control\n"
      "channels in Skeleton/DOF and BodyNode order, plus simulation time.\n"
      "\n"
      "Create one with World.capture_state(); it has no public constructor.\n"
      "Array properties return read-only NumPy copies. Parameters and exact\n"
      "stepping history are not part of a snapshot, and soft-body point\n"
      "states are captured and restored but not yet exposed to Python.")
      .def_prop_ro(
          "positions",
          [](const dart::simulation::WorldState& self) {
            return readOnlyCopy(Eigen::VectorXd(self.positions));
          },
          "Joint positions of all DOFs, shape (num_dofs,); read-only copy.")
      .def_prop_ro(
          "velocities",
          [](const dart::simulation::WorldState& self) {
            return readOnlyCopy(Eigen::VectorXd(self.velocities));
          },
          "Joint velocities of all DOFs, shape (num_dofs,); read-only copy.")
      .def_prop_ro(
          "accelerations",
          [](const dart::simulation::WorldState& self) {
            return readOnlyCopy(Eigen::VectorXd(self.accelerations));
          },
          "Joint accelerations of all DOFs, shape (num_dofs,); read-only "
          "copy.")
      .def_prop_ro(
          "forces",
          [](const dart::simulation::WorldState& self) {
            return readOnlyCopy(Eigen::VectorXd(self.forces));
          },
          "Joint forces of all DOFs, shape (num_dofs,); read-only copy.")
      .def_prop_ro(
          "commands",
          [](const dart::simulation::WorldState& self) {
            return readOnlyCopy(Eigen::VectorXd(self.commands));
          },
          "Joint commands of all DOFs, shape (num_dofs,); read-only copy.")
      .def_prop_ro(
          "body_external_forces",
          [](const dart::simulation::WorldState& self) {
            // The C++ 6xN layout holds one body per column; transpose so each
            // NumPy row is one body's 6-vector, as in Skeleton.get_*(...)
            // per-body tables.
            return readOnlyCopy(
                Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>(
                    self.bodyExternalForces.transpose()));
          },
          "External wrench on each body in its own frame as [torque, force],\n"
          "shape (num_bodies, 6), one row per body in Skeleton/BodyNode\n"
          "order; read-only copy.")
      .def_prop_ro(
          "time",
          [](const dart::simulation::WorldState& self) { return self.time; },
          "Simulation time in seconds.")
      .def("__repr__", [](const dart::simulation::WorldState& self) {
        return "WorldState(time="
               + std::string(nb::repr(nb::float_(self.time)).c_str())
               + ", num_dofs=" + std::to_string(self.positions.size()) + ")";
      });

  dartnb::dart_class<dart::simulation::World>(m, "World")
      .def(dartnb::factory(+[]() { return dart::simulation::World::create(); }))
      .def(
          dartnb::factory(+[](const std::string& name) {
            return dart::simulation::World::create(name);
          }),
          nb::arg("name"))
      .def(dartnb::factory(+[]() -> dart::simulation::WorldPtr {
        return dart::simulation::World::create();
      }))
      .def(
          dartnb::factory(
              +[](const std::string& name) -> dart::simulation::WorldPtr {
                return dart::simulation::World::create(name);
              }),
          nb::arg("name"))
      .def(
          "clone",
          +[](const dart::simulation::World* self)
              -> std::shared_ptr<dart::simulation::World> {
            return self->clone();
          })
      .def(
          "setName",
          +[](dart::simulation::World* self, const std::string& _newName)
              -> const std::string& { return self->setName(_newName); },
          nb::rv_policy::reference_internal,
          nb::arg("newName"))
      .def(
          "getName",
          +[](const dart::simulation::World* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setGravity",
          nb::overload_cast<const Eigen::Vector3d&>(
              &dart::simulation::World::setGravity),
          nb::arg("gravity"))
      .def(
          "setGravity",
          nb::overload_cast<double, double, double>(
              &dart::simulation::World::setGravity),
          nb::arg("x"),
          nb::arg("y"),
          nb::arg("z"))
      .def(
          "getGravity",
          +[](const dart::simulation::World* self) -> const Eigen::Vector3d& {
            return self->getGravity();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setTimeStep",
          +[](dart::simulation::World* self, double _timeStep) -> void {
            return self->setTimeStep(_timeStep);
          },
          nb::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::simulation::World* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "setNumSimulationThreads",
          +[](dart::simulation::World* self, std::size_t numThreads) -> void {
            self->setNumSimulationThreads(numThreads);
          },
          nb::arg("numThreads"))
      .def(
          "getNumSimulationThreads",
          +[](const dart::simulation::World* self) -> std::size_t {
            return self->getNumSimulationThreads();
          })
      .def(
          "getSkeleton",
          +[](const dart::simulation::World* self,
              std::size_t _index) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton(_index);
          },
          nb::arg("index"))
      .def(
          "getSkeleton",
          +[](const dart::simulation::World* self,
              const std::string& _name) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton(_name);
          },
          nb::arg("name"))
      .def(
          "getNumSkeletons",
          +[](const dart::simulation::World* self) -> std::size_t {
            return self->getNumSkeletons();
          })
      .def(
          "addSkeleton",
          +[](dart::simulation::World* self,
              const dart::dynamics::SkeletonPtr& _skeleton) -> std::string {
            return self->addSkeleton(_skeleton);
          },
          nb::arg("skeleton").none())
      .def(
          "removeSkeleton",
          +[](dart::simulation::World* self,
              const dart::dynamics::SkeletonPtr& _skeleton) -> void {
            return self->removeSkeleton(_skeleton);
          },
          nb::arg("skeleton").none())
      .def(
          "removeAllSkeletons",
          +[](dart::simulation::World* self)
              -> std::set<dart::dynamics::SkeletonPtr> {
            return self->removeAllSkeletons();
          })
      .def(
          "hasSkeleton",
          +[](const dart::simulation::World* self,
              const dart::dynamics::ConstSkeletonPtr& skeleton) -> bool {
            return self->hasSkeleton(skeleton);
          },
          nb::arg("skeleton").none())
      .def(
          "getIndex",
          +[](const dart::simulation::World* self, int _index) -> int {
            return self->getIndex(_index);
          },
          nb::arg("index"))
      .def(
          "getSimpleFrame",
          +[](const dart::simulation::World* self,
              std::size_t _index) -> dart::dynamics::SimpleFramePtr {
            return self->getSimpleFrame(_index);
          },
          nb::arg("index"))
      .def(
          "getSimpleFrame",
          +[](const dart::simulation::World* self,
              const std::string& _name) -> dart::dynamics::SimpleFramePtr {
            return self->getSimpleFrame(_name);
          },
          nb::arg("name"))
      .def(
          "getNumSimpleFrames",
          +[](const dart::simulation::World* self) -> std::size_t {
            return self->getNumSimpleFrames();
          })
      .def(
          "addSimpleFrame",
          +[](dart::simulation::World* self,
              const dart::dynamics::SimpleFramePtr& _frame) -> std::string {
            return self->addSimpleFrame(_frame);
          },
          nb::arg("frame").none())
      .def(
          "removeSimpleFrame",
          +[](dart::simulation::World* self,
              const dart::dynamics::SimpleFramePtr& _frame) -> void {
            return self->removeSimpleFrame(_frame);
          },
          nb::arg("frame").none())
      .def(
          "removeAllSimpleFrames",
          +[](dart::simulation::World* self)
              -> std::set<dart::dynamics::SimpleFramePtr> {
            return self->removeAllSimpleFrames();
          })
      .def(
          "checkCollision",
          +[](dart::simulation::World* self) -> bool {
            return self->checkCollision();
          })
      .def(
          "checkCollision",
          +[](dart::simulation::World* self,
              const dart::collision::CollisionOption& option) -> bool {
            return self->checkCollision(option);
          },
          nb::arg("option"))
      .def(
          "checkCollision",
          +[](dart::simulation::World* self,
              const dart::collision::CollisionOption& option,
              dart::collision::CollisionResult* result) -> bool {
            return self->checkCollision(option, result);
          },
          nb::arg("option"),
          nb::arg("result").none())
      .def(
          "getLastCollisionResult",
          +[](dart::simulation::World* self)
              -> const collision::CollisionResult& {
            return self->getLastCollisionResult();
          })
      .def(
          "setCollisionDetector",
          +[](dart::simulation::World* self,
              const collision::CollisionDetectorPtr& detector) {
            self->setCollisionDetector(detector);
          },
          nb::arg("collisionDetector").none())
      .def(
          "setCollisionDetector",
          +[](dart::simulation::World* self,
              dart::simulation::CollisionDetectorType type) {
            self->setCollisionDetector(type);
          },
          nb::arg("collisionDetectorType"))
      .def(
          "getCollisionDetector",
          +[](dart::simulation::World* self)
              -> dart::collision::CollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "reset",
          +[](dart::simulation::World* self) -> void { return self->reset(); })
      .def(
          "step",
          +[](dart::simulation::World* self) -> void { return self->step(); })
      .def(
          "step",
          +[](dart::simulation::World* self, bool _resetCommand) -> void {
            return self->step(_resetCommand);
          },
          nb::arg("resetCommand"))
      .def(
          "setDeactivationOptions",
          +[](dart::simulation::World* self,
              const dart::simulation::DeactivationOptions& options) -> void {
            self->setDeactivationOptions(options);
          },
          nb::arg("options"))
      .def(
          "getDeactivationOptions",
          +[](const dart::simulation::World* self)
              -> dart::simulation::DeactivationOptions {
            return self->getDeactivationOptions();
          })
      .def(
          "setTime",
          +[](dart::simulation::World* self, double _time) -> void {
            return self->setTime(_time);
          },
          nb::arg("time"))
      .def(
          "getTime",
          +[](const dart::simulation::World* self) -> double {
            return self->getTime();
          })
      .def(
          "getSimFrames",
          +[](const dart::simulation::World* self) -> int {
            return self->getSimFrames();
          })
      .def(
          "getConstraintSolver",
          +[](dart::simulation::World* self) -> constraint::ConstraintSolver* {
            return self->getConstraintSolver();
          },
          nb::rv_policy::reference_internal)
      .def(
          "capture_state",
          [](const dart::simulation::World& self,
             dart::simulation::WorldState* out) -> nb::object {
            if (out == nullptr)
              return nb::cast(self.captureState());
            self.captureState(*out);
            return nb::find(out);
          },
          nb::arg("out").none() = nb::none(),
          nb::sig("def capture_state(self, out: dartpy.simulation.WorldState "
                  "| None = None) -> dartpy.simulation.WorldState"),
          "Capture the dynamic state: positions, velocities, accelerations,\n"
          "forces, commands, external body forces, soft-body points and time.\n"
          "Parameters and exact stepping history are not included.\n"
          "\n"
          "If ``out`` is given, it is filled in place and returned instead of\n"
          "a new WorldState. Raises ValueError, leaving ``out`` unchanged, "
          "for\n"
          "unsupported custom state or an ``out`` from an incompatible World.")
      .def(
          "restore_state",
          [](dart::simulation::World& self,
             const dart::simulation::WorldState& state) {
            self.restoreState(state);
          },
          nb::arg("state"),
          "Restore a snapshot from capture_state(). Mobile Skeletons wake, "
          "and\n"
          "parameters stay those of this World; commands follow this World's\n"
          "actuator types and limits. Manual joint constraints must match in\n"
          "type and order, and clone() omits them, so re-add them to a clone\n"
          "before restoring into it. Raises ValueError, leaving the World\n"
          "unchanged, if the state's structure does not match this World or "
          "it\n"
          "holds non-finite values.")
      .def(
          "bake",
          +[](dart::simulation::World* self) -> void { return self->bake(); })
      .def(
          "getRecording",
          &dart::simulation::World::getRecording,
          nb::rv_policy::reference_internal)
      .def_ro("onNameChanged", &dart::simulation::World::onNameChanged);
}

} // namespace python
} // namespace dart
