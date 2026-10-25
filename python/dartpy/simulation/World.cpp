// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/world.hpp"

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
#include <stdexcept>
#include <string>

#include <cmath>
#include <cstddef>

namespace dart {
namespace python {

namespace {

void setTimeStep(dart::simulation::World& world, double timeStep)
{
  dartnb::checkWorldBusy(world);
  if (!std::isfinite(timeStep) || timeStep <= 0.0)
    throw nb::value_error("time_step must be positive and finite");
  world.setTimeStep(timeStep);
}

void setGravity(dart::simulation::World& world, nb::handle gravity)
{
  dartnb::checkWorldBusy(world);
  Eigen::Vector3d value;
  if (!nb::try_cast(gravity, value))
    throw nb::value_error("gravity must be a vector of length 3");
  world.setGravity(value);
}

struct WorldHandle
{
  nb::object owner;

  explicit WorldHandle(nb::handle world) : owner(nb::borrow(world)) {}

  dart::simulation::World& get() const
  {
    auto& world = nb::cast<dart::simulation::World&>(owner);
    dartnb::checkWorldBusy(world);
    return world;
  }

  void clear()
  {
    owner.reset();
  }
};

struct WorldCollision : WorldHandle
{
  using WorldHandle::WorldHandle;
};

struct WorldSolver : WorldHandle
{
  using WorldHandle::WorldHandle;
};

struct WorldEdit : WorldHandle
{
  using WorldHandle::WorldHandle;
  std::unique_ptr<dart::simulation::World::EditScope> scope;

  void clear()
  {
    scope.reset();
    WorldHandle::clear();
  }

  void exit(bool failed)
  {
    get();
    if (!scope)
      throw std::runtime_error("World edit scope is not active");
    if (failed) {
      struct BodyRaised
      {
      };
      try {
        // Python __exit__ has no C++ unwind; give EditScope the same signal.
        auto unwinding = std::move(scope);
        throw BodyRaised{};
      } catch (const BodyRaised&) {
      }
    } else {
      scope.reset();
    }
  }
};

template <class T>
const PyType_Slot* worldHandleSlots()
{
  static const PyType_Slot slots[] = {
      {Py_tp_traverse,
       reinterpret_cast<void*>(+[](PyObject* self, visitproc visit, void* arg) {
         Py_VISIT(Py_TYPE(self));
         if (nb::inst_ready(self))
           Py_VISIT(nb::inst_ptr<T>(self)->owner.ptr());
         return 0;
       })},
      {Py_tp_clear, reinterpret_cast<void*>(+[](PyObject* self) {
         if (nb::inst_ready(self))
           nb::inst_ptr<T>(self)->clear();
         return 0;
       })},
      {0, nullptr}};
  return slots;
}

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
  nb::class_<WorldCollision>(
      m, "WorldCollision", nb::type_slots(worldHandleSlots<WorldCollision>()))
      .def_prop_rw(
          "detector",
          [](const WorldCollision& self) {
            return self.get().getCollisionDetector();
          },
          [](WorldCollision& self,
             const dart::collision::CollisionDetectorPtr& detector) {
            self.get().setCollisionDetector(detector);
          },
          "The live detector. Replacement requires edit() on a compiled "
          "World.");

  nb::class_<WorldSolver>(
      m, "WorldSolver", nb::type_slots(worldHandleSlots<WorldSolver>()))
      .def_prop_rw(
          "threads",
          [](const WorldSolver& self) {
            return self.get().getNumSimulationThreads();
          },
          [](WorldSolver& self, std::size_t threads) {
            self.get().setNumSimulationThreads(threads);
          },
          "Simulation worker count; assigning 0 selects the hardware count.");

  nb::class_<WorldEdit>(
      m, "_WorldEdit", nb::type_slots(worldHandleSlots<WorldEdit>()))
      .def(
          "__enter__",
          [](WorldEdit& self) -> WorldEdit& {
            auto& world = self.get();
            if (self.scope)
              throw std::runtime_error("World edit scope is already active");
            self.scope
                = std::make_unique<dart::simulation::World::EditScope>(world);
            return self;
          },
          nb::rv_policy::reference_internal)
      .def(
          "__exit__",
          [](WorldEdit& self, nb::handle type, nb::handle, nb::handle) {
            self.exit(!type.is_none());
            return false;
          },
          nb::arg("exc_type").none(),
          nb::arg("exc_value").none(),
          nb::arg("traceback").none());

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
      .def(
          dartnb::factory(
              +[](const std::string& name,
                  double timeStep,
                  nb::handle gravity,
                  const dart::collision::CollisionDetectorPtr& detector,
                  std::size_t threads) {
                auto world = dart::simulation::World::create(name);
                setTimeStep(*world, timeStep);
                setGravity(*world, gravity);
                if (detector)
                  world->setCollisionDetector(detector);
                world->setNumSimulationThreads(threads);
                return world;
              }),
          nb::kw_only(),
          nb::arg("name") = "world",
          nb::arg("time_step") = 0.001,
          nb::arg("gravity") = nb::make_tuple(0.0, 0.0, -9.81),
          nb::arg("collision_detector").none() = nb::none(),
          nb::arg("threads") = 1,
          "Construct a World. Omitted collision_detector or None keeps FCL.")
      .def_prop_rw(
          "name",
          [](const dart::simulation::World& self) {
            dartnb::checkWorldBusy(self);
            return std::string(self.getName());
          },
          [](dart::simulation::World& self, const std::string& name) {
            dartnb::checkWorldBusy(self);
            self.setName(name);
          })
      .def_prop_rw(
          "time_step",
          [](const dart::simulation::World& self) {
            dartnb::checkWorldBusy(self);
            return self.getTimeStep();
          },
          &setTimeStep,
          "Step duration in seconds; must be positive and finite.")
      .def_prop_rw(
          "gravity",
          [](const dart::simulation::World& self) {
            dartnb::checkWorldBusy(self);
            return Eigen::Vector3d(self.getGravity());
          },
          &setGravity,
          "Gravity vector; the getter returns a read-only copy.")
      .def_prop_ro(
          "is_compiled",
          [](const dart::simulation::World& self) {
            dartnb::checkWorldBusy(self);
            return self.isCompiled();
          })
      .def_prop_ro(
          "collision",
          [](nb::handle self) {
            dartnb::checkWorldBusy(nb::cast<dart::simulation::World&>(self));
            return WorldCollision(self);
          })
      .def_prop_ro(
          "solver",
          [](nb::handle self) {
            dartnb::checkWorldBusy(nb::cast<dart::simulation::World&>(self));
            return WorldSolver(self);
          })
      .def(
          "compile",
          [](dart::simulation::World& self) {
            dartnb::checkWorldBusy(self);
            self.compile();
          },
          "Prepare simulation data and freeze topology.")
      .def(
          "edit",
          [](nb::handle self) {
            dartnb::checkWorldBusy(nb::cast<dart::simulation::World&>(self));
            return WorldEdit(self);
          },
          "Return a topology-edit context manager. Nested scopes are allowed.\n"
          "Successful exit recompiles a previously compiled World; lazy "
          "Worlds\n"
          "stay lazy. If the body raises, edits remain and the World stays\n"
          "uncompiled. A failed recompile is reported by step() or compile().")
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
          nb::arg("skeleton"))
      .def(
          "removeSkeleton",
          +[](dart::simulation::World* self,
              const dart::dynamics::SkeletonPtr& _skeleton) -> void {
            return self->removeSkeleton(_skeleton);
          },
          nb::arg("skeleton"))
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
          nb::arg("frame"))
      .def(
          "removeSimpleFrame",
          +[](dart::simulation::World* self,
              const dart::dynamics::SimpleFramePtr& _frame) -> void {
            return self->removeSimpleFrame(_frame);
          },
          nb::arg("frame"))
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
