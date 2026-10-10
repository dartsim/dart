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

void World(nb::module_& m)
{
  nb::enum_<dart::simulation::CollisionDetectorType>(
      m, "CollisionDetectorType", nb::is_arithmetic())
      .value("DART", dart::simulation::CollisionDetectorType::Dart)
      .value("FCL", dart::simulation::CollisionDetectorType::Fcl)
      .value("BULLET", dart::simulation::CollisionDetectorType::Bullet)
      .value("ODE", dart::simulation::CollisionDetectorType::Ode)
      .export_values();

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
