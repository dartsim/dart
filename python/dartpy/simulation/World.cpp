/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <dart/simulation/World.hpp>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void World(py::module& m)
{
  ::py::class_<
      dart::simulation::World,
      std::shared_ptr<dart::simulation::World>>(m, "World")
      .def(::py::init<>())
      .def(::py::init<const std::string&>(), ::py::arg("name"))
      .def(::py::init(+[]() -> dart::simulation::WorldPtr {
        return dart::simulation::World::create();
      }))
      .def(
          ::py::init(
              +[](const std::string& name) -> dart::simulation::WorldPtr {
                return dart::simulation::World::create(name);
              }),
          ::py::arg("name"))
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
          ::py::return_value_policy::reference_internal,
          ::py::arg("newName"))
      .def(
          "getName",
          +[](const dart::simulation::World* self) -> const std::string& {
            return self->getName();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setGravity",
          +[](dart::simulation::World* self, const Eigen::Vector3d& _gravity)
              -> void { return self->setGravity(_gravity); },
          ::py::arg("gravity"))
      .def(
          "getGravity",
          +[](const dart::simulation::World* self) -> const Eigen::Vector3d& {
            return self->getGravity();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setTimeStep",
          +[](dart::simulation::World* self, double _timeStep) -> void {
            return self->setTimeStep(_timeStep);
          },
          ::py::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::simulation::World* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "getSkeleton",
          +[](const dart::simulation::World* self,
              std::size_t _index) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton(_index);
          },
          ::py::arg("index"))
      .def(
          "getSkeleton",
          +[](const dart::simulation::World* self,
              const std::string& _name) -> dart::dynamics::SkeletonPtr {
            return self->getSkeleton(_name);
          },
          ::py::arg("name"))
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
          ::py::arg("skeleton"))
      .def(
          "removeSkeleton",
          +[](dart::simulation::World* self,
              const dart::dynamics::SkeletonPtr& _skeleton) -> void {
            return self->removeSkeleton(_skeleton);
          },
          ::py::arg("skeleton"))
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
          ::py::arg("skeleton"))
      .def(
          "getIndex",
          +[](const dart::simulation::World* self, int _index) -> int {
            return self->getIndex(_index);
          },
          ::py::arg("index"))
      .def(
          "getSimpleFrame",
          +[](const dart::simulation::World* self,
              std::size_t _index) -> dart::dynamics::SimpleFramePtr {
            return self->getSimpleFrame(_index);
          },
          ::py::arg("index"))
      .def(
          "getSimpleFrame",
          +[](const dart::simulation::World* self,
              const std::string& _name) -> dart::dynamics::SimpleFramePtr {
            return self->getSimpleFrame(_name);
          },
          ::py::arg("name"))
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
          ::py::arg("frame"))
      .def(
          "removeSimpleFrame",
          +[](dart::simulation::World* self,
              const dart::dynamics::SimpleFramePtr& _frame) -> void {
            return self->removeSimpleFrame(_frame);
          },
          ::py::arg("frame"))
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
          ::py::arg("option"))
      .def(
          "checkCollision",
          +[](dart::simulation::World* self,
              const dart::collision::CollisionOption& option,
              dart::collision::CollisionResult* result) -> bool {
            return self->checkCollision(option, result);
          },
          ::py::arg("option"),
          ::py::arg("result"))
      .def(
          "getLastCollisionResult",
          +[](dart::simulation::World* self)
              -> const collision::CollisionResult& {
            return self->getLastCollisionResult();
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
          ::py::arg("resetCommand"))
      .def(
          "setTime",
          +[](dart::simulation::World* self, double _time) -> void {
            return self->setTime(_time);
          },
          ::py::arg("time"))
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
          ::py::return_value_policy::reference_internal)
      .def(
          "bake",
          +[](dart::simulation::World* self) -> void { return self->bake(); })
      .def_readonly("onNameChanged", &dart::simulation::World::onNameChanged);
}

} // namespace python
} // namespace dart
