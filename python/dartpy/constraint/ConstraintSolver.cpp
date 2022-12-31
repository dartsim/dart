/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void ConstraintSolver(py::module& m)
{
  ::py::class_<
      dart::constraint::ConstraintSolver,
      std::shared_ptr<dart::constraint::ConstraintSolver>>(
      m, "ConstraintSolver")
      .def(
          "addSkeleton",
          +[](dart::constraint::ConstraintSolver* self,
              const dart::dynamics::SkeletonPtr& skeleton) {
            self->addSkeleton(skeleton);
          },
          ::py::arg("skeleton"))
      .def(
          "addSkeletons",
          +[](dart::constraint::ConstraintSolver* self,
              const std::vector<dart::dynamics::SkeletonPtr>& skeletons) {
            self->addSkeletons(skeletons);
          },
          ::py::arg("skeletons"))
      .def(
          "removeSkeleton",
          +[](dart::constraint::ConstraintSolver* self,
              const dart::dynamics::SkeletonPtr& skeleton) {
            self->removeSkeleton(skeleton);
          },
          ::py::arg("skeleton"))
      .def(
          "removeSkeletons",
          +[](dart::constraint::ConstraintSolver* self,
              const std::vector<dart::dynamics::SkeletonPtr>& skeletons) {
            self->removeSkeletons(skeletons);
          },
          ::py::arg("skeletons"))
      .def(
          "removeAllSkeletons",
          +[](dart::constraint::ConstraintSolver* self) {
            self->removeAllSkeletons();
          })
      .def(
          "addConstraint",
          +[](dart::constraint::ConstraintSolver* self,
              const dart::constraint::ConstraintBasePtr& constraint) {
            self->addConstraint(constraint);
          },
          ::py::arg("constraint"))
      .def(
          "removeConstraint",
          +[](dart::constraint::ConstraintSolver* self,
              const dart::constraint::ConstraintBasePtr& constraint) {
            self->removeConstraint(constraint);
          },
          ::py::arg("constraint"))
      .def(
          "removeAllConstraints",
          +[](dart::constraint::ConstraintSolver* self) {
            self->removeAllConstraints();
          })
      .def(
          "getNumConstraints",
          +[](const dart::constraint::ConstraintSolver* self) -> bool {
            return self->getNumConstraints();
          })
      .def(
          "getConstraint",
          +[](dart::constraint::ConstraintSolver* self,
              std::size_t index) -> constraint::ConstraintBasePtr {
            return self->getConstraint(index);
          },
          ::py::arg("index"))
      .def(
          "getNumConstraints",
          +[](const dart::constraint::ConstraintSolver* self) -> bool {
            return self->getNumConstraints();
          })
      .def(
          "getConstraint",
          +[](dart::constraint::ConstraintSolver* self,
              std::size_t index) -> constraint::ConstraintBasePtr {
            return self->getConstraint(index);
          },
          ::py::arg("index"))
      .def(
          "clearLastCollisionResult",
          +[](dart::constraint::ConstraintSolver* self) {
            self->clearLastCollisionResult();
          })
      .def(
          "setTimeStep",
          +[](dart::constraint::ConstraintSolver* self, double _timeStep) {
            self->setTimeStep(_timeStep);
          },
          ::py::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::constraint::ConstraintSolver* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "setCollisionDetector",
          +[](dart::constraint::ConstraintSolver* self,
              const std::shared_ptr<dart::collision::CollisionDetector>&
                  collisionDetector) {
            self->setCollisionDetector(collisionDetector);
          },
          ::py::arg("collisionDetector"))
      .def(
          "getCollisionDetector",
          +[](dart::constraint::ConstraintSolver* self)
              -> dart::collision::CollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "getCollisionDetector",
          +[](const dart::constraint::ConstraintSolver* self)
              -> dart::collision::ConstCollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "getCollisionGroup",
          +[](dart::constraint::ConstraintSolver* self)
              -> dart::collision::CollisionGroupPtr {
            return self->getCollisionGroup();
          })
      .def(
          "getCollisionGroup",
          +[](const dart::constraint::ConstraintSolver* self)
              -> dart::collision::ConstCollisionGroupPtr {
            return self->getCollisionGroup();
          })
      .def(
          "getCollisionOption",
          +[](dart::constraint::ConstraintSolver* self)
              -> dart::collision::CollisionOption& {
            return self->getCollisionOption();
          },
          "Returns collision option that is used for collision checkings in "
          "this ConstraintSolver to generate contact constraints.")
      .def(
          "getCollisionOption",
          +[](const dart::constraint::ConstraintSolver* self)
              -> const dart::collision::CollisionOption& {
            return self->getCollisionOption();
          },
          "Returns collision option that is used for collision checkings in "
          "this ConstraintSolver to generate contact constraints.")
      .def(
          "solve",
          +[](dart::constraint::ConstraintSolver* self) { self->solve(); });
}

} // namespace python
} // namespace dart
