/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void ConstraintSolver(py::module& m)
{
  ::py::class_<
      dart::dynamics::ConstraintSolver,
      std::shared_ptr<dart::dynamics::ConstraintSolver>>(m, "ConstraintSolver")
      .def(
          "addSkeleton",
          +[](dart::dynamics::ConstraintSolver* self,
              const dart::dynamics::SkeletonPtr& skeleton) {
            self->addSkeleton(skeleton);
          },
          ::py::arg("skeleton"))
      .def(
          "addSkeletons",
          +[](dart::dynamics::ConstraintSolver* self,
              const std::vector<dart::dynamics::SkeletonPtr>& skeletons) {
            self->addSkeletons(skeletons);
          },
          ::py::arg("skeletons"))
      .def(
          "removeSkeleton",
          +[](dart::dynamics::ConstraintSolver* self,
              const dart::dynamics::SkeletonPtr& skeleton) {
            self->removeSkeleton(skeleton);
          },
          ::py::arg("skeleton"))
      .def(
          "removeSkeletons",
          +[](dart::dynamics::ConstraintSolver* self,
              const std::vector<dart::dynamics::SkeletonPtr>& skeletons) {
            self->removeSkeletons(skeletons);
          },
          ::py::arg("skeletons"))
      .def(
          "removeAllSkeletons",
          +[](dart::dynamics::ConstraintSolver* self) {
            self->removeAllSkeletons();
          })
      .def(
          "addConstraint",
          +[](dart::dynamics::ConstraintSolver* self,
              const dart::dynamics::ConstraintBasePtr& dynamics) {
            self->addConstraint(dynamics);
          },
          ::py::arg("dynamics"))
      .def(
          "removeConstraint",
          +[](dart::dynamics::ConstraintSolver* self,
              const dart::dynamics::ConstraintBasePtr& dynamics) {
            self->removeConstraint(dynamics);
          },
          ::py::arg("dynamics"))
      .def(
          "removeAllConstraints",
          +[](dart::dynamics::ConstraintSolver* self) {
            self->removeAllConstraints();
          })
      .def(
          "getNumConstraints",
          +[](const dart::dynamics::ConstraintSolver* self) -> bool {
            return self->getNumConstraints();
          })
      .def(
          "getConstraint",
          +[](dart::dynamics::ConstraintSolver* self,
              std::size_t index) -> dynamics::ConstraintBasePtr {
            return self->getConstraint(index);
          },
          ::py::arg("index"))
      .def(
          "getNumConstraints",
          +[](const dart::dynamics::ConstraintSolver* self) -> bool {
            return self->getNumConstraints();
          })
      .def(
          "getConstraint",
          +[](dart::dynamics::ConstraintSolver* self,
              std::size_t index) -> dynamics::ConstraintBasePtr {
            return self->getConstraint(index);
          },
          ::py::arg("index"))
      .def(
          "clearLastCollisionResult",
          +[](dart::dynamics::ConstraintSolver* self) {
            self->clearLastCollisionResult();
          })
      .def(
          "setTimeStep",
          +[](dart::dynamics::ConstraintSolver* self, double _timeStep) {
            self->setTimeStep(_timeStep);
          },
          ::py::arg("timeStep"))
      .def(
          "getTimeStep",
          +[](const dart::dynamics::ConstraintSolver* self) -> double {
            return self->getTimeStep();
          })
      .def(
          "setCollisionDetector",
          +[](dart::dynamics::ConstraintSolver* self,
              const std::shared_ptr<dart::dynamics::CollisionDetector>&
                  collisionDetector) {
            self->setCollisionDetector(collisionDetector);
          },
          ::py::arg("collisionDetector"))
      .def(
          "getCollisionDetector",
          +[](dart::dynamics::ConstraintSolver* self)
              -> dart::dynamics::CollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "getCollisionDetector",
          +[](const dart::dynamics::ConstraintSolver* self)
              -> dart::dynamics::ConstCollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "getCollisionGroup",
          +[](dart::dynamics::ConstraintSolver* self)
              -> dart::dynamics::CollisionGroupPtr {
            return self->getCollisionGroup();
          })
      .def(
          "getCollisionGroup",
          +[](const dart::dynamics::ConstraintSolver* self)
              -> dart::dynamics::ConstCollisionGroupPtr {
            return self->getCollisionGroup();
          })
      .def(
          "getCollisionOption",
          +[](dart::dynamics::ConstraintSolver* self)
              -> dart::dynamics::CollisionOption& {
            return self->getCollisionOption();
          },
          "Returns collision option that is used for collision checkings in "
          "this ConstraintSolver to generate contact constraints.")
      .def(
          "getCollisionOption",
          +[](const dart::dynamics::ConstraintSolver* self)
              -> const dart::dynamics::CollisionOption& {
            return self->getCollisionOption();
          },
          "Returns collision option that is used for collision checkings in "
          "this ConstraintSolver to generate contact constraints.")
      .def("solve", +[](dart::dynamics::ConstraintSolver* self) {
        self->solve();
      });
}

} // namespace python
} // namespace dart
