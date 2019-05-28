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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void CollisionGroup(py::module& m)
{
  ::py::class_<
      dart::collision::CollisionGroup,
      std::shared_ptr<dart::collision::CollisionGroup> >(m, "CollisionGroup")
      .def(
          "getCollisionDetector",
          +[](dart::collision::CollisionGroup* self)
              -> dart::collision::CollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "getCollisionDetector",
          +[](const dart::collision::CollisionGroup* self)
              -> dart::collision::ConstCollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "addShapeFrame",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) {
            self->addShapeFrame(shapeFrame);
          },
          ::py::arg("shapeFrame"))
      .def(
          "addShapeFrames",
          +[](dart::collision::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) { self->addShapeFrames(shapeFrames); },
          ::py::arg("shapeFrames"))
      .def(
          "addShapeFramesOf",
          +[](dart::collision::CollisionGroup* self) {
            self->addShapeFramesOf();
          })
      .def(
          "subscribeTo",
          +[](dart::collision::CollisionGroup* self) { self->subscribeTo(); })
      .def(
          "removeShapeFrame",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) {
            self->removeShapeFrame(shapeFrame);
          },
          ::py::arg("shapeFrame"))
      .def(
          "removeShapeFrames",
          +[](dart::collision::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) { self->removeShapeFrames(shapeFrames); },
          ::py::arg("shapeFrames"))
      .def(
          "removeShapeFramesOf",
          +[](dart::collision::CollisionGroup* self) {
            self->removeShapeFramesOf();
          })
      .def(
          "removeAllShapeFrames",
          +[](dart::collision::CollisionGroup* self) {
            self->removeAllShapeFrames();
          })
      .def(
          "hasShapeFrame",
          +[](const dart::collision::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) -> bool {
            return self->hasShapeFrame(shapeFrame);
          },
          ::py::arg("shapeFrame"))
      .def(
          "getNumShapeFrames",
          +[](const dart::collision::CollisionGroup* self) -> std::size_t {
            return self->getNumShapeFrames();
          })
      .def(
          "collide",
          +[](dart::collision::CollisionGroup* self) -> bool {
            return self->collide();
          })
      .def(
          "collide",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::CollisionOption& option) -> bool {
            return self->collide(option);
          },
          ::py::arg("option"))
      .def(
          "collide",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::CollisionOption& option,
              dart::collision::CollisionResult* result) -> bool {
            return self->collide(option, result);
          },
          ::py::arg("option"),
          ::py::arg("result"))
      .def(
          "distance",
          +[](dart::collision::CollisionGroup* self) -> double {
            return self->distance();
          })
      .def(
          "distance",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::DistanceOption& option) -> double {
            return self->distance(option);
          },
          ::py::arg("option"))
      .def(
          "distance",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::DistanceOption& option,
              dart::collision::DistanceResult* result) -> double {
            return self->distance(option, result);
          },
          ::py::arg("option"),
          ::py::arg("result"))
      .def(
          "raycast",
          +[](dart::collision::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to) -> bool {
            return self->raycast(from, to);
          },
          ::py::arg("from"),
          ::py::arg("to"))
      .def(
          "raycast",
          +[](dart::collision::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to,
              const dart::collision::RaycastOption& option) -> bool {
            return self->raycast(from, to, option);
          },
          ::py::arg("from"),
          ::py::arg("to"),
          ::py::arg("option"))
      .def(
          "raycast",
          +[](dart::collision::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to,
              const dart::collision::RaycastOption& option,
              dart::collision::RaycastResult* result) -> bool {
            return self->raycast(from, to, option, result);
          },
          ::py::arg("from"),
          ::py::arg("to"),
          ::py::arg("option"),
          ::py::arg("result"))
      .def(
          "setAutomaticUpdate",
          +[](dart::collision::CollisionGroup* self) {
            self->setAutomaticUpdate();
          })
      .def(
          "setAutomaticUpdate",
          +[](dart::collision::CollisionGroup* self, bool automatic) {
            self->setAutomaticUpdate(automatic);
          },
          ::py::arg("automatic"))
      .def(
          "getAutomaticUpdate",
          +[](const dart::collision::CollisionGroup* self) -> bool {
            return self->getAutomaticUpdate();
          })
      .def(
          "update",
          +[](dart::collision::CollisionGroup* self) { self->update(); })
      .def(
          "removeDeletedShapeFrames",
          +[](dart::collision::CollisionGroup* self) {
            self->removeDeletedShapeFrames();
          });
}

} // namespace python
} // namespace dart
