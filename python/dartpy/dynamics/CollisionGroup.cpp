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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void CollisionGroup(py::module& m) {
  ::py::class_<
      dart::dynamics::CollisionGroup,
      std::shared_ptr<dart::dynamics::CollisionGroup> >(m, "CollisionGroup")
      .def(
          "getCollisionDetector",
          +[](dart::dynamics::CollisionGroup* self)
              -> dart::dynamics::CollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "getCollisionDetector",
          +[](const dart::dynamics::CollisionGroup* self)
              -> dart::dynamics::ConstCollisionDetectorPtr {
            return self->getCollisionDetector();
          })
      .def(
          "addShapeFrame",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) {
            self->addShapeFrame(shapeFrame);
          },
          ::py::arg("shapeFrame"))
      .def(
          "addShapeFrames",
          +[](dart::dynamics::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) {
            self->addShapeFrames(shapeFrames);
          },
          ::py::arg("shapeFrames"))
      .def(
          "addShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dynamics::ShapeFrame* shapeFrame) {
            self->addShapeFramesOf(shapeFrame);
          },
          ::py::arg("shapeFrame"),
          "Adds a ShapeFrame")
      .def(
          "addShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) {
            self->addShapeFramesOf(shapeFrames);
          },
          ::py::arg("shapeFrames"),
          "Adds ShapeFrames")
      .def(
          "addShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::CollisionGroup* otherGroup) {
            self->addShapeFramesOf(otherGroup);
          },
          ::py::arg("otherGroup"),
          "Adds ShapeFrames of other CollisionGroup")
      .def(
          "addShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::BodyNode* body) {
            self->addShapeFramesOf(body);
          },
          ::py::arg("body"),
          "Adds ShapeFrames of BodyNode")
      .def(
          "addShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::MetaSkeleton* skeleton) {
            self->addShapeFramesOf(skeleton);
          },
          ::py::arg("skeleton"),
          "Adds ShapeFrames of MetaSkeleton")
      .def(
          "subscribeTo",
          +[](dart::dynamics::CollisionGroup* self) {
            self->subscribeTo();
          })
      .def(
          "removeShapeFrame",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) {
            self->removeShapeFrame(shapeFrame);
          },
          ::py::arg("shapeFrame"))
      .def(
          "removeShapeFrames",
          +[](dart::dynamics::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) {
            self->removeShapeFrames(shapeFrames);
          },
          ::py::arg("shapeFrames"))
      .def(
          "removeShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dynamics::ShapeFrame* shapeFrame) {
            self->removeShapeFramesOf(shapeFrame);
          },
          ::py::arg("shapeFrame"),
          "Removes a ShapeFrame")
      .def(
          "removeShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) {
            self->removeShapeFramesOf(shapeFrames);
          },
          ::py::arg("shapeFrames"),
          "Removes ShapeFrames")
      .def(
          "removeShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::CollisionGroup* otherGroup) {
            self->removeShapeFramesOf(otherGroup);
          },
          ::py::arg("otherGroup"),
          "Removes ShapeFrames of other CollisionGroup")
      .def(
          "removeShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::BodyNode* body) {
            self->removeShapeFramesOf(body);
          },
          ::py::arg("body"),
          "Removes ShapeFrames of BodyNode")
      .def(
          "removeShapeFramesOf",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::MetaSkeleton* skeleton) {
            self->removeShapeFramesOf(skeleton);
          },
          ::py::arg("skeleton"),
          "Removes ShapeFrames of MetaSkeleton")
      .def(
          "removeAllShapeFrames",
          +[](dart::dynamics::CollisionGroup* self) {
            self->removeAllShapeFrames();
          })
      .def(
          "hasShapeFrame",
          +[](const dart::dynamics::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) -> bool {
            return self->hasShapeFrame(shapeFrame);
          },
          ::py::arg("shapeFrame"))
      .def(
          "getNumShapeFrames",
          +[](const dart::dynamics::CollisionGroup* self) -> std::size_t {
            return self->getNumShapeFrames();
          })
      .def(
          "collide",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::CollisionOption& option,
              dart::dynamics::CollisionResult* result) -> bool {
            return self->collide(option, result);
          },
          ::py::arg("option")
          = dart::dynamics::CollisionOption(false, 1u, nullptr),
          ::py::arg("result") = nullptr,
          "Performs collision check within this CollisionGroup")
      .def(
          "collide",
          +[](dart::dynamics::CollisionGroup* self,
              dart::dynamics::CollisionGroup* otherGroup,
              const dart::dynamics::CollisionOption& option,
              dart::dynamics::CollisionResult* result) -> bool {
            return self->collide(otherGroup, option, result);
          },
          ::py::arg("otherGroup"),
          ::py::arg("option")
          = dart::dynamics::CollisionOption(false, 1u, nullptr),
          ::py::arg("result") = nullptr,
          "Perform collision check against other CollisionGroup")
      .def(
          "distance",
          +[](dart::dynamics::CollisionGroup* self,
              const dart::dynamics::DistanceOption& option,
              dart::dynamics::DistanceResult* result) -> double {
            return self->distance(option, result);
          },
          ::py::arg("option")
          = dart::dynamics::DistanceOption(false, 0.0, nullptr),
          ::py::arg("result") = nullptr)
      .def(
          "raycast",
          +[](dart::dynamics::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to) -> bool {
            return self->raycast(from, to);
          },
          ::py::arg("from"),
          ::py::arg("to"))
      .def(
          "raycast",
          +[](dart::dynamics::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to,
              const dart::dynamics::RaycastOption& option) -> bool {
            return self->raycast(from, to, option);
          },
          ::py::arg("from"),
          ::py::arg("to"),
          ::py::arg("option"))
      .def(
          "raycast",
          +[](dart::dynamics::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to,
              const dart::dynamics::RaycastOption& option,
              dart::dynamics::RaycastResult* result) -> bool {
            return self->raycast(from, to, option, result);
          },
          ::py::arg("from"),
          ::py::arg("to"),
          ::py::arg("option"),
          ::py::arg("result"))
      .def(
          "setAutomaticUpdate",
          +[](dart::dynamics::CollisionGroup* self) {
            self->setAutomaticUpdate();
          })
      .def(
          "setAutomaticUpdate",
          +[](dart::dynamics::CollisionGroup* self, bool automatic) {
            self->setAutomaticUpdate(automatic);
          },
          ::py::arg("automatic"))
      .def(
          "getAutomaticUpdate",
          +[](const dart::dynamics::CollisionGroup* self) -> bool {
            return self->getAutomaticUpdate();
          })
      .def(
          "update",
          +[](dart::dynamics::CollisionGroup* self) {
            self->update();
          })
      .def(
          "removeDeletedShapeFrames",
          +[](dart::dynamics::CollisionGroup* self) {
            self->removeDeletedShapeFrames();
          });
}

} // namespace python
} // namespace dart
