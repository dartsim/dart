// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

#include <nanobind/stl/vector.h>

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

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/DistanceOption.hpp>
#include <dart/collision/DistanceResult.hpp>
#include <dart/collision/RaycastOption.hpp>
#include <dart/collision/RaycastResult.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/MetaSkeleton.hpp>
#include <dart/dynamics/ShapeFrame.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <cstddef>

namespace dart {
namespace python {

void CollisionGroup(nb::module_& m)
{
  dartnb::dart_class<dart::collision::CollisionGroup>(m, "CollisionGroup")
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
          nb::arg("shapeFrame").none())
      .def(
          "addShapeFrames",
          +[](dart::collision::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) { self->addShapeFrames(shapeFrames); },
          nb::arg("shapeFrames"))
      .def(
          "addShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dynamics::ShapeFrame* shapeFrame) {
            self->addShapeFramesOf(shapeFrame);
          },
          nb::arg("shapeFrame").none(),
          "Adds a ShapeFrame")
      .def(
          "addShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) { self->addShapeFramesOf(shapeFrames); },
          nb::arg("shapeFrames"),
          "Adds ShapeFrames")
      .def(
          "addShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::CollisionGroup* otherGroup) {
            self->addShapeFramesOf(otherGroup);
          },
          nb::arg("otherGroup").none(),
          "Adds ShapeFrames of other CollisionGroup")
      .def(
          "addShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::BodyNode* body) {
            self->addShapeFramesOf(body);
          },
          nb::arg("body").none(),
          "Adds ShapeFrames of BodyNode")
      .def(
          "addShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::MetaSkeleton* skeleton) {
            self->addShapeFramesOf(skeleton);
          },
          nb::arg("skeleton").none(),
          "Adds ShapeFrames of MetaSkeleton")
      .def(
          "subscribeTo",
          +[](dart::collision::CollisionGroup* self) { self->subscribeTo(); })
      .def(
          "removeShapeFrame",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::ShapeFrame* shapeFrame) {
            self->removeShapeFrame(shapeFrame);
          },
          nb::arg("shapeFrame").none())
      .def(
          "removeShapeFrames",
          +[](dart::collision::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) { self->removeShapeFrames(shapeFrames); },
          nb::arg("shapeFrames"))
      .def(
          "removeShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dynamics::ShapeFrame* shapeFrame) {
            self->removeShapeFramesOf(shapeFrame);
          },
          nb::arg("shapeFrame").none(),
          "Removes a ShapeFrame")
      .def(
          "removeShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const std::vector<const dart::dynamics::ShapeFrame*>&
                  shapeFrames) { self->removeShapeFramesOf(shapeFrames); },
          nb::arg("shapeFrames"),
          "Removes ShapeFrames")
      .def(
          "removeShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::CollisionGroup* otherGroup) {
            self->removeShapeFramesOf(otherGroup);
          },
          nb::arg("otherGroup").none(),
          "Removes ShapeFrames of other CollisionGroup")
      .def(
          "removeShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::BodyNode* body) {
            self->removeShapeFramesOf(body);
          },
          nb::arg("body").none(),
          "Removes ShapeFrames of BodyNode")
      .def(
          "removeShapeFramesOf",
          +[](dart::collision::CollisionGroup* self,
              const dart::dynamics::MetaSkeleton* skeleton) {
            self->removeShapeFramesOf(skeleton);
          },
          nb::arg("skeleton").none(),
          "Removes ShapeFrames of MetaSkeleton")
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
          nb::arg("shapeFrame").none())
      .def(
          "getNumShapeFrames",
          +[](const dart::collision::CollisionGroup* self) -> std::size_t {
            return self->getNumShapeFrames();
          })
      .def(
          "collide",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::CollisionOption& option,
              dart::collision::CollisionResult* result) -> bool {
            return self->collide(option, result);
          },
          nb::arg("option")
          = dart::collision::CollisionOption(false, 1u, nullptr),
          nb::arg("result").none() = nullptr,
          "Performs collision check within this CollisionGroup")
      .def(
          "collide",
          +[](dart::collision::CollisionGroup* self,
              dart::collision::CollisionGroup* otherGroup,
              const dart::collision::CollisionOption& option,
              dart::collision::CollisionResult* result) -> bool {
            return self->collide(otherGroup, option, result);
          },
          nb::arg("otherGroup").none(),
          nb::arg("option")
          = dart::collision::CollisionOption(false, 1u, nullptr),
          nb::arg("result").none() = nullptr,
          "Perform collision check against other CollisionGroup")
      .def(
          "distance",
          +[](dart::collision::CollisionGroup* self,
              const dart::collision::DistanceOption& option,
              dart::collision::DistanceResult* result) -> double {
            return self->distance(option, result);
          },
          nb::arg("option")
          = dart::collision::DistanceOption(false, 0.0, nullptr),
          nb::arg("result").none() = nullptr)
      .def(
          "raycast",
          +[](dart::collision::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to) -> bool {
            return self->raycast(from, to);
          },
          nb::arg("from"),
          nb::arg("to"))
      .def(
          "raycast",
          +[](dart::collision::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to,
              const dart::collision::RaycastOption& option) -> bool {
            return self->raycast(from, to, option);
          },
          nb::arg("from"),
          nb::arg("to"),
          nb::arg("option"))
      .def(
          "raycast",
          +[](dart::collision::CollisionGroup* self,
              const Eigen::Vector3d& from,
              const Eigen::Vector3d& to,
              const dart::collision::RaycastOption& option,
              dart::collision::RaycastResult* result) -> bool {
            return self->raycast(from, to, option, result);
          },
          nb::arg("from"),
          nb::arg("to"),
          nb::arg("option"),
          nb::arg("result").none())
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
          nb::arg("automatic"))
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
