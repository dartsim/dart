/*
 * Copyright (c) 2011-2021, The DART development contributors
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

void CollisionObject(py::module& m)
{
  ::py::class_<dart::collision::CollisionObject>(m, "CollisionObject")
      .def(
          "getCollisionDetector",
          +[](dart::collision::CollisionObject* self)
              -> dart::collision::CollisionDetector* {
            return self->getCollisionDetector();
          },
          ::py::return_value_policy::reference_internal,
          "Return collision detection engine associated with this "
          "CollisionObject.")
      .def(
          "getCollisionDetector",
          +[](const dart::collision::CollisionObject* self)
              -> const dart::collision::CollisionDetector* {
            return self->getCollisionDetector();
          },
          ::py::return_value_policy::reference_internal,
          "Return collision detection engine associated with this "
          "CollisionObject.")
      .def(
          "getShapeFrame",
          +[](const dart::collision::CollisionObject* self)
              -> const dynamics::ShapeFrame* { return self->getShapeFrame(); },
          ::py::return_value_policy::reference_internal,
          "Return the associated ShapeFrame.")
      .def(
          "getShape",
          +[](const dart::collision::CollisionObject* self)
              -> dart::dynamics::ConstShapePtr { return self->getShape(); },
          "Return the associated Shape.")
      .def(
          "getTransform",
          +[](const dart::collision::CollisionObject* self)
              -> const Eigen::Isometry3d& { return self->getTransform(); },
          ::py::return_value_policy::reference_internal,
          "Return the transformation of this CollisionObject in world "
          "coordinates.");
}

} // namespace python
} // namespace dart
