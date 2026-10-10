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

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/RaycastResult.hpp>

namespace dart {
namespace python {

void RaycastResult(nb::module_& m)
{
  dartnb::dart_class<dart::collision::RayHit>(m, "RayHit")
      .def(dartnb::init<>())
      .def_rw(
          "mCollisionObject",
          &dart::collision::RayHit::mCollisionObject,
          dartnb::setterArgument(&dart::collision::RayHit::mCollisionObject))
      .def_rw(
          "mNormal",
          &dart::collision::RayHit::mNormal,
          dartnb::setterArgument(&dart::collision::RayHit::mNormal))
      .def_rw(
          "mPoint",
          &dart::collision::RayHit::mPoint,
          dartnb::setterArgument(&dart::collision::RayHit::mPoint))
      .def_rw(
          "mFraction",
          &dart::collision::RayHit::mFraction,
          dartnb::setterArgument(&dart::collision::RayHit::mFraction));

  dartnb::dart_class<dart::collision::RaycastResult>(m, "RaycastResult")
      .def(dartnb::init<>())
      .def(
          "clear", +[](dart::collision::RaycastResult* self) { self->clear(); })
      .def(
          "hasHit",
          +[](const dart::collision::RaycastResult* self) -> bool {
            return self->hasHit();
          })
      .def_rw(
          "mRayHits",
          &dart::collision::RaycastResult::mRayHits,
          dartnb::setterArgument(&dart::collision::RaycastResult::mRayHits));
}

} // namespace python
} // namespace dart
