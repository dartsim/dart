// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/config.hpp>

namespace dart {
namespace python {

void Contact(nb::module_& sm);

void CollisionFilter(nb::module_& sm);
void CollisionObject(nb::module_& sm);
void CollisionOption(nb::module_& sm);
void CollisionResult(nb::module_& sm);

void DistanceOption(nb::module_& sm);
void DistanceResult(nb::module_& sm);

void RaycastOption(nb::module_& sm);
void RaycastResult(nb::module_& sm);

void CollisionDetector(nb::module_& sm);
void FCLCollisionDetector(nb::module_& sm);
void DARTCollisionDetector(nb::module_& sm);

void CollisionGroup(nb::module_& sm);
void FCLCollisionGroup(nb::module_& sm);
void DARTCollisionGroup(nb::module_& sm);

#if HAVE_BULLET
void BulletCollisionDetector(nb::module_& sm);
void BulletCollisionGroup(nb::module_& sm);
#endif // HAVE_BULLET

#if HAVE_ODE
void OdeCollisionDetector(nb::module_& sm);
void OdeCollisionGroup(nb::module_& sm);
#endif // HAVE_ODE

void dart_collision(nb::module_& m)
{
  auto sm = m.def_submodule("collision");

  Contact(sm);

  CollisionFilter(sm);
  CollisionObject(sm);
  CollisionOption(sm);
  CollisionResult(sm);

  DistanceOption(sm);
  DistanceResult(sm);

  RaycastOption(sm);
  RaycastResult(sm);

  CollisionDetector(sm);
  FCLCollisionDetector(sm);
  DARTCollisionDetector(sm);

  CollisionGroup(sm);
  FCLCollisionGroup(sm);
  DARTCollisionGroup(sm);

#if HAVE_BULLET
  BulletCollisionDetector(sm);
  BulletCollisionGroup(sm);
#endif // HAVE_BULLET

#if HAVE_ODE
  OdeCollisionDetector(sm);
  OdeCollisionGroup(sm);
#endif // HAVE_ODE
}

} // namespace python
} // namespace dart
