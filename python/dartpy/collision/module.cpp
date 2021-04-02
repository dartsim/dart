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

#include <dart/config.hpp>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void Contact(py::module& sm);

void CollisionFilter(py::module& sm);
void CollisionObject(py::module& sm);
void CollisionOption(py::module& sm);
void CollisionResult(py::module& sm);

void DistanceOption(py::module& sm);
void DistanceResult(py::module& sm);

void RaycastOption(py::module& sm);
void RaycastResult(py::module& sm);

void CollisionDetector(py::module& sm);
void FCLCollisionDetector(py::module& sm);
void DARTCollisionDetector(py::module& sm);

void CollisionGroup(py::module& sm);
void FCLCollisionGroup(py::module& sm);
void DARTCollisionGroup(py::module& sm);

#if HAVE_BULLET
void BulletCollisionDetector(py::module& sm);
void BulletCollisionGroup(py::module& sm);
#endif // HAVE_BULLET

#if HAVE_ODE
void OdeCollisionDetector(py::module& sm);
void OdeCollisionGroup(py::module& sm);
#endif // HAVE_ODE

void dart_collision(py::module& m)
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
