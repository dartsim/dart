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

#include <pybind11/pybind11.h>

namespace dart {
namespace python {

void ConstraintBase(pybind11::module& sm);
void JointConstraint(pybind11::module& sm);
void JointLimitConstraint(pybind11::module& sm);
void JointCoulombFrictionConstraint(pybind11::module& sm);

void BoxedLcpSolver(pybind11::module& sm);
void DantzigBoxedLcpSolver(pybind11::module& sm);
void PgsBoxedLcpSolver(pybind11::module& sm);

void ConstraintSolver(pybind11::module& sm);
void BoxedLcpConstraintSolver(pybind11::module& sm);

// void ConstraintBase(pybind11::module& sm);
// void ConstraintBase(pybind11::module& sm);
// void ConstraintBase(pybind11::module& sm);

void dart_constraint(pybind11::module& m)
{
  auto sm = m.def_submodule("constraint");

  ConstraintBase(sm);
  JointConstraint(sm);
  JointLimitConstraint(sm);
  JointCoulombFrictionConstraint(sm);

  BoxedLcpSolver(sm);
  DantzigBoxedLcpSolver(sm);
  PgsBoxedLcpSolver(sm);

  ConstraintSolver(sm);
  BoxedLcpConstraintSolver(sm);

  // ConstraintBase(sm);
  // ConstraintBase(sm);
  // ConstraintBase(sm);
  // ConstraintBase(sm);
  // ConstraintBase(sm);
  // ConstraintBase(sm);
  // ConstraintBase(sm);
}

} // namespace python
} // namespace dart
