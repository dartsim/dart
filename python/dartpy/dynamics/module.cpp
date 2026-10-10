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

namespace dart {
namespace python {

void Shape(nb::module_& sm);

void Entity(nb::module_& sm);
void Frame(nb::module_& sm);
void ShapeFrame(nb::module_& sm);
void SimpleFrame(nb::module_& sm);

void Node(nb::module_& sm);
void JacobianNode(nb::module_& sm);
void ShapeNode(nb::module_& sm);

void DegreeOfFreedom(nb::module_& sm);

void BodyNode(nb::module_& sm);
void SoftBodyNode(nb::module_& sm);

void Joint(nb::module_& sm);
void ZeroDofJoint(nb::module_& sm);
void WeldJoint(nb::module_& sm);
void GenericJoint(nb::module_& sm);
void RevoluteJoint(nb::module_& sm);
void PrismaticJoint(nb::module_& sm);
void ScrewJoint(nb::module_& sm);
void UniversalJoint(nb::module_& sm);
void TranslationalJoint2D(nb::module_& sm);
void PlanarJoint(nb::module_& sm);
void EulerJoint(nb::module_& sm);
void BallJoint(nb::module_& sm);
void TranslationalJoint(nb::module_& sm);
void FreeJoint(nb::module_& sm);

void MetaSkeleton(nb::module_& sm);
void ReferentialSkeleton(nb::module_& sm);
void Linkage(nb::module_& sm);
void Chain(nb::module_& sm);
void Skeleton(nb::module_& sm);

void InverseKinematics(nb::module_& sm);
void ContactInverseDynamics(nb::module_& sm);
void Inertia(nb::module_& sm);

void dart_dynamics(nb::module_& m)
{
  auto sm = m.def_submodule("dynamics");

  Shape(sm);

  Entity(sm);
  Frame(sm);
  ShapeFrame(sm);
  SimpleFrame(sm);

  Node(sm);
  JacobianNode(sm);
  ShapeNode(sm);

  DegreeOfFreedom(sm);

  BodyNode(sm);
  SoftBodyNode(sm);

  Joint(sm);
  ZeroDofJoint(sm);
  WeldJoint(sm);
  GenericJoint(sm);
  RevoluteJoint(sm);
  PrismaticJoint(sm);
  ScrewJoint(sm);
  UniversalJoint(sm);
  TranslationalJoint2D(sm);
  PlanarJoint(sm);
  EulerJoint(sm);
  BallJoint(sm);
  TranslationalJoint(sm);
  FreeJoint(sm);

  MetaSkeleton(sm);
  ReferentialSkeleton(sm);
  Linkage(sm);
  Chain(sm);
  Skeleton(sm);

  InverseKinematics(sm);
  ContactInverseDynamics(sm);

  Inertia(sm);
}

} // namespace python
} // namespace dart
