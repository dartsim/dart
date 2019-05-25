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

void Shape(pybind11::module& sm);

void Entity(pybind11::module& sm);
void Frame(pybind11::module& sm);
void ShapeFrame(pybind11::module& sm);
void SimpleFrame(pybind11::module& sm);

void Node(pybind11::module& sm);
void JacobianNode(pybind11::module& sm);
void ShapeNode(pybind11::module& sm);

void DegreeOfFreedom(pybind11::module& sm);

void BodyNode(pybind11::module& sm);

void Joint(pybind11::module& sm);
void ZeroDofJoint(pybind11::module& sm);
void WeldJoint(pybind11::module& sm);
void GenericJoint(pybind11::module& sm);
void RevoluteJoint(pybind11::module& sm);
void PrismaticJoint(pybind11::module& sm);
void ScrewJoint(pybind11::module& sm);
void UniversalJoint(pybind11::module& sm);
void TranslationalJoint2D(pybind11::module& sm);
void PlanarJoint(pybind11::module& sm);
void EulerJoint(pybind11::module& sm);
void BallJoint(pybind11::module& sm);
void TranslationalJoint(pybind11::module& sm);
void FreeJoint(pybind11::module& sm);

void MetaSkeleton(pybind11::module& sm);
void ReferentialSkeleton(pybind11::module& sm);
void Linkage(pybind11::module& sm);
void Chain(pybind11::module& sm);
void Skeleton(pybind11::module& sm);

void InverseKinematics(pybind11::module& sm);

void dart_dynamics(pybind11::module& m)
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
}

} // namespace python
} // namespace dart
