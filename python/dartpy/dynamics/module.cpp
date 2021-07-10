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

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

void Shape(py::module& sm);

void Entity(py::module& sm);
void Frame(py::module& sm);
void ShapeFrame(py::module& sm);
void SimpleFrame(py::module& sm);

void Node(py::module& sm);
void JacobianNode(py::module& sm);
void ShapeNode(py::module& sm);

void DegreeOfFreedom(py::module& sm);

void BodyNode(py::module& sm);

void Joint(py::module& sm);
void ZeroDofJoint(py::module& sm);
void WeldJoint(py::module& sm);
void GenericJoint(py::module& sm);
void RevoluteJoint(py::module& sm);
void PrismaticJoint(py::module& sm);
void ScrewJoint(py::module& sm);
void UniversalJoint(py::module& sm);
void TranslationalJoint2D(py::module& sm);
void PlanarJoint(py::module& sm);
void EulerJoint(py::module& sm);
void BallJoint(py::module& sm);
void TranslationalJoint(py::module& sm);
void FreeJoint(py::module& sm);

void MetaSkeleton(py::module& sm);
void ReferentialSkeleton(py::module& sm);
void Linkage(py::module& sm);
void Chain(py::module& sm);
void Skeleton(py::module& sm);

void InverseKinematics(py::module& sm);
void Inertia(py::module& sm);

void ConstraintBase(py::module& sm);
void JointConstraint(py::module& sm);
void JointCoulombFrictionConstraint(py::module& sm);
void DynamicJointConstraint(py::module& sm);

void BoxedLcpSolver(py::module& sm);
void DantzigBoxedLcpSolver(py::module& sm);
void PgsBoxedLcpSolver(py::module& sm);

void ConstraintSolver(py::module& sm);
void BoxedLcpConstraintSolver(py::module& sm);

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

#if DART_HAVE_BULLET
void BulletCollisionDetector(py::module& sm);
void BulletCollisionGroup(py::module& sm);
#endif // DART_HAVE_BULLET

#if DART_HAVE_ODE
void OdeCollisionDetector(py::module& sm);
void OdeCollisionGroup(py::module& sm);
#endif // DART_HAVE_ODE

void dart_dynamics(py::module& m)
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

  Inertia(sm);

  ConstraintBase(sm);
  JointConstraint(sm);
  JointCoulombFrictionConstraint(sm);
  DynamicJointConstraint(sm);

  BoxedLcpSolver(sm);
  DantzigBoxedLcpSolver(sm);
  PgsBoxedLcpSolver(sm);

  ConstraintSolver(sm);
  BoxedLcpConstraintSolver(sm);

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

#if DART_HAVE_BULLET
  BulletCollisionDetector(sm);
  BulletCollisionGroup(sm);
#endif // DART_HAVE_BULLET

#if DART_HAVE_ODE
  OdeCollisionDetector(sm);
  OdeCollisionGroup(sm);
#endif // DART_HAVE_ODE
}

} // namespace python
} // namespace dart
