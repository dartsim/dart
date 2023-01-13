/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#pragma once

#include <dart/collision/Export.hpp>

#include <dart/dynamics/Export.hpp>
#include <dart/dynamics/detail/BodyNodePtr.hpp>
#include <dart/dynamics/detail/DegreeOfFreedomPtr.hpp>
#include <dart/dynamics/detail/InverseKinematicsPtr.hpp>
#include <dart/dynamics/detail/JointPtr.hpp>
#include <dart/dynamics/detail/NodePtr.hpp>

#include <dart/math/Fwd.hpp>

namespace dart::collision {

DART_DECLARE_CLASS_POINTERS(CollisionDetector)
DART_DECLARE_CLASS_POINTERS(FCLCollisionDetector)
DART_DECLARE_CLASS_POINTERS(DARTCollisionDetector)

DART_DECLARE_CLASS_POINTERS(CollisionObject)
DART_DECLARE_CLASS_POINTERS(CollisionGroup)

class CollisionResult;

} // namespace dart::collision

namespace dart::dynamics {

DART_DECLARE_CLASS_POINTERS(ShapeFrame)
DART_DECLARE_CLASS_POINTERS(SimpleFrame)

DART_DECLARE_CLASS_POINTERS(NodeDestructor)

//-----------------------------------------------------------------------------
// Skeleton Smart Pointers
//-----------------------------------------------------------------------------
DART_DECLARE_CLASS_POINTERS(Skeleton)
// These pointers will take the form of:
// std::shared_ptr<Skeleton>        --> SkeletonPtr
// std::shared_ptr<const Skeleton>  --> ConstSkeletonPtr
// std::weak_ptr<Skeleton>          --> WeakSkeletonPtr
// std::weak_ptr<const Skeleton>    --> WeakConstSkeletonPtr

// MetaSkeleton smart pointers
DART_DECLARE_CLASS_POINTERS(MetaSkeleton)

// ReferentialSkeleton smart pointers
DART_DECLARE_CLASS_POINTERS(ReferentialSkeleton)

DART_DECLARE_CLASS_POINTERS(Group)
DART_DECLARE_CLASS_POINTERS(Linkage)
DART_DECLARE_CLASS_POINTERS(Branch)
DART_DECLARE_CLASS_POINTERS(Chain)

//-----------------------------------------------------------------------------
// Shape Smart Pointers
//-----------------------------------------------------------------------------
DART_DECLARE_CLASS_POINTERS(Shape)
DART_DECLARE_CLASS_POINTERS(ArrowShape)
DART_DECLARE_CLASS_POINTERS(BoxShape)
DART_DECLARE_CLASS_POINTERS(CylinderShape)
DART_DECLARE_CLASS_POINTERS(EllipsoidShape)
DART_DECLARE_CLASS_POINTERS(LineSegmentShape)
DART_DECLARE_CLASS_POINTERS(MeshShape)
DART_DECLARE_CLASS_POINTERS(PlaneShape)
DART_DECLARE_CLASS_POINTERS(SoftMeshShape)

//-----------------------------------------------------------------------------
// BodyNode Smart Pointers
//-----------------------------------------------------------------------------
#define DART_DYNAMICS_MAKE_BODYNODEPTR(X)                                      \
  class X;                                                                     \
  using X##Ptr = TemplateBodyNodePtr<X>;                                       \
  using Const##X##Ptr = TemplateBodyNodePtr<const X>;                          \
  using Weak##X##Ptr = TemplateWeakBodyNodePtr<X>;                             \
  using WeakConst##X##Ptr = TemplateWeakBodyNodePtr<const X>;

// BodyNode smart pointers
DART_DYNAMICS_MAKE_BODYNODEPTR(BodyNode)
// These pointers will take the form of:
// TemplateBodyNodePtr<BodyNode>            --> BodyNodePtr
// TemplateBodyNodePtr<const BodyNode>      --> ConstBodyNodePtr
// TemplateWeakBodyNodePtr<BodyNode>        --> WeakBodyNodePtr
// TemplateWeakBodyNodePtr<const BodyNode>  --> WeakConstBodyNodePtr

// SoftBodyNode smart pointers
DART_DYNAMICS_MAKE_BODYNODEPTR(SoftBodyNode)

//-----------------------------------------------------------------------------
// BodyNode-dependent Smart Pointers
//-----------------------------------------------------------------------------
#define DART_DYNAMICS_MAKE_BN_DEPENDENT_PTR(X)                                 \
  class X;                                                                     \
  using X##Ptr = Template##X##Ptr<X, BodyNode>;                                \
  using Const##X##Ptr = Template##X##Ptr<const X, const BodyNode>;             \
  using Weak##X##Ptr = TemplateWeak##X##Ptr<X, BodyNode>;                      \
  using WeakConst##X##Ptr = TemplateWeak##X##Ptr<const X, const BodyNode>;

// Joint smart pointers
DART_DYNAMICS_MAKE_BN_DEPENDENT_PTR(Joint)
// These pointers will take the form of:
// TemplateJointPtr<Joint>            --> JointPtr
// TemplateJointPtr<const Joint>      --> ConstJointPtr
// TemplateWeakJointPtr<Joint>        --> WeakJointPtr
// TemplateWeakJointPtr<const Joint>  --> WeakConstJointPtr

// DegreeOfFreedom smart pointers
DART_DYNAMICS_MAKE_BN_DEPENDENT_PTR(DegreeOfFreedom)
// These pointers will take the form of:
// TemplateDegreeOfFreedomPtr<DegreeOfFreedom>       --> DegreeOfFreedomPtr
// TemplateDegreeOfFreedomPtr<const DegreeOfFreedom> --> ConstDegreeOfFreedomPtr
// TemplateWeakDegreeOfFreedomPtr<DegreeOfFreedom>   --> WeakDegreeOfFreedomPtr
// TemplateWeakDegreeOfFreedomPtr<const DegreeOfFreedom>
//                                               --> WeakConstDegreeOfFreedomPtr

//-----------------------------------------------------------------------------
// Node Smart Pointers
//-----------------------------------------------------------------------------
#define DART_DYNAMICS_MAKE_NODEPTR(X)                                          \
  class X;                                                                     \
  using X##Ptr = TemplateNodePtr<X, BodyNode>;                                 \
  using Const##X##Ptr = TemplateNodePtr<const X, const BodyNode>;              \
  using Weak##X##Ptr = TemplateWeakNodePtr<X, BodyNode>;                       \
  using WeakConst##X##Ptr = TemplateWeakNodePtr<const X, const BodyNode>;

DART_DYNAMICS_MAKE_NODEPTR(Node)
// These pointers will take the form of:
// TemplateNodePtr<Node>            --> NodePtr
// TemplateNodePtr<const Node>      --> ConstNodePtr
// TemplateWeakNodePtr<Node>        --> WeakNodePtr
// TemplateWeakNodePtr<const Node>  --> WeakConstNodePtr

DART_DYNAMICS_MAKE_NODEPTR(JacobianNode)

DART_DYNAMICS_MAKE_NODEPTR(EndEffector)
// These pointers will take the form of:
// TemplateNodePtr<EndEffector>             --> EndEffectorPtr
// TemplateNodePtr<const EndEffector>       --> ConstEndEffectorPtr
// TemplateWeakNodePtr<EndEffector>         --> WeakEndEffectorPtr
// TemplateWeakNodePtr<const EndEffector>   --> WeakConstEndEffectorPtr

DART_DYNAMICS_MAKE_NODEPTR(ShapeNode)

//-----------------------------------------------------------------------------
// InverseKinematics Smart Pointers
//-----------------------------------------------------------------------------
#define DART_DYNAMICS_MAKE_IK_PTR(X)                                           \
  class X;                                                                     \
  using X##Ptr = TemplateInverseKinematicsPtr<X, JacobianNodePtr>;             \
  using Const##X##Ptr                                                          \
      = TemplateInverseKinematicsPtr<const X, ConstJacobianNodePtr>;           \
  using Weak##X##Ptr = TemplateWeakInverseKinematicsPtr<X, JacobianNodePtr>;   \
  using WeakConst##X##Ptr                                                      \
      = TemplateWeakInverseKinematicsPtr<const X, ConstJacobianNodePtr>;

DART_DYNAMICS_MAKE_IK_PTR(InverseKinematics)

//-----------------------------------------------------------------------------
// Constraints Smart Pointers
//-----------------------------------------------------------------------------

DART_DECLARE_CLASS_POINTERS(ConstraintSolver)

DART_DECLARE_CLASS_POINTERS(ConstrainedGroup)

DART_DECLARE_CLASS_POINTERS(ConstraintBase)
DART_DECLARE_CLASS_POINTERS(ClosedLoopConstraint)
DART_DECLARE_CLASS_POINTERS(ContactConstraint)
DART_DECLARE_CLASS_POINTERS(ContactSurfaceHandler)
DART_DECLARE_CLASS_POINTERS(SoftContactConstraint)
DART_DECLARE_CLASS_POINTERS(JointConstraint)
DART_DECLARE_CLASS_POINTERS(MimicMotorConstraint)
DART_DECLARE_CLASS_POINTERS(JointCoulombFrictionConstraint)

DART_DECLARE_CLASS_POINTERS(LCPSolver)
DART_DECLARE_CLASS_POINTERS(BoxedLcpSolver)
DART_DECLARE_CLASS_POINTERS(PgsBoxedLcpSolver)
DART_DECLARE_CLASS_POINTERS(PsorBoxedLcpSolver)
DART_DECLARE_CLASS_POINTERS(JacobiBoxedLcpSolver)

DART_DECLARE_CLASS_POINTERS(DynamicJointConstraint)
DART_DECLARE_CLASS_POINTERS(BallJointConstraint)
DART_DECLARE_CLASS_POINTERS(WeldJointConstraint)

DART_DECLARE_CLASS_POINTERS(BalanceConstraint)

} // namespace dart::dynamics
