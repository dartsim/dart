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

#ifndef DART_DYNAMICS_SMARTPOINTER_HPP_
#define DART_DYNAMICS_SMARTPOINTER_HPP_

#include "dart/common/SmartPointer.hpp"
#include "dart/dynamics/detail/BodyNodePtr.hpp"
#include "dart/dynamics/detail/DegreeOfFreedomPtr.hpp"
#include "dart/dynamics/detail/InverseKinematicsPtr.hpp"
#include "dart/dynamics/detail/JointPtr.hpp"
#include "dart/dynamics/detail/NodePtr.hpp"

// This file is a lightweight means of providing the smart pointers which are
// commonly used within the dart::dynamics namespace. It is 'lightweight' in the
// sense that it does not depend on any types being fully defined, making this
// header suitable for inclusion in other headers which might only want access
// to the smart pointers without needing fully defined classes.

namespace dart {
namespace dynamics {

DART_COMMON_DECLARE_SHARED_WEAK(ShapeFrame)
DART_COMMON_DECLARE_SHARED_WEAK(SimpleFrame)

DART_COMMON_DECLARE_SHARED_WEAK(NodeDestructor)

//-----------------------------------------------------------------------------
// Skeleton Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_DECLARE_SHARED_WEAK(Skeleton)
// These pointers will take the form of:
// std::shared_ptr<Skeleton>        --> SkeletonPtr
// std::shared_ptr<const Skeleton>  --> ConstSkeletonPtr
// std::weak_ptr<Skeleton>          --> WeakSkeletonPtr
// std::weak_ptr<const Skeleton>    --> WeakConstSkeletonPtr

// MetaSkeleton smart pointers
DART_COMMON_DECLARE_SHARED_WEAK(MetaSkeleton)

// ReferentialSkeleton smart pointers
DART_COMMON_DECLARE_SHARED_WEAK(ReferentialSkeleton)

DART_COMMON_DECLARE_SHARED_WEAK(Group)
DART_COMMON_DECLARE_SHARED_WEAK(Linkage)
DART_COMMON_DECLARE_SHARED_WEAK(Branch)
DART_COMMON_DECLARE_SHARED_WEAK(Chain)

//-----------------------------------------------------------------------------
// Shape Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_DECLARE_SHARED_WEAK(Shape)
DART_COMMON_DECLARE_SHARED_WEAK(ArrowShape)
DART_COMMON_DECLARE_SHARED_WEAK(BoxShape)
DART_COMMON_DECLARE_SHARED_WEAK(CylinderShape)
DART_COMMON_DECLARE_SHARED_WEAK(EllipsoidShape)
DART_COMMON_DECLARE_SHARED_WEAK(LineSegmentShape)
DART_COMMON_DECLARE_SHARED_WEAK(MeshShape)
DART_COMMON_DECLARE_SHARED_WEAK(PlaneShape)
DART_COMMON_DECLARE_SHARED_WEAK(SoftMeshShape)

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

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SMARTPOINTER_HPP_
