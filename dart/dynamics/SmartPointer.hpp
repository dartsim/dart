/*
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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
#include "dart/dynamics/detail/JointPtr.hpp"
#include "dart/dynamics/detail/DegreeOfFreedomPtr.hpp"

// This file is a lightweight means of providing the smart pointers which are
// commonly used within the dart::dynamics namespace. It is 'lightweight' in the
// sense that it does not depend on any types being fully defined, making this
// header suitable for inclusion in other headers which might only want access
// to the smart pointers without needing fully defined classes.

namespace dart {
namespace dynamics {

DART_COMMON_MAKE_SHARED_WEAK(ShapeFrame)
DART_COMMON_MAKE_SHARED_WEAK(SimpleFrame)

//-----------------------------------------------------------------------------
// Skeleton Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_MAKE_SHARED_WEAK(Skeleton)
// These pointers will take the form of:
// std::shared_ptr<Skeleton>        --> SkeletonPtr
// std::shared_ptr<const Skeleton>  --> ConstSkeletonPtr
// std::weak_ptr<Skeleton>          --> WeakSkeletonPtr
// std::weak_ptr<const Skeleton>    --> WeakConstSkeletonPtr

// MetaSkeleton smart pointers
DART_COMMON_MAKE_SHARED_WEAK(MetaSkeleton)

// ReferentialSkeleton smart pointers
DART_COMMON_MAKE_SHARED_WEAK(ReferentialSkeleton)

DART_COMMON_MAKE_SHARED_WEAK(Group)
DART_COMMON_MAKE_SHARED_WEAK(Linkage)
DART_COMMON_MAKE_SHARED_WEAK(Branch)
DART_COMMON_MAKE_SHARED_WEAK(Chain)

//-----------------------------------------------------------------------------
// Shape Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_MAKE_SHARED_WEAK(Shape)
DART_COMMON_MAKE_SHARED_WEAK(ArrowShape)
DART_COMMON_MAKE_SHARED_WEAK(BoxShape)
DART_COMMON_MAKE_SHARED_WEAK(CylinderShape)
DART_COMMON_MAKE_SHARED_WEAK(EllipsoidShape)
DART_COMMON_MAKE_SHARED_WEAK(LineSegmentShape)
DART_COMMON_MAKE_SHARED_WEAK(MeshShape)
DART_COMMON_MAKE_SHARED_WEAK(PlaneShape)
DART_COMMON_MAKE_SHARED_WEAK(SoftMeshShape)


//-----------------------------------------------------------------------------
// BodyNode Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_MAKE_SHARED_WEAK(BodyNode)
DART_COMMON_MAKE_SHARED_WEAK(SoftBodyNode)


//-----------------------------------------------------------------------------
// BodyNode-dependent Smart Pointers
//-----------------------------------------------------------------------------
#define DART_DYNAMICS_MAKE_BN_DEPENDENT_PTR( X )                                          \
  class X ;                                                                               \
  typedef Template ## X ## Ptr < X , BodyNode >                   X ## Ptr;               \
  typedef Template ## X ## Ptr < const X , const BodyNode >       Const ## X ## Ptr;      \
  typedef TemplateWeak ## X ## Ptr < X , BodyNode >               Weak ## X ## Ptr;       \
  typedef TemplateWeak ## X ## Ptr < const X , const BodyNode >   WeakConst ## X ## Ptr;


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
// TemplateDegreeOfFreedomPtr<DegreeOfFreedom>            --> DegreeOfFreedomPtr
// TemplateDegreeOfFreedomPtr<const DegreeOfFreedom>      --> ConstDegreeOfFreedomPtr
// TemplateWeakDegreeOfFreedomPtr<DegreeOfFreedom>        --> WeakDegreeOfFreedomPtr
// TemplateWeakDegreeOfFreedomPtr<const DegreeOfFreedom>  --> WeakConstDegreeOfFreedomPtr


//-----------------------------------------------------------------------------
// Node Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_MAKE_SHARED_WEAK(Node)
DART_COMMON_MAKE_SHARED_WEAK(JacobianNode)
DART_COMMON_MAKE_SHARED_WEAK(EndEffector)
DART_COMMON_MAKE_SHARED_WEAK(ShapeNode)


//-----------------------------------------------------------------------------
// InverseKinematics Smart Pointers
//-----------------------------------------------------------------------------
DART_COMMON_MAKE_SHARED_WEAK(InverseKinematics)

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SMARTPOINTER_HPP_
