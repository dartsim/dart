/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_DYNAMICS_PTR_H_
#define DART_DYNAMICS_PTR_H_

#include "dart/dynamics/BodyNodePtr.impl"
#include "dart/dynamics/JointPtr.impl"
#include "dart/dynamics/DegreeOfFreedomPtr.impl"

// This file is a lightweight means of providing the smart pointers which are
// commonly used within the dart::dynamics namespace. It is 'lightweight' in the
// sense that it does not depend on any types being fully defined, making this
// header suitable for inclusion in other headers which might only want access
// to the smart pointers without needing fully defined classes.

namespace dart {
namespace dynamics {

// Skeleton smart pointers
class Skeleton;
typedef std::shared_ptr<Skeleton>       SkeletonPtr;
typedef std::shared_ptr<const Skeleton> ConstSkeletonPtr;
typedef std::weak_ptr<Skeleton>         WeakSkeletonPtr;
typedef std::weak_ptr<const Skeleton>   WeakConstSkeletonPtr;

// MetaSkeleton smart pointers
class MetaSkeleton;
typedef std::shared_ptr<MetaSkeleton>       MetaSkeletonPtr;
typedef std::shared_ptr<const MetaSkeleton> ConstMetaSkeletonPtr;
typedef std::weak_ptr<MetaSkeleton>         WeakMetaSkeletonptr;
typedef std::weak_ptr<const MetaSkeleton>   WeakConstMetaSkeletonPtr;

// BodyNode smart pointers
class BodyNode;
typedef TemplateBodyNodePtr<BodyNode>           BodyNodePtr;
typedef TemplateBodyNodePtr<const BodyNode>     ConstBodyNodePtr;
typedef TemplateWeakBodyNodePtr<BodyNode>       WeakBodyNodePtr;
typedef TemplateWeakBodyNodePtr<const BodyNode> WeakConstBodyNodePtr;

// SoftBodyNode smart pointers
class SoftBodyNode;
typedef TemplateBodyNodePtr<SoftBodyNode>           SoftBodyNodePtr;
typedef TemplateBodyNodePtr<const SoftBodyNode>     ConstSoftBodyNodePtr;
typedef TemplateWeakBodyNodePtr<SoftBodyNode>       WeakSoftBodyNodePtr;
typedef TemplateWeakBodyNodePtr<const SoftBodyNode> WeakConstSoftBodyNodePtr;

// Joint smart pointers
class Joint;
typedef TemplateJointPtr<Joint, BodyNode>                 JointPtr;
typedef TemplateJointPtr<const Joint, const BodyNode>     ConstJointPtr;
typedef TemplateWeakJointPtr<Joint, BodyNode>             WeakJointPtr;
typedef TemplateWeakJointPtr<const Joint, const BodyNode> WeakConstJointPtr;

// DegreeOfFreedom smart pointers
class DegreeOfFreedom;
typedef TemplateDegreeOfFreedomPtr<DegreeOfFreedom, BodyNode>                 DegreeOfFreedomPtr;
typedef TemplateDegreeOfFreedomPtr<const DegreeOfFreedom, const BodyNode>     ConstDegreeOfFreedomPtr;
typedef TemplateWeakDegreeOfFreedomPtr<DegreeOfFreedom, BodyNode>             WeakDegreeOfFreedomPtr;
typedef TemplateWeakDegreeOfFreedomPtr<const DegreeOfFreedom, const BodyNode> WeakConstDegreeOfFreedomPtr;

// Shape smart pointers
class Shape;
typedef std::shared_ptr<Shape>       ShapePtr;
typedef std::shared_ptr<const Shape> ConstShapePtr;
typedef std::weak_ptr<Shape>         WeakShapePtr;
typedef std::weak_ptr<const Shape>   WeakConstShapePtr;

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_PTR_H_
