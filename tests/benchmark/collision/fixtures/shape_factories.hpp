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

#pragma once

#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <Eigen/Geometry>

#include <memory>

namespace dart::benchmark::collision {

inline Eigen::Isometry3d MakeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  return tf;
}

inline dynamics::SkeletonPtr CreateSingleShapeSkeleton(
    const std::shared_ptr<dynamics::Shape>& shape,
    const Eigen::Isometry3d& transform)
{
  auto skeleton = dynamics::Skeleton::create();
  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  auto* body = pair.second;

  body->createShapeNodeWith<dynamics::VisualAspect, dynamics::CollisionAspect>(
      shape);

  pair.first->setTransform(transform);

  return skeleton;
}

inline void AddSkeletonToGroup(
    collision::CollisionGroup* group, const dynamics::SkeletonPtr& skeleton)
{
  group->addShapeFramesOf(skeleton.get());
}

inline collision::CollisionOption MakeCollisionOption(
    std::size_t max_contacts = 1000)
{
  collision::CollisionOption option;
  option.maxNumContacts = max_contacts;
  return option;
}

} // namespace dart::benchmark::collision
