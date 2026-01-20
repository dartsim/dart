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

#include <dart/simulation/experimental/dynamics/motion_subspace.hpp>

namespace dart::simulation::experimental::dynamics {

MotionSubspace computeFixedMotionSubspace()
{
  return MotionSubspace(6, 0);
}

MotionSubspace1 computeRevoluteMotionSubspace(const Eigen::Vector3d& axis)
{
  MotionSubspace1 S;
  S.head<3>() = axis;
  S.tail<3>().setZero();
  return S;
}

MotionSubspace1 computePrismaticMotionSubspace(const Eigen::Vector3d& axis)
{
  MotionSubspace1 S;
  S.head<3>().setZero();
  S.tail<3>() = axis;
  return S;
}

MotionSubspace1 computeScrewMotionSubspace(
    const Eigen::Vector3d& axis, double pitch)
{
  MotionSubspace1 S;
  S.head<3>() = axis;
  S.tail<3>() = pitch * axis;
  return S;
}

MotionSubspace2 computeUniversalMotionSubspace(
    const Eigen::Vector3d& axis1,
    const Eigen::Vector3d& axis2InParentFrame,
    double angle1ForAxis2Rotation)
{
  MotionSubspace2 S;

  S.col(0).head<3>() = axis1;
  S.col(0).tail<3>().setZero();

  const Eigen::AngleAxisd rotation(angle1ForAxis2Rotation, axis1);
  const Eigen::Vector3d axis2Rotated = rotation * axis2InParentFrame;
  S.col(1).head<3>() = axis2Rotated;
  S.col(1).tail<3>().setZero();

  return S;
}

MotionSubspace3 computeBallMotionSubspace()
{
  MotionSubspace3 S = MotionSubspace3::Zero();
  S.topRows<3>() = Eigen::Matrix3d::Identity();
  return S;
}

MotionSubspace3 computePlanarMotionSubspace(
    const Eigen::Vector3d& planeNormal, const Eigen::Vector3d& inPlaneAxis)
{
  MotionSubspace3 S = MotionSubspace3::Zero();

  const Eigen::Vector3d secondInPlaneAxis = planeNormal.cross(inPlaneAxis);

  S.col(0).head<3>() = planeNormal;
  S.col(1).tail<3>() = inPlaneAxis;
  S.col(2).tail<3>() = secondInPlaneAxis;

  return S;
}

MotionSubspace6 computeFreeMotionSubspace()
{
  return MotionSubspace6::Identity();
}

MotionSubspace computeMotionSubspace(const comps::Joint& joint)
{
  switch (joint.type) {
    case comps::JointType::Fixed:
      return computeFixedMotionSubspace();
    case comps::JointType::Revolute:
      return computeRevoluteMotionSubspace(joint.axis);
    case comps::JointType::Prismatic:
      return computePrismaticMotionSubspace(joint.axis);
    case comps::JointType::Screw:
      return computeScrewMotionSubspace(joint.axis, joint.pitch);
    case comps::JointType::Universal:
      return computeUniversalMotionSubspace(
          joint.axis, joint.axis2, joint.position(0));
    case comps::JointType::Ball:
      return computeBallMotionSubspace();
    case comps::JointType::Planar:
      return computePlanarMotionSubspace(joint.axis, joint.axis2);
    case comps::JointType::Free:
      return computeFreeMotionSubspace();
    default:
      return computeFixedMotionSubspace();
  }
}

} // namespace dart::simulation::experimental::dynamics
