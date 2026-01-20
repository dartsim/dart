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

#include "dart/simulation/experimental/kinematics/joint_transform.hpp"

#include <dart/simulation/experimental/common/exceptions.hpp>

#include <cmath>

namespace dart::simulation::experimental::kinematics {

Eigen::Isometry3d computeFixedTransform()
{
  return Eigen::Isometry3d::Identity();
}

Eigen::Isometry3d computeRevoluteTransform(
    const Eigen::Vector3d& axis, double angle)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  return T;
}

Eigen::Isometry3d computePrismaticTransform(
    const Eigen::Vector3d& axis, double displacement)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = axis.normalized() * displacement;
  return T;
}

Eigen::Isometry3d computeScrewTransform(
    const Eigen::Vector3d& axis, double pitch, double angle)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d normalizedAxis = axis.normalized();
  T.linear() = Eigen::AngleAxisd(angle, normalizedAxis).toRotationMatrix();
  T.translation() = normalizedAxis * (pitch * angle);
  return T;
}

Eigen::Isometry3d computeUniversalTransform(
    const Eigen::Vector3d& axis1,
    const Eigen::Vector3d& axis2,
    double angle1,
    double angle2)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // First rotation around axis1
  const Eigen::Matrix3d R1
      = Eigen::AngleAxisd(angle1, axis1.normalized()).toRotationMatrix();

  // Second rotation around axis2 (rotated by R1 to maintain perpendicularity)
  const Eigen::Vector3d rotatedAxis2 = R1 * axis2.normalized();
  const Eigen::Matrix3d R2
      = Eigen::AngleAxisd(angle2, rotatedAxis2).toRotationMatrix();

  T.linear() = R2 * R1;
  return T;
}

Eigen::Isometry3d computeBallTransform(const Eigen::Vector3d& eulerAngles)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // ZYX Euler angles: first Z (yaw), then Y (pitch), then X (roll)
  // eulerAngles = [roll, pitch, yaw] for compatibility with common conventions
  T.linear() = (Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();

  return T;
}

Eigen::Isometry3d computeBallTransformQuat(const Eigen::Quaterniond& quaternion)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = quaternion.normalized().toRotationMatrix();
  return T;
}

Eigen::Isometry3d computePlanarTransform(
    const Eigen::Vector3d& planeNormal,
    const Eigen::Vector3d& inPlaneAxis,
    double x,
    double y,
    double theta)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  // Normalize axes
  const Eigen::Vector3d normal = planeNormal.normalized();
  const Eigen::Vector3d axisX = inPlaneAxis.normalized();
  const Eigen::Vector3d axisY = normal.cross(axisX).normalized();

  // Translation in the plane
  T.translation() = x * axisX + y * axisY;

  // Rotation around the plane normal
  T.linear() = Eigen::AngleAxisd(theta, normal).toRotationMatrix();

  return T;
}

Eigen::Isometry3d computeFreeTransform(const Eigen::Vector<double, 6>& position)
{
  return computeFreeTransform(
      position.head<3>(), // translation
      position.tail<3>()  // euler angles
  );
}

Eigen::Isometry3d computeFreeTransform(
    const Eigen::Vector3d& translation, const Eigen::Vector3d& eulerAngles)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  T.translation() = translation;

  // ZYX Euler angles
  T.linear() = (Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();

  return T;
}

Eigen::Isometry3d computeJointTransform(const comps::Joint& joint)
{
  using comps::JointType;

  switch (joint.type) {
    case JointType::Fixed:
      return computeFixedTransform();

    case JointType::Revolute:
      if (joint.position.size() < 1) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException, "Revolute joint requires 1 DOF position");
      }
      return computeRevoluteTransform(joint.axis, joint.position[0]);

    case JointType::Prismatic:
      if (joint.position.size() < 1) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException,
            "Prismatic joint requires 1 DOF position");
      }
      return computePrismaticTransform(joint.axis, joint.position[0]);

    case JointType::Screw:
      if (joint.position.size() < 1) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException, "Screw joint requires 1 DOF position");
      }
      return computeScrewTransform(joint.axis, joint.pitch, joint.position[0]);

    case JointType::Universal:
      if (joint.position.size() < 2) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException,
            "Universal joint requires 2 DOF position");
      }
      return computeUniversalTransform(
          joint.axis, joint.axis2, joint.position[0], joint.position[1]);

    case JointType::Ball:
      if (joint.position.size() < 3) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException, "Ball joint requires 3 DOF position");
      }
      return computeBallTransform(joint.position.head<3>());

    case JointType::Planar:
      if (joint.position.size() < 3) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException, "Planar joint requires 3 DOF position");
      }
      return computePlanarTransform(
          joint.axis,
          joint.axis2,
          joint.position[0],
          joint.position[1],
          joint.position[2]);

    case JointType::Free:
      if (joint.position.size() < 6) {
        DART_EXPERIMENTAL_THROW_T(
            InvalidArgumentException, "Free joint requires 6 DOF position");
      }
      return computeFreeTransform(
          Eigen::Vector<double, 6>(joint.position.head<6>()));

    case JointType::Custom:
      // Custom joints require user-defined transform computation
      // For now, return identity (user should handle this via ECS systems)
      return Eigen::Isometry3d::Identity();

    default:
      DART_EXPERIMENTAL_THROW_T(
          InvalidArgumentException,
          "Unknown joint type: {}",
          static_cast<int>(joint.type));
  }
}

} // namespace dart::simulation::experimental::kinematics
