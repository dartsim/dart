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

#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dart::simulation::experimental::kinematics {

/// @defgroup JointTransforms Joint Transform Functions
/// @brief Functions for computing joint-space to Cartesian-space transforms
///
/// These functions compute the relative transformation between parent and
/// child links based on joint type, parameters, and current position.
///
/// All transforms follow the convention:
/// - T_child_in_parent = computeXXXTransform(...)
/// - Child link pose = Parent link pose * T_child_in_parent
///
/// @{

/// Compute transform for a Fixed joint (identity)
///
/// Fixed joints rigidly connect two links with no relative motion.
///
/// @return Identity transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeFixedTransform();

/// Compute transform for a Revolute joint
///
/// Revolute joints allow rotation around a single axis.
///
/// @param axis Unit vector representing the rotation axis (in parent frame)
/// @param angle Rotation angle in radians
/// @return Rotation transform around the axis
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeRevoluteTransform(
    const Eigen::Vector3d& axis, double angle);

/// Compute transform for a Prismatic joint
///
/// Prismatic joints allow translation along a single axis.
///
/// @param axis Unit vector representing the translation axis (in parent frame)
/// @param displacement Translation distance
/// @return Translation transform along the axis
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computePrismaticTransform(
    const Eigen::Vector3d& axis, double displacement);

/// Compute transform for a Screw joint
///
/// Screw joints couple rotation and translation (helical motion).
/// For every radian of rotation, the joint translates by `pitch` units.
///
/// @param axis Unit vector representing the screw axis (in parent frame)
/// @param pitch Translation per radian of rotation (meters/radian)
/// @param angle Rotation angle in radians
/// @return Combined rotation + translation transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeScrewTransform(
    const Eigen::Vector3d& axis, double pitch, double angle);

/// Compute transform for a Universal joint
///
/// Universal joints allow rotation around two perpendicular axes.
/// The first rotation is around axis1, the second around axis2.
///
/// @param axis1 First rotation axis (in parent frame)
/// @param axis2 Second rotation axis (perpendicular to axis1, in parent frame)
/// @param angle1 First rotation angle in radians
/// @param angle2 Second rotation angle in radians
/// @return Combined rotation transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeUniversalTransform(
    const Eigen::Vector3d& axis1,
    const Eigen::Vector3d& axis2,
    double angle1,
    double angle2);

/// Compute transform for a Ball joint using Euler angles
///
/// Ball joints allow free rotation (3-DOF). This overload uses ZYX Euler
/// angles.
///
/// @param eulerAngles ZYX Euler angles (yaw, pitch, roll) in radians
/// @return Pure rotation transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeBallTransform(
    const Eigen::Vector3d& eulerAngles);

/// Compute transform for a Ball joint using quaternion
///
/// Ball joints allow free rotation (3-DOF). This overload uses a quaternion
/// for singularity-free representation.
///
/// @param quaternion Unit quaternion representing orientation
/// @return Pure rotation transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeBallTransformQuat(
    const Eigen::Quaterniond& quaternion);

/// Compute transform for a Planar joint
///
/// Planar joints constrain motion to a plane (2 translations + 1 rotation).
/// The plane is defined by its normal vector.
///
/// @param planeNormal Normal vector of the constraint plane (in parent frame)
/// @param inPlaneAxis Direction of one in-plane axis (perpendicular to normal)
/// @param x Translation along inPlaneAxis
/// @param y Translation along (normal x inPlaneAxis)
/// @param theta Rotation around the normal
/// @return Combined in-plane transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computePlanarTransform(
    const Eigen::Vector3d& planeNormal,
    const Eigen::Vector3d& inPlaneAxis,
    double x,
    double y,
    double theta);

/// Compute transform for a Free joint
///
/// Free joints allow unconstrained 6-DOF motion.
/// Position representation: [x, y, z, rx, ry, rz]
/// where rotation is ZYX Euler angles.
///
/// @param position 6-DOF position vector [translation(3), rotation(3)]
/// @return Full 6-DOF transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeFreeTransform(
    const Eigen::Vector<double, 6>& position);

/// Compute transform for a Free joint using separate components
///
/// @param translation 3D translation vector
/// @param eulerAngles ZYX Euler angles (yaw, pitch, roll) in radians
/// @return Full 6-DOF transform
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeFreeTransform(
    const Eigen::Vector3d& translation, const Eigen::Vector3d& eulerAngles);

/// @}

/// Generic joint transform computation
///
/// Dispatches to the appropriate transform function based on joint type.
/// This is the main entry point for computing joint transforms.
///
/// @param joint Joint component containing type, parameters, and position
/// @return Transform from parent link to child link
[[nodiscard]] DART_EXPERIMENTAL_API Eigen::Isometry3d computeJointTransform(
    const comps::Joint& joint);

} // namespace dart::simulation::experimental::kinematics
