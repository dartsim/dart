/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "frame_bind.hpp"

#include "dart7/common/logging.hpp"
#include "dart7/frame/fixed_frame.hpp"
#include "dart7/frame/frame.hpp"
#include "dart7/frame/free_frame.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

// NOTE: Future consideration for Python bindings:
// Consider adding Pythonic property access alongside C++-style methods:
//   - frame.translation / frame.position (property) vs get_world_translation()
//   - frame.rotation (property) vs get_world_rotation()
//   - frame.quaternion / frame.quat (property) vs get_world_quaternion()
//
// This would provide both C++-consistent names and idiomatic Python properties,
// similar to numpy arrays having both .shape (property) and methods.
// Reference: https://numpy.org/doc/stable/reference/arrays.ndarray.html

namespace nb = nanobind;

namespace dartpy7 {

void defFrame(nb::module_& m)
{
  DART7_DEBUG("Registering Frame classes");

  // Frame base class
  nb::class_<dart7::Frame>(m, "Frame", "Base class for coordinate frames")
      .def(
          "is_valid",
          &dart7::Frame::isValid,
          "Check if this frame handle is valid\n\n"
          "Returns:\n"
          "    True if frame is valid, False otherwise")
      .def(
          "is_world",
          &dart7::Frame::isWorld,
          "Check if this is the world frame\n\n"
          "Returns:\n"
          "    True if this is the world frame, False otherwise")
      .def(
          "is_same_instance_as",
          &dart7::Frame::isSameInstanceAs,
          nb::arg("other"),
          "Check if two frames refer to the same entity\n\n"
          "Args:\n"
          "    other: Another Frame to compare with\n\n"
          "Returns:\n"
          "    True if both frames refer to the same entity")
      .def_static(
          "world",
          &dart7::Frame::world,
          "Get the world frame reference\n\n"
          "Returns:\n"
          "    World frame handle")
      .def(
          "__eq__",
          &dart7::Frame::operator==,
          nb::arg("other"),
          "Check equality")
      .def(
          "__ne__",
          &dart7::Frame::operator!=,
          nb::arg("other"),
          "Check inequality")
      .def("__repr__", [](const dart7::Frame& self) {
        if (!self.isValid()) {
          return std::string("<dart7.Frame: invalid>");
        }
        if (self.isWorld()) {
          return std::string("<dart7.Frame: world>");
        }
        return std::string("<dart7.Frame>");
      });

  // FreeFrame class
  nb::class_<dart7::FreeFrame, dart7::Frame>(
      m, "FreeFrame", "Independently positioned frame with 6-DOF")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "set_relative_transform",
      //     &dart7::FreeFrame::setRelativeTransform,
      //     nb::arg("transform"),
      //     "Set the relative transform (Isometry3d)\n\n"
      //     "Args:\n"
      //     "    transform: 4x4 transformation matrix as Eigen::Isometry3d")
      // .def(
      //     "set_relative_translation",
      //     &dart7::FreeFrame::setRelativeTranslation,
      //     nb::arg("translation"),
      //     "Set the relative translation (position)\n\n"
      //     "Args:\n"
      //     "    translation: 3D translation vector")
      // .def(
      //     "set_relative_rotation",
      //     nb::overload_cast<const Eigen::Matrix3d&>(
      //         &dart7::FreeFrame::setRelativeRotation),
      //     nb::arg("rotation"),
      //     "Set the relative rotation (3x3 matrix)\n\n"
      //     "Args:\n"
      //     "    rotation: 3x3 rotation matrix")
      // .def(
      //     "set_relative_rotation",
      //     [](dart7::FreeFrame& self, const Eigen::Vector4d& quat_wxyz) {
      //       // Convert [w, x, y, z] to Eigen::Quaterniond(w, x, y, z)
      //       Eigen::Quaterniond q(
      //           quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
      //       self.setRelativeRotation(q);
      //     },
      //     nb::arg("quaternion"),
      //     "Set the relative rotation (quaternion)\n\n"
      //     "Args:\n"
      //     "    quaternion: Rotation quaternion as [w, x, y, z] array")
      // .def(
      //     "set_transform",
      //     [](dart7::FreeFrame& self,
      //        const Eigen::Vector3d& position,
      //        const Eigen::Vector4d& quat_wxyz) {
      //       // Convert [w, x, y, z] to Eigen::Quaterniond(w, x, y, z)
      //       Eigen::Quaterniond q(
      //           quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]);
      //       self.setTransform(position, q);
      //     },
      //     nb::arg("position"),
      //     nb::arg("quaternion"),
      //     "Set transform using position and quaternion\n\n"
      //     "Args:\n"
      //     "    position: 3D translation vector\n"
      //     "    quaternion: Rotation quaternion as [w, x, y, z] array")
      // .def(
      //     "set_transform_matrix",
      //     &dart7::FreeFrame::setTransformMatrix,
      //     nb::arg("matrix"),
      //     "Set transform from 4x4 homogeneous matrix\n\n"
      //     "Args:\n"
      //     "    matrix: 4x4 homogeneous transformation matrix")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "get_relative_transform",
      //     [](const dart7::FreeFrame& self) -> Eigen::Matrix4d {
      //       return self.getRelativeTransform().matrix();
      //     },
      //     "Get the relative transform\n\n"
      //     "Returns:\n"
      //     "    4x4 transformation matrix")
      // .def(
      //     "get_relative_translation",
      //     &dart7::FreeFrame::getRelativeTranslation,
      //     "Get the relative translation (position)\n\n"
      //     "Returns:\n"
      //     "    3D translation vector")
      // .def(
      //     "get_relative_rotation",
      //     &dart7::FreeFrame::getRelativeRotation,
      //     "Get the relative rotation (3x3 matrix)\n\n"
      //     "Returns:\n"
      //     "    3x3 rotation matrix")
      // .def(
      //     "get_relative_quaternion",
      //     [](const dart7::FreeFrame& self) -> Eigen::Vector4d {
      //       auto q = self.getRelativeQuaternion();
      //       // Return as [w, x, y, z]
      //       return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
      //     },
      //     "Get the relative rotation (quaternion)\n\n"
      //     "Returns:\n"
      //     "    Rotation quaternion as [w, x, y, z] array")
      // .def(
      //     "get_transform_matrix",
      //     &dart7::FreeFrame::getTransformMatrix,
      //     "Get the transform as 4x4 homogeneous matrix\n\n"
      //     "Returns:\n"
      //     "    4x4 transformation matrix")
      // .def(
      //     "set_relative_spatial_velocity",
      //     &dart7::FreeFrame::setRelativeSpatialVelocity,
      //     nb::arg("spatial_velocity"),
      //     "Set the spatial velocity (6D vector [angular; linear])\n\n"
      //     "Args:\n"
      //     "    spatial_velocity: 6D vector [omega; v]")
      // .def(
      //     "set_relative_velocity",
      //     &dart7::FreeFrame::setRelativeVelocity,
      //     nb::arg("linear"),
      //     nb::arg("angular"),
      //     "Set linear and angular velocity separately\n\n"
      //     "Args:\n"
      //     "    linear: Linear velocity vector\n"
      //     "    angular: Angular velocity vector")
      // .def(
      //     "set_relative_linear_velocity",
      //     &dart7::FreeFrame::setRelativeLinearVelocity,
      //     nb::arg("linear"),
      //     "Set only the linear velocity\n\n"
      //     "Args:\n"
      //     "    linear: Linear velocity vector")
      // .def(
      //     "set_relative_angular_velocity",
      //     &dart7::FreeFrame::setRelativeAngularVelocity,
      //     nb::arg("angular"),
      //     "Set only the angular velocity\n\n"
      //     "Args:\n"
      //     "    angular: Angular velocity vector")
      // .def(
      //     "get_relative_spatial_velocity",
      //     &dart7::FreeFrame::getRelativeSpatialVelocity,
      //     "Get the spatial velocity (6D vector)\n\n"
      //     "Returns:\n"
      //     "    6D vector [angular; linear]")
      // .def(
      //     "get_relative_linear_velocity",
      //     &dart7::FreeFrame::getRelativeLinearVelocity,
      //     "Get the linear velocity component\n\n"
      //     "Returns:\n"
      //     "    3D linear velocity vector")
      // .def(
      //     "get_relative_angular_velocity",
      //     &dart7::FreeFrame::getRelativeAngularVelocity,
      //     "Get the angular velocity component\n\n"
      //     "Returns:\n"
      //     "    3D angular velocity vector")
      // .def(
      //     "set_relative_spatial_acceleration",
      //     &dart7::FreeFrame::setRelativeSpatialAcceleration,
      //     nb::arg("spatial_acceleration"),
      //     "Set the spatial acceleration (6D vector [angular; linear])\n\n"
      //     "Args:\n"
      //     "    spatial_acceleration: 6D vector [alpha; a]")
      // .def(
      //     "set_relative_acceleration",
      //     &dart7::FreeFrame::setRelativeAcceleration,
      //     nb::arg("linear"),
      //     nb::arg("angular"),
      //     "Set linear and angular acceleration separately\n\n"
      //     "Args:\n"
      //     "    linear: Linear acceleration vector\n"
      //     "    angular: Angular acceleration vector")
      // .def(
      //     "set_relative_linear_acceleration",
      //     &dart7::FreeFrame::setRelativeLinearAcceleration,
      //     nb::arg("linear"),
      //     "Set only the linear acceleration\n\n"
      //     "Args:\n"
      //     "    linear: Linear acceleration vector")
      // .def(
      //     "set_relative_angular_acceleration",
      //     &dart7::FreeFrame::setRelativeAngularAcceleration,
      //     nb::arg("angular"),
      //     "Set only the angular acceleration\n\n"
      //     "Args:\n"
      //     "    angular: Angular acceleration vector")
      // .def(
      //     "get_relative_spatial_acceleration",
      //     &dart7::FreeFrame::getRelativeSpatialAcceleration,
      //     "Get the spatial acceleration (6D vector)\n\n"
      //     "Returns:\n"
      //     "    6D vector [angular; linear]")
      // .def(
      //     "get_relative_linear_acceleration",
      //     &dart7::FreeFrame::getRelativeLinearAcceleration,
      //     "Get the linear acceleration component\n\n"
      //     "Returns:\n"
      //     "    3D linear acceleration vector")
      // .def(
      //     "get_relative_angular_acceleration",
      //     &dart7::FreeFrame::getRelativeAngularAcceleration,
      //     "Get the angular acceleration component\n\n"
      //     "Returns:\n"
      //     "    3D angular acceleration vector")
      // .def(
      //     "get_parent_frame",
      //     &dart7::FreeFrame::getParentFrame,
      //     "Get the parent frame\n\n"
      //     "Returns:\n"
      //     "    Parent frame handle")
      // .def(
      //     "set_parent_frame",
      //     &dart7::FreeFrame::setParentFrame,
      //     nb::arg("parent"),
      //     "Set the parent frame\n\n"
      //     "Args:\n"
      //     "    parent: New parent frame")
      // // World-frame kinematics
      // .def(
      //     "get_world_translation",
      //     &dart7::FreeFrame::getWorldTranslation,
      //     "Get the world-frame translation\n\n"
      //     "Returns:\n"
      //     "    3D world-frame position vector")
      // .def(
      //     "get_world_rotation",
      //     &dart7::FreeFrame::getWorldRotation,
      //     "Get the world-frame rotation (3x3 matrix)\n\n"
      //     "Returns:\n"
      //     "    3x3 world-frame rotation matrix")
      // .def(
      //     "get_world_quaternion",
      //     [](const dart7::FreeFrame& self) -> Eigen::Vector4d {
      //       auto q = self.getWorldQuaternion();
      //       // Return as [x, y, z, w]
      //       return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
      //     },
      //     "Get the world-frame rotation (quaternion)\n\n"
      //     "Returns:\n"
      //     "    World-frame rotation quaternion as [x, y, z, w] array")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "get_world_transform_matrix",
      //     &dart7::FreeFrame::getWorldTransformMatrix,
      //     "Get the world-frame transform as 4x4 matrix\n\n"
      //     "Returns:\n"
      //     "    4x4 world-frame transformation matrix")
      // .def(
      //     "get_world_spatial_velocity",
      //     &dart7::FreeFrame::getWorldSpatialVelocity,
      //     "Get the spatial velocity in world frame\n\n"
      //     "Returns:\n"
      //     "    6D vector [angular; linear] in world frame")
      // .def(
      //     "get_world_linear_velocity",
      //     &dart7::FreeFrame::getWorldLinearVelocity,
      //     "Get the linear velocity in world frame\n\n"
      //     "Returns:\n"
      //     "    3D world-frame linear velocity")
      // .def(
      //     "get_world_angular_velocity",
      //     &dart7::FreeFrame::getWorldAngularVelocity,
      //     "Get the angular velocity in world frame\n\n"
      //     "Returns:\n"
      //     "    3D world-frame angular velocity")
      // .def(
      //     "get_world_spatial_acceleration",
      //     &dart7::FreeFrame::getWorldSpatialAcceleration,
      //     "Get the spatial acceleration in world frame\n\n"
      //     "Returns:\n"
      //     "    6D vector [angular; linear] in world frame")
      // .def(
      //     "get_world_linear_acceleration",
      //     &dart7::FreeFrame::getWorldLinearAcceleration,
      //     "Get the linear acceleration in world frame\n\n"
      //     "Returns:\n"
      //     "    3D world-frame linear acceleration")
      // .def(
      //     "get_world_angular_acceleration",
      //     &dart7::FreeFrame::getWorldAngularAcceleration,
      //     "Get the angular acceleration in world frame\n\n"
      //     "Returns:\n"
      //     "    3D world-frame angular acceleration")
      // // Convenience aliases for most common operations (world-frame)
      // .def(
      //     "get_position",
      //     &dart7::FreeFrame::getWorldTranslation,
      //     "Get world-frame position (convenience alias)\n\n"
      //     "Returns:\n"
      //     "    3D world-frame position vector")
      // .def(
      //     "get_quaternion",
      //     [](const dart7::FreeFrame& self) -> Eigen::Vector4d {
      //       auto q = self.getWorldQuaternion();
      //       return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
      //     },
      //     "Get world-frame orientation as quaternion (convenience alias)\n\n"
      //     "Returns:\n"
      //     "    World-frame rotation quaternion as [x, y, z, w] array")
      // .def(
      //     "get_rotation_matrix",
      //     &dart7::FreeFrame::getWorldRotation,
      //     "Get world-frame rotation matrix (convenience alias)\n\n"
      //     "Returns:\n"
      //     "    3x3 world-frame rotation matrix")
      // .def(
      //     "get_transform",
      //     &dart7::FreeFrame::getWorldTransformMatrix,
      //     "Get world-frame transform (convenience alias)\n\n"
      //     "Note: 4x4 matrix is the source of truth. Position/quaternion
      //     are\n" "      extracted from this matrix for convenience.\n\n"
      //     "Returns:\n"
      //     "    4x4 world-frame homogeneous transformation matrix")
      // .def(
      //     "set_position",
      //     &dart7::FreeFrame::setRelativeTranslation,
      //     nb::arg("position"),
      //     "Convenience alias for set_relative_translation()\n\n"
      //     "Args:\n"
      //     "    position: 3D position vector")
      // .def(
      //     "set_quaternion",
      //     [](dart7::FreeFrame& self, const Eigen::Vector4d& quat_xyzw) {
      //       // Accept [x, y, z, w] format (more common in Python)
      //       Eigen::Quaterniond q(
      //           quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
      //       self.setRelativeRotation(q);
      //     },
      //     nb::arg("quaternion"),
      //     "Convenience method to set rotation from quaternion\n\n"
      //     "Args:\n"
      //     "    quaternion: Rotation quaternion as [x, y, z, w] array")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "set_rotation_matrix",
      //     nb::overload_cast<const Eigen::Matrix3d&>(
      //         &dart7::FreeFrame::setRelativeRotation),
      //     nb::arg("rotation"),
      //     "Convenience alias for set_relative_rotation()\n\n"
      //     "Args:\n"
      //     "    rotation: 3x3 rotation matrix")
      .def("__repr__", [](const dart7::FreeFrame& self) {
        if (!self.isValid()) {
          return std::string("<dart7.FreeFrame: invalid>");
        }
        return std::string("<dart7.FreeFrame>");
      });

  // FixedFrame class
  nb::class_<dart7::FixedFrame, dart7::Frame>(
      m, "FixedFrame", "Rigidly attached frame with fixed offset from parent")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "set_relative_transform",
      //     &dart7::FixedFrame::setRelativeTransform,
      //     nb::arg("transform"),
      //     "Set the fixed relative transform offset\n\n"
      //     "Args:\n"
      //     "    transform: Fixed transformation offset")
      // .def(
      //     "set_transform",
      //     [](dart7::FixedFrame& self,
      //        const Eigen::Vector3d& position,
      //        const Eigen::Vector4d& quat_xyzw) {
      //       // Convert [x, y, z, w] to Eigen::Quaterniond(w, x, y, z)
      //       Eigen::Quaterniond q(
      //           quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]);
      //       self.setTransform(position, q);
      //     },
      //     nb::arg("position"),
      //     nb::arg("quaternion"),
      //     "Set transform using position and quaternion\n\n"
      //     "Args:\n"
      //     "    position: 3D translation vector\n"
      //     "    quaternion: Rotation quaternion as [x, y, z, w] array")
      // .def(
      //     "set_transform_matrix",
      //     &dart7::FixedFrame::setTransformMatrix,
      //     nb::arg("matrix"),
      //     "Set transform from 4x4 homogeneous matrix\n\n"
      //     "Args:\n"
      //     "    matrix: 4x4 homogeneous transformation matrix")
      // .def(
      //     "get_relative_transform",
      //     [](const dart7::FixedFrame& self) -> Eigen::Matrix4d {
      //       return self.getRelativeTransform().matrix();
      //     },
      //     "Get the fixed relative transform offset\n\n"
      //     "Returns:\n"
      //     "    4x4 transformation matrix")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "get_relative_translation",
      //     &dart7::FixedFrame::getRelativeTranslation,
      //     "Get the relative translation (position)\n\n"
      //     "Returns:\n"
      //     "    3D translation vector")
      // .def(
      //     "get_relative_rotation",
      //     &dart7::FixedFrame::getRelativeRotation,
      //     "Get the relative rotation (3x3 matrix)\n\n"
      //     "Returns:\n"
      //     "    3x3 rotation matrix")
      // .def(
      //     "get_relative_quaternion",
      //     [](const dart7::FixedFrame& self) -> Eigen::Vector4d {
      //       auto q = self.getRelativeQuaternion();
      //       // Return as [x, y, z, w]
      //       return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
      //     },
      //     "Get the relative rotation (quaternion)\n\n"
      //     "Returns:\n"
      //     "    Rotation quaternion as [x, y, z, w] array")
      // .def(
      //     "get_transform_matrix",
      //     &dart7::FixedFrame::getTransformMatrix,
      //     "Get the transform as 4x4 homogeneous matrix\n\n"
      //     "Returns:\n"
      //     "    4x4 transformation matrix")
      // .def(
      //     "get_relative_spatial_velocity",
      //     &dart7::FixedFrame::getRelativeSpatialVelocity,
      //     "Get the spatial velocity (always zero for attached frames)\n\n"
      //     "Returns:\n"
      //     "    Zero 6D vector")
      // .def(
      //     "get_relative_linear_velocity",
      //     &dart7::FixedFrame::getRelativeLinearVelocity,
      //     "Get the linear velocity (always zero)\n\n"
      //     "Returns:\n"
      //     "    Zero vector")
      // .def(
      //     "get_relative_angular_velocity",
      //     &dart7::FixedFrame::getRelativeAngularVelocity,
      //     "Get the angular velocity (always zero)\n\n"
      //     "Returns:\n"
      //     "    Zero vector")
      // .def(
      //     "get_relative_spatial_acceleration",
      //     &dart7::FixedFrame::getRelativeSpatialAcceleration,
      //     "Get the spatial acceleration (always zero for attached
      //     frames)\n\n" "Returns:\n" "    Zero 6D vector")
      // .def(
      //     "get_relative_linear_acceleration",
      //     &dart7::FixedFrame::getRelativeLinearAcceleration,
      //     "Get the linear acceleration (always zero)\n\n"
      //     "Returns:\n"
      //     "    Zero vector")
      // .def(
      //     "get_relative_angular_acceleration",
      //     &dart7::FixedFrame::getRelativeAngularAcceleration,
      //     "Get the angular acceleration (always zero)\n\n"
      //     "Returns:\n"
      //     "    Zero vector")
      // .def(
      //     "get_parent_frame",
      //     &dart7::FixedFrame::getParentFrame,
      //     "Get the parent frame\n\n"
      //     "Returns:\n"
      //     "    Parent frame handle")
      // // World-frame kinematics
      // .def(
      //     "get_world_translation",
      //     &dart7::FixedFrame::getWorldTranslation,
      //     "Get the world-frame translation\n\n"
      //     "Returns:\n"
      //     "    3D world-frame position vector")
      // .def(
      //     "get_world_rotation",
      //     &dart7::FixedFrame::getWorldRotation,
      //     "Get the world-frame rotation (3x3 matrix)\n\n"
      //     "Returns:\n"
      //     "    3x3 world-frame rotation matrix")
      // .def(
      //     "get_world_quaternion",
      //     [](const dart7::FixedFrame& self) -> Eigen::Vector4d {
      //       auto q = self.getWorldQuaternion();
      //       // Return as [x, y, z, w]
      //       return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
      //     },
      //     "Get the world-frame rotation (quaternion)\n\n"
      //     "Returns:\n"
      //     "    World-frame rotation quaternion as [x, y, z, w] array")
      // TODO: Uncomment when C++ methods are implemented
      // .def(
      //     "get_world_transform_matrix",
      //     &dart7::FixedFrame::getWorldTransformMatrix,
      //     "Get the world-frame transform as 4x4 matrix\n\n"
      //     "Returns:\n"
      //     "    4x4 world-frame transformation matrix")
      // .def(
      //     "get_world_spatial_velocity",
      //     &dart7::FixedFrame::getWorldSpatialVelocity,
      //     "Get the spatial velocity in world frame (inherited from
      //     parent)\n\n" "Returns:\n" "    6D vector [angular; linear] in world
      //     frame")
      // .def(
      //     "get_world_linear_velocity",
      //     &dart7::FixedFrame::getWorldLinearVelocity,
      //     "Get the linear velocity in world frame (inherited from
      //     parent)\n\n" "Returns:\n" "    3D world-frame linear velocity")
      // .def(
      //     "get_world_angular_velocity",
      //     &dart7::FixedFrame::getWorldAngularVelocity,
      //     "Get the angular velocity in world frame (inherited from
      //     parent)\n\n" "Returns:\n" "    3D world-frame angular velocity")
      // .def(
      //     "get_world_spatial_acceleration",
      //     &dart7::FixedFrame::getWorldSpatialAcceleration,
      //     "Get the spatial acceleration in world frame (inherited from "
      //     "parent)\n\n"
      //     "Returns:\n"
      //     "    6D vector [angular; linear] in world frame")
      // .def(
      //     "get_world_linear_acceleration",
      //     &dart7::FixedFrame::getWorldLinearAcceleration,
      //     "Get the linear acceleration in world frame (inherited from "
      //     "parent)\n\n"
      //     "Returns:\n"
      //     "    3D world-frame linear acceleration")
      // .def(
      //     "get_world_angular_acceleration",
      //     &dart7::FixedFrame::getWorldAngularAcceleration,
      //     "Get the angular acceleration in world frame (inherited from "
      //     "parent)\n\n"
      //     "Returns:\n"
      //     "    3D world-frame angular acceleration")
      .def("__repr__", [](const dart7::FixedFrame& self) {
        if (!self.isValid()) {
          return std::string("<dart7.FixedFrame: invalid>");
        }
        return std::string("<dart7.FixedFrame>");
      });
}

} // namespace dartpy7
