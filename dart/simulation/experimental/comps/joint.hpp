/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/simulation/experimental/comps/component_category.hpp>

#include <Eigen/Dense>
#include <entt/entt.hpp>

#include <string>

namespace dart::simulation::experimental::comps {

/// Joint type enumeration
///
/// Defines all standard joint types used in multibody dynamics and robotics.
/// Implementation status indicates which are currently supported.
///
/// Naming follows modern C++ convention: PascalCase for enum members.
/// (ALL_CAPS is reserved for macros only)
///
/// Design philosophy:
/// - All types defined upfront for extensibility
/// - ECS architecture makes adding new types straightforward
/// - No polymorphism - single JointComponent handles all types
/// - Type-specific behavior via switch/if on JointType enum
///
/// **Internal Implementation Detail** - Not exposed in public API
enum class JointType
{
  // ========== 0-DOF Joints ==========

  /// Fixed/Weld joint (0-DOF) - Rigidly connects two links
  /// Use: Connect links without relative motion (e.g., mounting plates)
  /// Status: TODO
  Fixed,

  // ========== 1-DOF Joints ==========

  /// Revolute/Hinge joint (1-DOF) - Rotation around a single axis
  /// Use: Most common robot joint (e.g., shoulder, elbow, knee)
  /// Status: IMPLEMENTED
  Revolute,

  /// Prismatic/Slider joint (1-DOF) - Translation along a single axis
  /// Use: Linear actuators, telescoping mechanisms
  /// Status: IMPLEMENTED
  Prismatic,

  /// Screw/Helical joint (1-DOF) - Coupled rotation + translation
  /// Use: Threaded mechanisms, lead screws
  /// Parameters: pitch (translation per radian of rotation)
  /// Status: TODO
  Screw,

  // ========== 2-DOF Joints ==========

  /// Universal joint (2-DOF) - Rotation around two perpendicular axes
  /// Use: Drive shafts, gimbal mechanisms
  /// Parameters: Two perpendicular axes
  /// Status: TODO
  Universal,

  // ========== 3-DOF Joints ==========

  /// Ball/Spherical joint (3-DOF) - Free rotation in all directions
  /// Use: Human shoulders, camera gimbals, floating objects
  /// Representation: Quaternion or Euler angles
  /// Status: TODO
  Ball,

  /// Planar joint (3-DOF) - Motion constrained to a plane
  /// DOF: 2 translations + 1 rotation within the plane
  /// Use: Flat surfaces, mobile robots on ground
  /// Status: TODO
  Planar,

  // ========== 6-DOF Joints ==========

  /// Free joint (6-DOF) - Unconstrained motion
  /// DOF: 3 translations + 3 rotations
  /// Use: Floating base (humanoids, flying robots, space robots)
  /// Representation: Position (3D) + Orientation (quaternion)
  /// Status: TODO
  Free,

  // ========== Extensibility ==========

  /// Custom joint type for user-defined joint behavior
  ///
  /// This enables extensibility without modifying the core library.
  /// Users can define custom joint types (e.g., neural network-based,
  /// soft/compliant, or application-specific) by:
  ///
  /// 1. Setting joint type to Custom
  /// 2. Attaching user-defined components to the joint entity:
  ///    @code
  ///    struct NeuralJointComponent {
  ///      std::vector<Eigen::MatrixXd> weights;
  ///      std::function<Eigen::Isometry3d(const Eigen::VectorXd&)>
  ///      forwardKinematics; std::function<Eigen::MatrixXd(const
  ///      Eigen::VectorXd&)> jacobian;
  ///    };
  ///    world.getRegistry().emplace<NeuralJointComponent>(jointEntity);
  ///    @endcode
  ///
  /// 3. Implementing custom ECS systems to handle the custom joint behavior
  ///
  /// This leverages ECS architecture for true extensibility without
  /// inheritance or core library modifications.
  ///
  /// Status: TODO
  Custom
};

/// Joint limits
///
/// **Internal Implementation Detail** - Not exposed in public API
struct JointLimits
{
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;
};

/// Joint component (single joint in articulated body)
///
/// Stores all joint properties in a unified structure. Different fields
/// are used depending on the joint type:
///
/// Field usage by joint type:
/// - Fixed:      (no geometric parameters)
/// - Revolute:   axis
/// - Prismatic:  axis
/// - Screw:      axis, pitch
/// - Universal:  axis, axis2 (must be perpendicular)
/// - Ball:       (no geometric parameters - uses quaternion position)
/// - Planar:     axis (normal to plane), axis2 (in-plane direction)
/// - Free:       (no geometric parameters - 6-DOF position)
///
/// **Internal Implementation Detail** - Not exposed in public API
struct Joint
{
  DART_EXPERIMENTAL_STATE_COMPONENT(Joint);

  JointType type = JointType::Revolute;
  std::string name;

  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd torque;

  JointLimits limits;

  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();

  double pitch = 0.0;

  entt::entity parentLink = entt::null;
  entt::entity childLink = entt::null;

  static constexpr auto entityFields()
  {
    return std::make_tuple(&Joint::parentLink, &Joint::childLink);
  }

  [[nodiscard]] std::size_t getDOF() const
  {
    switch (type) {
      case JointType::Fixed:
        return 0;
      case JointType::Revolute:
        return 1;
      case JointType::Prismatic:
        return 1;
      case JointType::Screw:
        return 1;
      case JointType::Universal:
        return 2;
      case JointType::Ball:
        return 3;
      case JointType::Planar:
        return 3;
      case JointType::Free:
        return 6;
      default:
        return 0;
    }
  }
};

} // namespace dart::simulation::experimental::comps
