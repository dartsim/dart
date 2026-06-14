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

#include <dart/simulation/comps/component_category.hpp>

#include <Eigen/Dense>
#include <entt/entt.hpp>

#include <limits>
#include <string>

namespace dart::simulation::comps {

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

  /// Spherical joint (3-DOF) - free rotation in all directions
  /// (ball-and-socket) Use: Human shoulders, camera gimbals, floating objects
  /// Representation: Quaternion or Euler angles
  /// Status: TODO
  Spherical,

  /// Planar joint (3-DOF) - Motion constrained to a plane
  /// DOF: 2 translations + 1 rotation within the plane
  /// Use: Flat surfaces, mobile robots on ground
  /// Status: TODO
  Planar,

  // ========== 6-DOF Joints ==========

  /// Floating joint (6-DOF) - Unconstrained motion
  /// DOF: 3 translations + 3 rotations
  /// Use: Floating base (humanoids, flying robots, space robots)
  /// Representation: Position (3D) + Orientation (quaternion)
  /// Status: TODO
  Floating,

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
  ///    dart::simulation::detail::registryOf(world).emplace<NeuralJointComponent>(jointEntity);
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

/// Joint actuator type (how the joint is driven during forward dynamics)
///
/// **Internal Implementation Detail** - Not exposed in public API
enum class ActuatorType
{
  Force,        ///< Commanded joint effort drives the dynamics (default)
  Passive,      ///< No commanded effort; only passive forces act
  Servo,        ///< Velocity servo to a target (not yet implemented)
  Velocity,     ///< Prescribed velocity (not yet implemented)
  Acceleration, ///< Prescribed acceleration (not yet implemented)
  Locked,       ///< Frozen at the current position (not yet implemented)
  Mimic         ///< Follows another joint (not yet implemented)
};

/// Joint limits
///
/// Per-coordinate lower/upper bounds for position, velocity, and actuation
/// effort. Unbounded coordinates use +/- infinity.
///
/// **Internal Implementation Detail** - Not exposed in public API
static constexpr Eigen::Index kMaxJointDof = 6;
using JointVector = Eigen::
    Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, kMaxJointDof, 1>;

inline JointVector makeJointVector(Eigen::Index size, double value)
{
  JointVector result(size);
  result.setConstant(value);
  return result;
}

struct JointLimits
{
  JointVector lower;         ///< Position lower bounds
  JointVector upper;         ///< Position upper bounds
  JointVector velocityLower; ///< Velocity lower bounds
  JointVector velocityUpper; ///< Velocity upper bounds
  JointVector effortLower;   ///< Actuation effort lower bounds
  JointVector effortUpper;   ///< Actuation effort upper bounds
};

/// Joint component (single joint in articulated body)
///
/// Stores all joint properties in a unified structure. Different fields
/// are used depending on the joint type:
///
/// Field usage by joint type:
/// - Fixed:      rigidBodyFixedJointLocalAnchor* for public rigid-body joints
/// - Revolute:   axis
/// - Prismatic:  axis
/// - Screw:      axis, pitch
/// - Universal:  axis, axis2 (must be perpendicular)
/// - Spherical:       (no geometric parameters - uses quaternion position)
/// - Planar:     axis (normal to plane), axis2 (in-plane direction)
/// - Floating:       (no geometric parameters - 6-DOF position)
///
/// **Internal Implementation Detail** - Not exposed in public API
struct Joint
{
  DART_SIMULATION_STATE_COMPONENT(Joint);

  JointType type = JointType::Revolute;
  ActuatorType actuatorType = ActuatorType::Force;
  std::string name;

  JointVector position;
  JointVector velocity;
  JointVector acceleration;
  JointVector torque;

  /// Passive joint dynamics (per generalized coordinate).
  JointVector springStiffness;
  JointVector dampingCoefficient;
  JointVector restPosition;

  /// Rotor/reflected inertia added to the joint-space mass-matrix diagonal
  /// (per generalized coordinate).
  JointVector armature;

  /// Coulomb (dry) friction force/torque magnitude that opposes joint motion
  /// (per generalized coordinate).
  JointVector coulombFriction;

  /// Commanded target velocity used by the Velocity actuator type (per
  /// generalized coordinate).
  JointVector commandVelocity;

  /// Maximum AVBD constraint force before the joint is marked broken. A value
  /// of 0 disables automatic breakage.
  double breakForce = 0.0;

  /// Whether the joint has been broken by an AVBD break-force threshold.
  bool broken = false;

  JointLimits limits;

  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();

  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();

  double pitch = 0.0;

  entt::entity parentLink = entt::null;
  entt::entity childLink = entt::null;

  bool hasRigidBodyFixedJointAnchors = false;
  Eigen::Vector3d rigidBodyFixedJointLocalAnchorParent
      = Eigen::Vector3d::Zero();
  Eigen::Vector3d rigidBodyFixedJointLocalAnchorChild = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rigidBodyFixedJointTargetRelativeOrientation
      = Eigen::Quaterniond::Identity();

  static constexpr auto entityFields()
  {
    return std::make_tuple(&Joint::parentLink, &Joint::childLink);
  }

  [[nodiscard]] std::size_t getDOF() const
  {
    using enum JointType;

    switch (type) {
      case Fixed:
        return 0;
      case Revolute:
        return 1;
      case Prismatic:
        return 1;
      case Screw:
        return 1;
      case Universal:
        return 2;
      case Spherical:
        return 3;
      case Planar:
        return 3;
      case Floating:
        return 6;
      default:
        return 0;
    }
  }
};

/// Family-owned AVBD point-joint stiffness sidecar.
///
/// Holds the per-family AVBD point-joint stiffness inputs that used to live on
/// the family-neutral Joint component. The presence of this component on a
/// joint entity is itself the "stiffness has been materialized/set" flag: when
/// the component is ABSENT, reads fall back to these same defaults (legacy
/// binaries derive start/max defaults from the endpoint AVBD config on first
/// use). Setting any stiffness emplaces the component and populates all four
/// fields.
///
/// **Internal Implementation Detail** - Not exposed in public API
struct AvbdJointStiffness
{
  DART_SIMULATION_PROPERTY_COMPONENT(AvbdJointStiffness);

  /// AVBD point-joint row starting stiffness.
  double startStiffness = 1.0;

  /// AVBD linear point-joint material stiffness. Infinity keeps hard rows.
  double linearStiffness = std::numeric_limits<double>::infinity();

  /// AVBD angular point-joint material stiffness. Infinity keeps hard rows.
  double angularStiffness = std::numeric_limits<double>::infinity();

  /// AVBD point-joint maximum ramp stiffness.
  double maxStiffness = std::numeric_limits<double>::infinity();
};

} // namespace dart::simulation::comps
