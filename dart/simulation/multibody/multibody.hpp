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

#include <dart/simulation/entity.hpp>
#include <dart/simulation/fwd.hpp>
#include <dart/simulation/multibody/link.hpp> // Need complete type for LinkOptions

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <cstddef>

namespace dart::simulation {

/// Joint construction value object.
///
/// JointSpec describes the public joint configuration used when connecting a
/// child link to a parent link. It intentionally contains joint concepts only;
/// the parent link remains part of the link-construction call so the same
/// joint spec can be reused for multiple construction sites.
///
/// Usage:
/// @code
///   auto base = robot.addLink("base");
///   auto forearm = robot.addLink("forearm", base, JointSpec{
///       .name = "elbow",
///       .type = JointType::Revolute,
///       .axis = Eigen::Vector3d::UnitZ(),
///   });
/// @endcode
struct JointSpec
{
  std::string name;                     ///< Name of connecting joint
  JointType type = JointType::Revolute; ///< Type of joint
  Eigen::Vector3d axis
      = Eigen::Vector3d::UnitZ(); ///< Joint axis (rotation or translation)

  /// Secondary joint axis (Universal: second rotation axis; Planar: in-plane
  /// direction). Must not be parallel to `axis` for those joint types.
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();

  /// Transform from the parent link frame to the joint input frame, applied
  /// before the joint motion. Identity (the default) anchors the joint at the
  /// parent link's frame origin; a non-identity value offsets the joint from
  /// the parent origin (for example, a child joint partway along a parent link,
  /// or sibling joints at different locations on a branching parent).
  Eigen::Isometry3d transformToParent = Eigen::Isometry3d::Identity();

  /// Transform from the parent-joint frame (after joint motion) to the child
  /// link frame. This places the child link relative to its joint, giving the
  /// chain spatial extent (for example, the length of a pendulum link). The
  /// child link's center of mass is at the child link frame origin.
  Eigen::Isometry3d transformFromParent = Eigen::Isometry3d::Identity();

  // Future: Add more joint-specific parameters.
  // double pitch;           // For Screw
};

/// Options for creating a link with parent joint
///
/// Specifies how a link connects to its parent via a joint.
/// Used for child links (not root links).
///
/// Usage:
/// @code
///   auto root = robot.addLink("base");  // Root link (no options)
///
///   // Revolute joint
///   auto link1 = robot.addLink("link1", {
///       .parentLink = root,
///       .jointName = "shoulder",
///       .jointType = JointType::Revolute,
///       .axis = {0, 0, 1}
///   });
///
///   // Prismatic joint
///   auto link2 = robot.addLink("link2", {
///       .parentLink = link1,
///       .jointName = "slider",
///       .jointType = JointType::Prismatic,
///       .axis = {1, 0, 0}
///   });
/// @endcode
struct LinkOptions
{
  Link parentLink;                           ///< Parent link handle
  std::string jointName;                     ///< Name of connecting joint
  JointType jointType = JointType::Revolute; ///< Type of joint
  Eigen::Vector3d axis
      = Eigen::Vector3d::UnitZ(); ///< Joint axis (rotation or translation)

  /// Secondary joint axis (Universal: second rotation axis; Planar: in-plane
  /// direction). Must not be parallel to `axis` for those joint types.
  Eigen::Vector3d axis2 = Eigen::Vector3d::UnitX();

  /// Transform from the parent link frame to the joint input frame, applied
  /// before the joint motion. See JointSpec::transformToParent.
  Eigen::Isometry3d transformToParent = Eigen::Isometry3d::Identity();

  /// Transform from the parent-joint frame (after joint motion) to the child
  /// link frame. See JointSpec::transformFromParent.
  Eigen::Isometry3d transformFromParent = Eigen::Isometry3d::Identity();

  // Future: Add more joint-specific parameters
  // double pitch;           // For Screw
};

/// Multibody represents an articulated rigid body system
///
/// This is a lightweight handle class that references World-owned articulation
/// state. It provides convenient API to access and modify Multibody properties
/// without owning the data.
///
/// A Multibody consists of multiple rigid Links connected by Joints.
/// This class is suitable for representing:
/// - Robotic manipulators (e.g., UR5, Franka Emika Panda)
/// - Humanoid characters (e.g., bipeds, quadrupeds)
/// - Mechanical systems (e.g., furniture, mechanisms)
///
/// The term "Multibody" is used for consistency with other classes in
/// this library (e.g., RigidBody, SoftBody, FixedFrame, FreeFrame).
///
/// @note Handles are lightweight and safe to copy. Handles become invalid if
///       the referenced multibody or World is destroyed.
class DART_SIMULATION_API Multibody
{
public:
  /// Construct a Multibody handle
  ///
  /// @param entity The opaque identity token.
  /// @param world Pointer to the owning World.
  Multibody(Entity entity, World* world);

  /// Get the name of this Multibody
  ///
  /// @return The name
  std::string_view getName() const;

  /// Set the name of this Multibody
  ///
  /// @param name New name
  void setName(std::string_view name);

  /// Get the number of links in this Multibody
  ///
  /// @return Number of links
  std::size_t getLinkCount() const;

  /// Get the number of joints in this Multibody
  ///
  /// @return Number of joints
  std::size_t getJointCount() const;

  /// Get the total number of degrees of freedom
  ///
  /// @return Total DOFs across all joints
  std::size_t getDOFCount() const;

  /// Get a link by name
  ///
  /// @param name Link name
  /// @return Link handle if found, std::nullopt otherwise
  std::optional<Link> getLink(std::string_view name) const;

  /// Get a joint by name
  ///
  /// @param name Joint name
  /// @return Joint handle if found, std::nullopt otherwise
  std::optional<Joint> getJoint(std::string_view name) const;

  /// Get all link handles in construction order.
  ///
  /// The returned vector is a snapshot of lightweight handles. The handles
  /// remain tied to this Multibody's World and follow the same validity rules
  /// as handles returned by addLink() and getLink().
  [[nodiscard]] std::vector<Link> getLinks() const;

  /// Get all joint handles in construction order.
  ///
  /// The returned vector is a snapshot of lightweight handles. The handles
  /// remain tied to this Multibody's World and follow the same validity rules
  /// as handles returned by getJoint().
  [[nodiscard]] std::vector<Joint> getJoints() const;

  /// Get link names in construction order.
  ///
  /// Link names are unique within this Multibody.
  [[nodiscard]] std::vector<std::string> getLinkNames() const;

  /// Get joint names in construction order.
  ///
  /// Joint names are unique within this Multibody.
  [[nodiscard]] std::vector<std::string> getJointNames() const;

  /// Get the backend-neutral identity token for this multibody.
  [[nodiscard]] Entity getEntity() const;

  /// Get the owning World pointer.
  ///
  /// @return Pointer to the World
  [[nodiscard]] World* getWorld() const;

  /// Return whether this handle still refers to a live Multibody.
  ///
  /// Handles become invalid after the owning World destroys the underlying
  /// entity, including through World::clear().
  [[nodiscard]] bool isValid() const;

  /// Return whether this multibody is currently asleep under World
  /// deactivation.
  ///
  /// Returns false when deactivation is disabled, unsupported for the active
  /// world configuration, or the multibody has not entered sleep.
  [[nodiscard]] bool isSleeping() const;

  /// Return this multibody's current deactivation group index, or -1 when none.
  [[nodiscard]] int getDeactivationGroupIndex() const;

  //--------------------------------------------------------------------------
  /// @name Kinematic Structure (Design-time only)
  //--------------------------------------------------------------------------

  /// Create a link (root link with no parent joint)
  ///
  /// Creates a root link without a parent joint. This is typically the
  /// base/root of your kinematic tree.
  ///
  /// @param name Link name (empty = auto-generate "link_NNN")
  /// @return Link handle
  /// @throws InvalidArgumentException if in simulation mode
  Link addLink(std::string_view name = "");

  /// Create a link with parent joint
  ///
  /// Creates a link and its parent joint atomically using LinkOptions.
  /// This maintains kinematic tree validity.
  ///
  /// @param name Link name (empty = auto-generate "link_NNN")
  /// @param options LinkOptions specifying parent and joint configuration
  /// @return Link handle (use getParentJoint() to access the created joint)
  /// @throws InvalidArgumentException if in simulation mode or parent invalid
  Link addLink(std::string_view name, const LinkOptions& options);

  /// Create a link with parent joint
  ///
  /// Creates a link and its parent joint atomically using a parent Link and
  /// JointSpec. This is the preferred DART 7 experimental construction shape
  /// because JointSpec can be shared by C++ and dartpy.
  ///
  /// @param name Link name (empty = auto-generate "link_NNN")
  /// @param parentLink Parent link handle
  /// @param joint JointSpec describing the parent joint
  /// @return Link handle (use getParentJoint() to access the created joint)
  /// @throws InvalidArgumentException if in simulation mode, parent invalid, or
  /// joint parameters invalid
  Link addLink(
      std::string_view name,
      const Link& parentLink,
      const JointSpec& joint = {});

  //--------------------------------------------------------------------------
  /// @name Generalized-Coordinate Dynamics
  //--------------------------------------------------------------------------

  /// Get the joint-space mass matrix M(q) at the current configuration.
  ///
  /// The matrix is symmetric positive-definite and square with size
  /// getDOFCount(), ordered by joint construction order. Returns an empty
  /// matrix when the multibody has no movable degrees of freedom.
  ///
  /// Supports fixed-base trees with fixed/revolute/prismatic joints; other
  /// joint types are rejected.
  [[nodiscard]] Eigen::MatrixXd getMassMatrix() const;

  /// Get the inverse of the joint-space mass matrix at the current
  /// configuration. Returns an empty matrix when there are no movable DOFs.
  [[nodiscard]] Eigen::MatrixXd getInverseMassMatrix() const;

  /// Get the Coriolis/centrifugal generalized forces C(q, qdot) qdot at the
  /// current configuration and velocity.
  ///
  /// Size getDOFCount(), ordered by joint construction order. Empty when there
  /// are no movable DOFs.
  [[nodiscard]] Eigen::VectorXd getCoriolisForces() const;

  /// Get the gravity generalized forces g(q) at the current configuration.
  ///
  /// Size getDOFCount(), ordered by joint construction order. Empty when there
  /// are no movable DOFs.
  [[nodiscard]] Eigen::VectorXd getGravityForces() const;

  /// Get the combined Coriolis/centrifugal and gravity generalized forces
  /// C(q, qdot) qdot + g(q) at the current configuration and velocity.
  ///
  /// Size getDOFCount(), ordered by joint construction order. Empty when there
  /// are no movable DOFs.
  [[nodiscard]] Eigen::VectorXd getCoriolisAndGravityForces() const;

  /// Compute the generalized joint forces (inverse dynamics) that produce a
  /// desired generalized acceleration at the current configuration and
  /// velocity: `tau = M(q) qddot + C(q, qdot) qdot + g(q)`, including joint
  /// armature.
  ///
  /// @param desiredAcceleration Target generalized acceleration, size
  ///        getDOFCount(), ordered by joint construction order.
  /// @return The required generalized forces in the same ordering. Empty when
  ///         there are no movable DOFs.
  /// @throws InvalidArgumentException if the acceleration size does not match
  ///         the movable DOF count.
  [[nodiscard]] Eigen::VectorXd computeInverseDynamics(
      const Eigen::VectorXd& desiredAcceleration) const;

  /// Compute the generalized velocity change produced by a generalized
  /// joint-space impulse at the current configuration: `dqdot = M(q)^-1 f`,
  /// where `M` includes joint armature.
  ///
  /// This is the joint-space primitive used by impulse-based constraint
  /// solving; full constrained impulse dynamics (mapping Cartesian constraint
  /// impulses through body Jacobians) additionally requires those Jacobians.
  ///
  /// @param jointImpulse Generalized impulse, size getDOFCount(), ordered by
  ///        joint construction order.
  /// @return The generalized velocity change in the same ordering. Empty when
  ///         there are no movable DOFs.
  /// @throws InvalidArgumentException if the impulse size does not match the
  ///         movable DOF count.
  [[nodiscard]] Eigen::VectorXd computeImpulseResponse(
      const Eigen::VectorXd& jointImpulse) const;

  /// Get the body-frame spatial Jacobian of a link in this multibody.
  ///
  /// The returned 6 x getDOFCount() matrix maps the generalized velocity to the
  /// link's spatial velocity `[angular; linear]` expressed in the link's own
  /// frame, with columns ordered by joint construction order. It depends only
  /// on the current joint configuration (not on world transforms). Columns of
  /// joints that do not move the link are zero.
  ///
  /// @param link A link belonging to this multibody.
  /// @return The 6 x getDOFCount() body Jacobian.
  /// @throws InvalidArgumentException if the link belongs to a different world
  ///         or is not part of this multibody.
  [[nodiscard]] Eigen::MatrixXd getJacobian(const Link& link) const;

  /// Get the world-frame geometric Jacobian of a link in this multibody.
  ///
  /// The returned 6 x getDOFCount() matrix maps the generalized velocity to the
  /// link's spatial velocity `[angular; linear]` expressed in world axes, with
  /// the link-frame origin as the linear reference point. The link world
  /// transform is derived from the joint configuration and the base world
  /// transform.
  ///
  /// @param link A link belonging to this multibody.
  /// @return The 6 x getDOFCount() world-frame Jacobian.
  /// @throws InvalidArgumentException if the link belongs to a different world
  ///         or is not part of this multibody.
  [[nodiscard]] Eigen::MatrixXd getWorldJacobian(const Link& link) const;

  /// **EXPERIMENTAL (PLAN-084 Phase C).** Configure compliant ground contact
  /// for the variational integrator: an analytic half-space `{x : n.(x-p0) >=
  /// 0}` with penalty stiffness `k`, optional Coulomb friction `mu`, and
  /// Kelvin-Voigt normal damping `c`. Add contact points with
  /// `addGroundContactPoint()`. Only active under the `"variational
  /// integrator"` integration family; a no-op for other families. Resets any
  /// previously configured contact points.
  ///
  /// @param dualUpdateCadence Augmented-Lagrangian (C3) cadence. `0` (default)
  ///        keeps the robust C2 compliant penalty, which rests at the `mg/k`
  ///        penetration. `N > 0` enables the drift-free AL rung, advancing the
  ///        per-point duals every `N` steps so the penetration drives to ~0;
  ///        the cadence must be slower than the primal (a handful of steps) for
  ///        stability on the undamped symplectic step. Pair `N > 0` with some
  ///        `dampingCoefficient`.
  void setGroundContact(
      const Eigen::Vector3d& planeNormal,
      const Eigen::Vector3d& planePoint,
      double stiffness,
      double frictionCoefficient = 0.0,
      double frictionRegularization = 1.0e-4,
      double dampingCoefficient = 0.0,
      std::size_t dualUpdateCadence = 0);

  /// **EXPERIMENTAL (PLAN-084 Phase C).** Add a body-fixed contact point at
  /// `localPoint` (body frame) on `link`, evaluated against the plane set by
  /// `setGroundContact()`. Throws if `setGroundContact()` has not been called
  /// or `link` is not part of this multibody.
  void addGroundContactPoint(
      const Link& link, const Eigen::Vector3d& localPoint);

private:
  Entity m_entity; ///< Opaque entity token
  World* m_world;  ///< Non-owning pointer to World
};

} // namespace dart::simulation
