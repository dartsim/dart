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

#ifndef DART_DYNAMICS_CONTACTINVERSEDYNAMICS_HPP_
#define DART_DYNAMICS_CONTACTINVERSEDYNAMICS_HPP_

#include "dart/dynamics/SmartPointer.hpp"

#include <Eigen/Core>

#include <vector>

#include <cstddef>

namespace dart {
namespace dynamics {

class BodyNode;

/// Computes joint forces together with friction-consistent contact forces for
/// a Skeleton that tracks a desired motion while in contact.
///
/// Skeleton::computeInverseDynamics() returns the generalized forces that
/// realize the current accelerations, but for a floating-base Skeleton the
/// root (unactuated) components of that result cannot be produced by
/// actuators; they must be realized by contact forces. This solver
/// distributes the unactuated components over a set of user-specified contact
/// points, keeping every contact force inside its (linearized) Coulomb
/// friction cone, and returns the corrected joint forces along with the
/// per-contact forces.
///
/// Each contact force is parameterized as a nonnegative combination of
/// friction-cone edge directions, which turns the distribution problem into a
/// small nonnegative least squares problem (see
/// math::solveNonNegativeLeastSquares()); no general-purpose QP solver is
/// involved. When the requested motion cannot be realized with the given
/// contacts (for example, when it would require pulling on the ground or
/// exceeding the friction limit), the residual on the unactuated components
/// is reported and Result::feasible is set to false.
///
/// The solver reads the Skeleton's current positions, velocities, and
/// accelerations, exactly like Skeleton::computeInverseDynamics(). The
/// Skeleton's generalized joint forces and joint commands are saved and
/// restored internally, so they are unchanged when compute() returns; apply
/// the output with Skeleton::setForces(Result::jointForces) if desired.
/// As with calling Skeleton::computeInverseDynamics() directly, the
/// per-body transmitted wrenches reported by BodyNode::getBodyForce() and
/// Joint::getWrenchToChildBodyNode()/getWrenchToParentBodyNode() reflect the
/// inverse-dynamics pass afterwards.
///
/// compute() mutates the Skeleton (force state and lazy kinematics caches)
/// while it runs, so the Skeleton must not be accessed concurrently. For
/// parallel evaluation, use one solver and one Skeleton clone per thread
/// (see Skeleton::cloneSkeleton()).
class ContactInverseDynamics
{
public:
  /// Specification of a single contact point.
  struct Contact
  {
    /// Body in contact. Must belong to the solver's Skeleton.
    BodyNode* bodyNode = nullptr;

    /// Contact point expressed in the coordinates of bodyNode. Must be
    /// finite.
    Eigen::Vector3d localOffset = Eigen::Vector3d::Zero();

    /// Contact normal in world coordinates, pointing from the environment
    /// into the body. Does not need to be unit length, but must be finite
    /// and nonzero.
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

    /// Coulomb friction coefficient. Must be finite and non-negative. Zero
    /// restricts the contact force to the normal direction.
    double frictionCoeff = 1.0;

    /// Number of edge directions of the linearized friction cone. Must be at
    /// least 3 when frictionCoeff is positive; ignored when frictionCoeff is
    /// zero.
    std::size_t numBasis = 4;
  };

  /// Output of compute().
  struct Result
  {
    /// Generalized forces after subtracting the contribution of the contact
    /// forces. The unactuated components equal unactuatedResidual; they are
    /// approximately zero when the solution is feasible. Empty on invalid
    /// input.
    Eigen::VectorXd jointForces;

    /// Contact force at each contact point in world coordinates, in the same
    /// order as the contacts passed to setContacts().
    std::vector<Eigen::Vector3d> contactForces;

    /// Leftover generalized forces on the unactuated degrees of freedom that
    /// the contact forces could not realize.
    Eigen::VectorXd unactuatedResidual;

    /// True when the solver converged and the unactuated residual is within
    /// the residual tolerance.
    bool feasible = false;
  };

  /// Creates a solver for the given Skeleton.
  explicit ContactInverseDynamics(SkeletonPtr skeleton);

  /// Returns the Skeleton this solver operates on.
  const SkeletonPtr& getSkeleton() const;

  /// Sets the contact points used by compute().
  void setContacts(const std::vector<Contact>& contacts);

  /// Returns the contact points used by compute().
  const std::vector<Contact>& getContacts() const;

  /// Sets the Tikhonov regularization weight on the contact force
  /// coefficients. Larger values distribute forces more evenly across the
  /// contacts at the cost of a slightly larger residual. Must be
  /// non-negative.
  void setRegularization(double regularization);

  /// Returns the regularization weight.
  double getRegularization() const;

  /// Sets the relative tolerance on the unactuated residual used to decide
  /// Result::feasible.
  void setResidualTolerance(double tolerance);

  /// Returns the residual tolerance.
  double getResidualTolerance() const;

  /// Overrides the automatically detected unactuated degrees of freedom with
  /// an explicit list of Skeleton DOF indices.
  void setUnactuatedDofs(const std::vector<std::size_t>& indices);

  /// Returns the effective unactuated DOF indices: the explicit list when one
  /// was set, otherwise every DOF of each tree root joint that has six DOFs
  /// (i.e., floating bases).
  std::vector<std::size_t> getUnactuatedDofs() const;

  /// Computes joint forces and contact forces for the Skeleton's current
  /// positions, velocities, and accelerations. The flags mirror
  /// Skeleton::computeInverseDynamics(). The Skeleton's generalized joint
  /// forces and joint commands are left unchanged; see the class
  /// documentation for the side effects shared with
  /// Skeleton::computeInverseDynamics().
  Result compute(
      bool withExternalForces = false,
      bool withDampingForces = false,
      bool withSpringForces = false);

private:
  /// Skeleton this solver operates on.
  SkeletonPtr mSkeleton;

  /// Contact points used by compute().
  std::vector<Contact> mContacts;

  /// Tikhonov regularization weight on the contact force coefficients.
  double mRegularization;

  /// Relative tolerance on the unactuated residual.
  double mResidualTolerance;

  /// Explicit unactuated DOF indices; used when mHasUnactuatedDofsOverride.
  std::vector<std::size_t> mUnactuatedDofs;

  /// Whether setUnactuatedDofs() was called.
  bool mHasUnactuatedDofsOverride;

  /// Workspace: stacked generator directions for all contacts.
  std::vector<Eigen::Matrix3Xd> mGenerators;

  /// Workspace: full-row generalized-force contribution of each generator.
  Eigen::MatrixXd mFullColumns;

  /// Workspace: NNLS system matrix (unactuated rows plus regularization).
  Eigen::MatrixXd mSystem;

  /// Workspace: NNLS right-hand side.
  Eigen::VectorXd mRhs;

  /// Workspace: NNLS solution.
  Eigen::VectorXd mCoefficients;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_CONTACTINVERSEDYNAMICS_HPP_
