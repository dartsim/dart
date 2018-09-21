/*
 * Copyright (c) 2011-2018, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_CONSTRAINT_CONSTRAINTBASE_HPP_
#define DART_CONSTRAINT_CONSTRAINTBASE_HPP_

#include <cstddef>

#include <Eigen/Dense>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace constraint {

/// ConstraintInfo
struct ConstraintInfo
{
  /// Impulse
  double* x;

  /// Lower bound of x
  double* lo;

  /// Upper bound of x
  double* hi;

  /// Bias term
  double* b;

  /// Slack variable
  double* w;

  /// Friction index
  int* findex;

  /// Inverse of time step
  double invTimeStep;
};

/// Constraint is a base class of concrete constraints classes
class ConstraintBase
{
public:
  Eigen::MatrixXd mUnitResponses;

  /// Return dimesion of this constranit
  std::size_t getDimension() const;

  virtual std::size_t getNumSkeletons() const
  {
    return 0;
  }
  // TODO(JS): Make this a pure virtual functions.

  virtual dynamics::Skeleton* getSkeleton(std::size_t index)
  {
    return nullptr;
  }

  virtual const dynamics::Skeleton* getSkeleton(std::size_t index) const
  {
    return nullptr;
  }

  /// Update constraint using updated Skeleton's states
  virtual void update() = 0;

  /// Fill LCP variables
  virtual void getInformation(ConstraintInfo* info) = 0;

  /// Apply unit impulse to constraint space
  virtual void applyUnitImpulse(std::size_t index) = 0;

  /// Get velocity change due to the uint impulse
  virtual void getVelocityChange(double* vel, bool withCfm) = 0;

  virtual const Eigen::MatrixXd& getJacobian(std::size_t skeletonIndex) const
  {
    static Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(0, 0);
    return zeroMat;
  }

  virtual const Eigen::MatrixXd& getUnitResponses(
      std::size_t skeletonIndex) const
  {
    static Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(0, 0);
    return zeroMat;
  }

  virtual const Eigen::MatrixXd& getConstraintMassMatrix() const
  {
    static Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(0, 0);
    return zeroMat;
  }

  virtual const Eigen::MatrixXd& computeConstraintMassMatrix(
      const ConstraintBase& other) const
  {
    // Unit responses of this constraint against the Jacobians of other
    // constraints

    const std::size_t numSkeletonsA = getNumSkeletons();
    const std::size_t numSkeletonsB = other.getNumSkeletons();

    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(mDim, other.mDim);

    for (std::size_t i = 0; i < numSkeletonsA; ++i)
    {
      const dynamics::Skeleton* skeletonA = getSkeleton(i);
      for (std::size_t j = 0; j < numSkeletonsB; ++j)
      {
        const dynamics::Skeleton* skeletonB = getSkeleton(j);

        if (skeletonA != skeletonB)
          continue;

        //        result.noalias() += m
      }
    }

    static Eigen::MatrixXd zeroMat = Eigen::MatrixXd::Zero(0, 0);
    return zeroMat;
  }

  /// Excite the constraint
  virtual void excite() = 0;

  /// Unexcite the constraint
  virtual void unexcite() = 0;

  /// Apply computed constraint impulse to constrained skeletons
  virtual void applyImpulse(double* lambda) = 0;

  /// Return true if this constraint is active
  virtual bool isActive() const = 0;

  virtual dynamics::SkeletonPtr getRootSkeleton() const = 0;

  virtual void uniteSkeletons();

  /// Performs impulse test and returns the responses.
  ///
  /// The responses is basically a vector where the size is the DOFs of the
  /// skeleton of \c bodyNode. Each element would be either velocity change or
  /// impulse for dynamic joint and kinematic joint, respectively.
  ///
  /// We assume that all the constraint impulses are already cleared out. If not
  /// the output \c responses won't be as expected. This function clears out the
  /// constraint impulses applied to the body before returns.
  ///
  /// \param[out] responses The output vector.
  /// \param[in/out] bodyNode The body we apply impulse.
  /// \param[in] bodyImpulse The body impulse applied to \c bodyNode.
  static void computeJointImpulseResponses(
      Eigen::Ref<Eigen::VectorXd> responses,
      dynamics::BodyNode& bodyNode,
      const Eigen::Vector6d& bodyImpulse);

  /// Performs impulse test and returns the responses, which is used for
  /// self-collision cases.
  ///
  /// The responses is basically a vector where the size is the DOFs of the
  /// skeleton of \c bodyNode. Each element would be either velocity change or
  /// impulse for dynamic joint and kinematic joint, respectively.
  ///
  /// We assume that all the constraint impulses are already cleared out. If not
  /// the output \c responses won't be as expected. This function clears out the
  /// constraint impulses applied to the bodies before returns.
  ///
  /// The skeletons of the two bodies should be the same. Otherwise, it's
  /// undefined behavior.
  ///
  /// \param[in] bodyNodeA The first body we apply \c impA.
  /// \param[in] bodyImpulse The body impulse applied to \c bodyNodeA.
  /// \param[in] bodyNode The second body we apply \c impB.
  /// \param[in] bodyImpulse The body impulse applied to \c bodyNodeB.
  static void computeJointImpulseResponses(
      Eigen::VectorXd& responses,
      dynamics::BodyNode& bodyNodeA,
      const Eigen::Vector6d& impA,
      dynamics::BodyNode& bodyNodeB,
      const Eigen::Vector6d& impB);

  /// Performs impulse test and returns the responses.
  ///
  /// The responses is basically a vector where the size is the DOFs of the
  /// skeleton of \c bodyNode. Each element would be either velocity change or
  /// impulse for dynamic joint and kinematic joint, respectively.
  ///
  /// We assume that all the constraint impulses are already cleared out. If not
  /// the output \c responses won't be as expected. This function clears out the
  /// constraint impulses applied to the bodies before returns.
  ///
  /// \param[in] softBodyNode The SoftBodyNode contains \c pointMass.
  /// \param[in] pointMass The point mass we apply \c imp.
  /// \param[in] imp The linear impulse will be applied to \c pointMass.
  static void computeJointImpulseResponses(
      Eigen::VectorXd& responses,
      dynamics::SoftBodyNode& softBodyNode,
      dynamics::PointMass& pointMass,
      const Eigen::Vector3d& imp);

  static dynamics::SkeletonPtr compressPath(dynamics::SkeletonPtr skeleton);

  static dynamics::SkeletonPtr getRootSkeleton(dynamics::SkeletonPtr skeleton);

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;
  friend class ConstrainedGroup;

protected:
  /// Default contructor
  ConstraintBase();

  /// Destructor
  virtual ~ConstraintBase();

protected:
  /// Dimension of constraint
  std::size_t mDim;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_CONSTRAINTBASE_HPP_
