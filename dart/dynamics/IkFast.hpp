/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_DYNAMICS_IKFAST_HPP_
#define DART_DYNAMICS_IKFAST_HPP_

#include <array>

#define IKFAST_HAS_LIBRARY
#include "dart/external/ikfast/ikfast.h"

#include "dart/dynamics/InverseKinematics.hpp"

namespace dart {
namespace dynamics {

/// A base class for IkFast-based analytical inverse kinematics classes.
///
/// The detail of IkFast can be found here:
/// http://openrave.org/docs/0.8.2/openravepy/ikfast/
class IkFast : public InverseKinematics::Analytical
{
public:
  /// Inverse kinematics types supported by IkFast
  // Following conversion is referred from:
  // https://github.com/rdiankov/openrave/blob/b1ebe135b4217823ebdf56d9af5fe89b29723603/include/openrave/openrave.h#L575-L623
  enum IkType
  {
    /// End effector reaches desired 6D transformation
    TRANSFORM_6D,

    /// End effector reaches desired 3D rotation
    ROTATION_3D,

    /// End effector origin reaches desired 3D translation
    TRANSLATION_3D,

    /// Direction on end effector coordinate system reaches desired direction
    DIRECTION_3D,

    /// Ray on end effector coordinate system reaches desired global ray
    RAY_4D,

    /// Direction on end effector coordinate system points to desired 3D
    /// position
    LOOKAT_3D,

    /// End effector origin and direction reaches desired 3D translation and
    /// direction. Can be thought of as Ray IK where the origin of the ray must
    /// coincide.
    TRANSLATION_DIRECTION_5D,

    /// End effector origin reaches desired XY translation position, Z is
    /// ignored. The coordinate system with relative to the base link.
    TRANSLATION_XY_2D,

    /// 2D translation along XY plane and 1D rotation around Z axis. The offset
    /// of the rotation is measured starting at +X, so at +X is it 0, at +Y it
    /// is pi/2.
    TRANSLATION_XY_ORIENTATION_3D,

    /// Local point on end effector origin reaches desired 3D global point.
    /// Because both local point and global point can be specified, there are 6
    /// values.
    TRANSLATION_LOCAL_GLOBAL_6D,

    /// End effector origin reaches desired 3D translation, manipulator
    /// direction makes a specific angle with x-axis (defined in the
    /// manipulator base link’s coordinate system)
    TRANSLATION_X_AXIS_ANGLE_4D,

    /// End effector origin reaches desired 3D translation, manipulator
    /// direction makes a specific angle with y-axis (defined in the
    /// manipulator base link’s coordinate system)
    TRANSLATION_Y_AXIS_ANGLE_4D,

    /// End effector origin reaches desired 3D translation, manipulator
    /// direction makes a specific angle with z-axis (defined in the
    /// manipulator base link’s coordinate system)
    TRANSLATION_Z_AXIS_ANGLE_4D,

    /// End effector origin reaches desired 3D translation, manipulator
    /// direction needs to be orthogonal to z-axis and be rotated at a certain
    /// angle starting from the x-axis (defined in the manipulator base link's
    /// coordinate system)
    TRANSLATION_X_AXIS_ANGLE_Z_NORM_4D,

    /// End effector origin reaches desired 3D translation, manipulator
    /// direction needs to be orthogonal to z-axis and be rotated at a certain
    /// angle starting from the y-axis (defined in the manipulator base link's
    /// coordinate system)
    TRANSLATION_Y_AXIS_ANGLE_X_NORM_4D,

    /// End effector origin reaches desired 3D translation, manipulator
    /// direction needs to be orthogonal to z-axis and be rotated at a certain
    /// angle starting from the z-axis (defined in the manipulator base link's
    /// coordinate system)
    TRANSLATION_Z_AXIS_ANGLE_Y_NORM_4D,

    UNKNOWN,
  };

  /// Constructor
  ///
  /// \param[in] ik The parent InverseKinematics solver that is associated with
  /// this gradient method.
  /// \param[in] dofMap The indices to the degrees-of-freedom that will be
  /// solved by IkFast. The number of DOFs can be varied depending on the IkFast
  /// solvers.
  /// \param[in] freeDofMap The indices to the DOFs that are not solved by the
  /// IkFast solver. The values of these DOFs should be set properly.
  /// \param[in] methodName The name of this analytical inverse kinematics
  /// method.
  /// \param[in] properties Properties of InverseKinematics::Analytical.
  IkFast(
      InverseKinematics* ik,
      const std::vector<std::size_t>& dofMap,
      const std::vector<std::size_t>& freeDofMap,
      const std::string& methodName = "IKFast",
      const Analytical::Properties& properties = Analytical::Properties());

  // Documentation inherited.
  const std::vector<InverseKinematics::Analytical::Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredBodyTf) override;

  /// Computes forward kinematics given joint positions where the dimension is
  /// the same as getNumJoints2().
  Eigen::Isometry3d computeFk(const Eigen::VectorXd& parameters);

  /// Returns the indices of the DegreeOfFreedoms that are part of the joints
  /// that IkFast solves for.
  const std::vector<std::size_t>& getDofs() const override;

  /// Returns the indices of the DegreeOfFreedoms that are part of the joints
  /// that IkFast solves but the values should be set by the user in prior.
  const std::vector<std::size_t>& getFreeDofs() const;

  /// Returns true if this IkFast is ready to solve.
  virtual bool isConfigured() const;

  /// Returns the number of free parameters users has to set apriori.
  std::size_t getNumFreeParameters2() const;
  // TODO(JS): Rename to getNumFreeParameters() in DART 7

  /// Returns the total number of indices of the chane.
  std::size_t getNumJoints2() const;
  // TODO(JS): Rename to getNumJoints in DART 7

  /// Returns the IK type.
  IkType getIkType2() const;
  // TODO(JS): Rename to getIkType() in DART 7

  /// Returns a hash of all the chain values used for double checking that the
  /// correct IK is used.
  const std::string getKinematicsHash2() const;
  // TODO(JS): Rename to getKinematicsHash() in DART 7

  /// Returns the IkFast version used to generate this file
  std::string getIkFastVersion2() const;
  // TODO(JS): Rename to getIkFastVersion() in DART 7

protected:
  /// Returns the number of free parameters users has to set apriori.
  virtual int getNumFreeParameters() const = 0;
  // TODO(JS): Remove in DART 7

  /// Returns the indicies of the free parameters indexed by the chain joints.
  virtual int* getFreeParameters() const = 0;
  // TODO(JS): Remove in DART 7

  /// Returns the total number of indices of the chane.
  virtual int getNumJoints() const = 0;
  // TODO(JS): Remove in DART 7

  /// Returns the size in bytes of the configured number type.
  virtual int getIkRealSize() const = 0;

  /// Returns the IK type.
  virtual int getIkType() const = 0;

  /// Computes the inverse kinematics solutions using the generated IKFast code.
  virtual bool computeIk(
      const IkReal* targetTranspose,
      const IkReal* targetRotation,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions)
      = 0;

  /// Computes the forward kinematics solutions using the generated IKFast code.
  virtual void computeFk(
      const IkReal* parameters, IkReal* targetTranspose, IkReal* targetRotation)
      = 0;

  /// Returns a hash of all the chain values used for double checking that the
  /// correct IK is used.
  virtual const char* getKinematicsHash() = 0;
  // TODO(JS): Rename in DART 7

  /// Returns the IkFast version used to generate this file
  virtual const char* getIkFastVersion() = 0;
  // TODO(JS): Rename in DART 7

  /// Configure IkFast. If it's successfully configured, isConfigured() returns
  /// true.
  virtual void configure() const;

  mutable std::vector<double> mFreeParams;

  /// True if this IkFast is ready to solve.
  mutable bool mConfigured;

  /// Indices of the DegreeOfFreedoms associated to the variable parameters of
  /// this IkFast.
  mutable std::vector<std::size_t> mDofs;

  /// Indices of the DegreeOfFreedoms associated to the free parameters of this
  /// IkFast.
  mutable std::vector<std::size_t> mFreeDofs;

private:
  /// Cache data for the target rotation used by IKFast.
  std::array<IkReal, 9> mTargetRotation;

  /// Cache data for the target translation used by IKFast.
  std::array<IkReal, 3> mTargetTranspose;
};

/// Tries to wrap a single dof IK solution, from IKFast, in the range of joint
/// limits.
///
/// This function shouldn't be used for a non-cyclic DegreeOfFreedom.
///
/// If multiple solution are available (when the range is wider than 2*pi),
/// returns the closest solution to the current joint position.
///
/// \param[in] curr The current joint positions before solving IK.
/// \param[in] lb The lower joint position limit.
/// \param[in] ub The upper joint position limit.
/// \param[in,out] sol The IK solution to be wrapped. No assumption that the
/// value initially in the bounds. This value is only updated if an available
/// solution is found.
/// \return True if a solution is found. False, otherwise.
bool wrapCyclicSolution(double curr, double lb, double ub, double& sol);

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_IKFAST_HPP_
