/*
 * Copyright (c) 2011-2019, The DART development contributors
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
  auto computeSolutions(const Eigen::Isometry3d& desiredBodyTf)
      -> const std::vector<InverseKinematics::Analytical::Solution>& override;

  // Documentation inherited.
  auto getDofs() const -> const std::vector<std::size_t>& override;

  /// Returns true if this IkFast is ready to solve.
  virtual bool isConfigured() const;

protected:
  virtual int getNumFreeParameters() const = 0;
  virtual int* getFreeParameters() const = 0;
  virtual int getNumJoints() const = 0;
  virtual int getIkRealSize() const = 0;
  virtual int getIkType() const = 0;

  /// Computes the inverse kinematics solutions using the generated IKFast code.
  virtual bool computeIk(
      const IkReal* mTargetTranspose,
      const IkReal* mTargetRotation,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions) = 0;

  virtual const char* getKinematicsHash() = 0;

  virtual const char* getIkFastVersion() = 0;

  /// Configure IkFast. If it's successfully configured, isConfigured() returns
  /// true.
  virtual void configure() const;

protected:
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

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_IKFAST_HPP_
