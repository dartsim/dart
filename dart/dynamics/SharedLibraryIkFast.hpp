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

#ifndef DART_DYNAMICS_SHAREDLIBRARYIKFAST_HPP_
#define DART_DYNAMICS_SHAREDLIBRARYIKFAST_HPP_

#include "dart/common/SharedLibrary.hpp"
#include "dart/dynamics/IkFast.hpp"

namespace dart {
namespace dynamics {

/// IkFast-based analytical inverse kinematics class.
///
/// The detail of IkFast can be found here:
/// http://openrave.org/docs/0.8.2/openravepy/ikfast/
class SharedLibraryIkFast : public IkFast
{
public:
  /// Constructor
  ///
  /// \param[in] ik The parent InverseKinematics solver that is associated with
  /// this gradient method.
  /// \param[in] filePath The path to the shared library of the IkFast binary
  /// file
  /// \param[in] dofMap The indices to the degrees-of-freedom that will be
  /// solved by IkFast. The number of DOFs can be varied depending on the IkFast
  /// solvers.
  /// \param[in] freeDofMap The indices to the DOFs that are not solved by the
  /// IkFast solver. The values of these DOFs should be set properly.
  /// \param[in] methodName The name of this analytical inverse kinematics
  /// method.
  /// \param[in] properties Properties of InverseKinematics::Analytical.
  SharedLibraryIkFast(
      InverseKinematics* ik,
      const std::string& filePath,
      const std::vector<std::size_t>& dofMap,
      const std::vector<std::size_t>& freeDofMap,
      const std::string& methodName = "IKFast",
      const Analytical::Properties& properties = Analytical::Properties());

  // Documentation inherited.
  auto clone(InverseKinematics* newIK) const
      -> std::unique_ptr<GradientMethod> override;

protected:
  // Documentation inherited.
  int getNumFreeParameters() const override;

  // Documentation inherited.
  int* getFreeParameters() const override;

  // Documentation inherited.
  int getNumJoints() const override;

  // Documentation inherited.
  int getIkRealSize() const override;

  // Documentation inherited.
  int getIkType() const override;

  // Documentation inherited.
  bool computeIk(
      const IkReal* targetTranspose,
      const IkReal* targetRotation,
      const IkReal* freeParams,
      ikfast::IkSolutionListBase<IkReal>& solutions) override;

  // Documentation inherited.
  const char* getKinematicsHash() override;

  // Documentation inherited.
  const char* getIkFastVersion() override;

  // Documentation inherited.
  void configure() const override;

protected:
  using IkFastFuncGetInt = int (*)();
  using IkFastFuncGetIntPtr = int* (*)();
  using IkFastFuncComputeIk = bool (*)(
      const IkReal* targetTranspose,
      const IkReal* targetRotation,
      const IkReal* freeParams,
      ikfast::IkSolutionListBase<IkReal>& solutions);
  using IkFastFuncGetConstCharPtr = const char* (*)();

  /// File path to the ikfast shared library.
  std::string mFilePath;

  mutable std::shared_ptr<common::SharedLibrary> mSharedLibrary;

  mutable IkFastFuncGetInt mGetNumFreeParameters;
  mutable IkFastFuncGetIntPtr mGetFreeParameters;
  mutable IkFastFuncGetInt mGetNumJoints;
  mutable IkFastFuncGetInt mGetIkRealSize;
  mutable IkFastFuncGetInt mGetIkType;
  mutable IkFastFuncComputeIk mComputeIk;
  mutable IkFastFuncGetConstCharPtr mGetKinematicsHash;
  mutable IkFastFuncGetConstCharPtr mGetIkFastVersion;

  /// Indices of the DegreeOfFreedoms associated to this IkFast.
  mutable std::vector<std::size_t> mDofs;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SHAREDLIBRARYIKFAST_HPP_
