/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_DYNAMICS_IMPORTEDIKFAST_HPP_
#define DART_DYNAMICS_IMPORTEDIKFAST_HPP_

#include "dart/common/SharedLibrary.hpp"
#include "dart/dynamics/Ikfast.hpp"

namespace dart {
namespace dynamics {

class ImportedIkfast : public Ikfast
{
public:
  ImportedIkfast(
      InverseKinematics* ik,
      const std::string& fileName,
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
      const IkReal* mTargetTranspose,
      const IkReal* mTargetRotation,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions) override;

  // Documentation inherited.
  const char* getKinematicsHash() override;

  // Documentation inherited.
  const char* getIkFastVersion() override;

  // Documentation inherited.
  void configure() const override;

protected:
  using IkfastFuncGetInt = int (*)();
  using IkfastFuncGetIntPtr = int* (*)();
  using IkfastFuncComputeIk = bool (*)(
      const IkReal* mTargetTranspose,
      const IkReal* mTargetRotation,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions);
  using IkfastFuncGetConstCharPtr = const char* (*)();

  /// Filename of the ikfast shared library.
  std::string mFileName;

  mutable std::shared_ptr<common::SharedLibrary> mSharedLibrary;

  mutable IkfastFuncGetInt mGetNumFreeParameters;
  mutable IkfastFuncGetIntPtr mGetFreeParameters;
  mutable IkfastFuncGetInt mGetNumJoints;
  mutable IkfastFuncGetInt mGetIkRealSize;
  mutable IkfastFuncGetInt mGetIkType;
  mutable IkfastFuncComputeIk mComputeIk;
  mutable IkfastFuncGetConstCharPtr mGetKinematicsHash;
  mutable IkfastFuncGetConstCharPtr mGetIkFastVersion;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_IMPORTEDIKFAST_HPP_
