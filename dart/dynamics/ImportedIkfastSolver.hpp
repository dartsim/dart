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

#ifndef DART_DYNAMICS_IMPORTEDIKFASTSOLVER_HPP_
#define DART_DYNAMICS_IMPORTEDIKFASTSOLVER_HPP_

#include <memory>
#include "dart/common/SharedLibrary.hpp"
#include "dart/dynamics/IkfastSolver.hpp"

namespace dart {
namespace dynamics {

/// A wrapper class of the generated IKFast code.
class ImportedIkfastSolver : public IkfastSolver
{
public:
  /// Constructor
  ImportedIkfastSolver(const std::string& fileName);
  // TODO(JS): replace with uri and retriever.

  // Documentation inherited
  int getNumFreeParameters() override;

  // Documentation inherited
  int* getFreeParameters() override;

  // Documentation inherited
  int getNumJoints() override;

  // Documentation inherited
  int getIkRealSize() override;

  // Documentation inherited
  int getIkType() override;

  // Documentation inherited
  bool computeIk(
      const IkReal* eetrans,
      const IkReal* eerot,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions) override;

  // Documentation inherited
  const char* getKinematicsHash() override;

  // Documentation inherited
  const char* getIkFastVersion() override;

protected:
  using IkfastFuncGetInt = int (*)();
  using IkfastFuncGetIntPtr = int* (*)();
  using IkfastFuncComputeIk = bool (*)(
      const IkReal* eetrans,
      const IkReal* eerot,
      const IkReal* pfree,
      ikfast::IkSolutionListBase<IkReal>& solutions);
  using IkfastFuncGetConstCharPtr = const char* (*)();

  std::shared_ptr<common::SharedLibrary> mSharedLibrary;

  IkfastFuncGetInt mGetNumFreeParameters;
  IkfastFuncGetIntPtr mGetFreeParameters;
  IkfastFuncGetInt mGetNumJoints;
  IkfastFuncGetInt mGetIkRealSize;
  IkfastFuncGetInt mGetIkType;
  IkfastFuncComputeIk mComputeIk;
  IkfastFuncGetConstCharPtr mGetKinematicsHash;
  IkfastFuncGetConstCharPtr mGetIkFastVersion;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_IMPORTEDIKFASTSOLVER_HPP_
