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

#ifndef DART_TEST_SHAREDLIBRARYWAMIKFAST_HPP_
#define DART_TEST_SHAREDLIBRARYWAMIKFAST_HPP_

#include "dart/common/SharedLibrary.hpp"
#include "dart/dynamics/IkFast.hpp"

class SharedLibraryWamIkFast : public dart::dynamics::IkFast
{
public:
  SharedLibraryWamIkFast(
      dart::dynamics::InverseKinematics* ik,
      const std::vector<std::size_t>& dofMap,
      const std::vector<std::size_t>& freeDofMap,
      const std::string& methodName = "IKFast",
      const Analytical::Properties& properties = Analytical::Properties());

  // Documentation inherited.
  auto clone(dart::dynamics::InverseKinematics* newIK) const
      -> std::unique_ptr<GradientMethod>;

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
};

#endif // DART_TEST_SHAREDLIBRARYWAMIKFAST_HPP_
