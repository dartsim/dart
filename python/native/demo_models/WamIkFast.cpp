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

#include "WamIkFast.hpp"

// An empty IKFAST_NAMESPACE keeps the generated solver in an anonymous
// namespace.
#define IKFAST_NAMESPACE
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/IkFast.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#define IKFAST_NO_MAIN
#include "wam_ikfast/ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp"

#undef IKFAST_NO_MAIN
#undef IKFAST_NAMESPACE
#undef IKFAST_HAS_LIBRARY
#undef IKFAST_API
#undef IKFAST_COMPILE_ASSERT
#undef IKFAST_STRINGIZE2
#undef IKFAST_STRINGIZE
#undef IKFAST_ASSERT
#undef IKFAST_ALIGNED16
#undef IK2PI
#undef IKPI
#undef IKPI_2
#undef IKFAST_SINCOS_THRESH
#undef IKFAST_ATAN2_MAGTHRESH
#undef IKFAST_SOLUTION_THRESH
#undef IKFAST_EVALCOND_THRESH
#undef __PRETTY_FUNCTION__

namespace dart_demos {
namespace {

class WamIkFast final : public dart::dynamics::IkFast
{
public:
  WamIkFast(
      dart::dynamics::InverseKinematics* ik,
      const std::string& methodName = "IKFast",
      const Analytical::Properties& properties = Analytical::Properties())
    : IkFast(ik, {0, 1, 3, 4, 5, 6}, {2}, methodName, properties)
  {
  }

  std::unique_ptr<GradientMethod> clone(
      dart::dynamics::InverseKinematics* newIK) const override
  {
    return std::make_unique<WamIkFast>(
        newIK, getMethodName(), getAnalyticalProperties());
  }

protected:
  int getNumFreeParameters() const override
  {
    return ::GetNumFreeParameters();
  }

  int* getFreeParameters() const override
  {
    return ::GetFreeParameters();
  }

  int getNumJoints() const override
  {
    return ::GetNumJoints();
  }

  int getIkRealSize() const override
  {
    return ::GetIkRealSize();
  }

  int getIkType() const override
  {
    return ::GetIkType();
  }

  bool computeIk(
      const IkReal* translation,
      const IkReal* rotation,
      const IkReal* freeParams,
      ikfast::IkSolutionListBase<IkReal>& solutions) override
  {
    return ::ComputeIk(translation, rotation, freeParams, solutions);
  }

  void computeFk(
      const IkReal* parameters, IkReal* translation, IkReal* rotation) override
  {
    ::ComputeFk(parameters, translation, rotation);
  }

  const char* getKinematicsHash() override
  {
    return ::GetKinematicsHash();
  }

  const char* getIkFastVersion() override
  {
    return ::GetIkFastVersion();
  }
};

} // namespace

//==============================================================================
void setWamIkFast(dart::dynamics::InverseKinematics& ik)
{
  const auto* ee
      = dynamic_cast<const dart::dynamics::EndEffector*>(ik.getNode());
  const auto skeleton = ee ? ee->getSkeleton() : nullptr;
  if (!skeleton || skeleton->getNumDofs() != 7
      || ee->getBodyNode() != skeleton->getBodyNode("/wam7"))
    throw std::invalid_argument("ik must belong to a WAM /wam7 end effector");

  for (std::size_t i = 0; i < 7; ++i) {
    if (skeleton->getDof(i)->getName() != "/j" + std::to_string(i + 1))
      throw std::invalid_argument("ik requires WAM DOFs /j1 through /j7");
  }

  ik.setGradientMethod<WamIkFast>();
}

} // namespace dart_demos
