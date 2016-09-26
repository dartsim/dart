/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_DYNAMICS_DETAIL_INVERSEKINEMATICS_HPP_
#define DART_DYNAMICS_DETAIL_INVERSEKINEMATICS_HPP_

#include <type_traits>

#include "dart/dynamics/InverseKinematics.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
template <class IKErrorMethod, typename... Args>
IKErrorMethod& InverseKinematics::setErrorMethod(Args&&... args)
{
  IKErrorMethod* newMethod =
      new IKErrorMethod(this, std::forward<Args>(args)...);
  mErrorMethod = std::unique_ptr<ErrorMethod>(newMethod);
  return *newMethod;
}

//==============================================================================
template <class IKGradientMethod, typename... Args>
IKGradientMethod& InverseKinematics::setGradientMethod(Args&&... args)
{
  IKGradientMethod* newMethod =
      new IKGradientMethod(this, std::forward<Args>(args)...);
  mGradientMethod = std::unique_ptr<GradientMethod>(newMethod);

  mAnalytical = dynamic_cast<Analytical*>(mGradientMethod.get());
  if(nullptr != mAnalytical)
    mAnalytical->constructDofMap();

  return *newMethod;
}

//==============================================================================
template <class DegreeOfFreedomT>
void InverseKinematics::setDofs(const std::vector<DegreeOfFreedomT*>& _dofs)
{
  std::vector<std::size_t> indices;
  indices.reserve(_dofs.size());
  for(const DegreeOfFreedomT* dof : _dofs)
    indices.push_back(dof->getIndexInSkeleton());

  setDofs(indices);
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_INVERSEKINEMATICS_HPP_
