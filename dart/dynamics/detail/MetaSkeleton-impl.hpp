/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_DYNAMICS_DETAIL_METASKELETON_IMPL_HPP_
#define DART_DYNAMICS_DETAIL_METASKELETON_IMPL_HPP_

#include <dart/dynamics/MetaSkeleton.hpp>

namespace dart::dynamics {

//==============================================================================
template <typename Func>
void MetaSkeleton::eachBodyNode(Func func) const
{
  if constexpr (std::is_same_v<
                    std::invoke_result_t<Func, const dynamics::BodyNode*>,
                    bool>) {
    for (auto i = 0u; i < getNumBodyNodes(); ++i) {
      if (!func(getBodyNode(i)))
        return;
    }
  } else {
    for (auto i = 0u; i < getNumBodyNodes(); ++i) {
      func(getBodyNode(i));
    }
  }
}

//==============================================================================
template <typename Func>
void MetaSkeleton::eachBodyNode(Func func)
{
  if constexpr (std::is_same_v<
                    std::invoke_result_t<Func, dynamics::BodyNode*>,
                    bool>) {
    for (auto i = 0u; i < getNumBodyNodes(); ++i) {
      if (!func(getBodyNode(i)))
        return;
    }
  } else {
    for (auto i = 0u; i < getNumBodyNodes(); ++i) {
      func(getBodyNode(i));
    }
  }
}

//==============================================================================
template <typename Func>
void MetaSkeleton::eachJoint(Func func) const
{
  if constexpr (std::is_same_v<
                    std::invoke_result_t<Func, const dynamics::Joint*>,
                    bool>) {
    for (auto i = 0u; i < getNumJoints(); ++i) {
      if (!func(getJoint(i)))
        return;
    }
  } else {
    for (auto i = 0u; i < getNumJoints(); ++i) {
      func(getJoint(i));
    }
  }
}

//==============================================================================
template <typename Func>
void MetaSkeleton::eachJoint(Func func)
{
  if constexpr (std::is_same_v<
                    std::invoke_result_t<Func, dynamics::Joint*>,
                    bool>) {
    for (auto i = 0u; i < getNumJoints(); ++i) {
      if (!func(getJoint(i)))
        return;
    }
  } else {
    for (auto i = 0u; i < getNumJoints(); ++i) {
      func(getJoint(i));
    }
  }
}

//==============================================================================
template <typename Func>
void MetaSkeleton::eachDof(Func func) const
{
  if constexpr (std::is_same_v<
                    std::
                        invoke_result_t<Func, const dynamics::DegreeOfFreedom*>,
                    bool>) {
    for (auto i = 0u; i < getNumDofs(); ++i) {
      if (!func(getDof(i)))
        return;
    }
  } else {
    for (auto i = 0u; i < getNumDofs(); ++i) {
      func(getDof(i));
    }
  }
}

//==============================================================================
template <typename Func>
void MetaSkeleton::eachDof(Func func)
{
  if constexpr (std::is_same_v<
                    std::invoke_result_t<Func, dynamics::DegreeOfFreedom*>,
                    bool>) {
    for (auto i = 0u; i < getNumDofs(); ++i) {
      if (!func(getDof(i)))
        return;
    }
  } else {
    for (auto i = 0u; i < getNumDofs(); ++i) {
      func(getDof(i));
    }
  }
}

} // namespace dart::dynamics

#endif // DART_DYNAMICS_DETAIL_METASKELETON_IMPL_HPP_
