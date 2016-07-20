/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_PROXYASPECT_HPP_
#define DART_COMMON_PROXYASPECT_HPP_

#include "dart/common/detail/ProxyAspect.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class CompositeT, typename StateT>
using ProxyStateAspect = detail::ProxyStateAspect<
    common::CompositeTrackingAspect<CompositeT>, CompositeT, StateT>;

//==============================================================================
template <class CompositeT, typename PropertiesT>
using ProxyPropertiesAspect = detail::ProxyPropertiesAspect<
    common::CompositeTrackingAspect<CompositeT>, CompositeT, PropertiesT>;

//==============================================================================
template <class CompositeT, typename StateT, typename PropertiesT>
class ProxyStateAndPropertiesAspect :
    public detail::ProxyPropertiesAspect<
        ProxyStateAspect<CompositeT, StateT>,
        CompositeT, PropertiesT>
{
public:

  using State = StateT;
  using Properties = PropertiesT;
  using CompositeType = CompositeT;

  using AspectStateImpl = ProxyStateAspect<CompositeType, State>;
  using AspectPropertiesImpl = detail::ProxyPropertiesAspect<
      AspectStateImpl, CompositeType, Properties>;

  using Base = AspectPropertiesImpl;

  virtual ~ProxyStateAndPropertiesAspect() = default;

  // Forwarding constructor
  template <typename... Args>
  ProxyStateAndPropertiesAspect(Args&&... args)
    : Base(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<ProxyStateAndPropertiesAspect>();
  }

};

} // namespace common
} // namespace dart

#endif // DART_COMMON_PROXYASPECT_HPP_
