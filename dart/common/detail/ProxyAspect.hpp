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

#ifndef DART_COMMON_DETAIL_PROXYASPECT_HPP_
#define DART_COMMON_DETAIL_PROXYASPECT_HPP_

#include "dart/common/Aspect.hpp"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
template <class BaseT, class CompositeT, typename StateT>
class ProxyStateAspect : public BaseT
{
public:

  using Base = BaseT;
  using CompositeType = CompositeT;
  using State = StateT;

  virtual ~ProxyStateAspect() = default;

  /// General constructor
  template <typename... Args>
  ProxyStateAspect(Args&&... args)
    : Base(std::forward<Args>(args)...),
      mProxyState()
  {
    // Do nothing
  }

  // Documentation inherited
  void setAspectState(const Aspect::State& state) override final
  {
    mProxyState.set(static_cast<const State&>(state));
  }

  // Documentation inherited
  const Aspect::State* getAspectState() const override final
  {
    return &mProxyState;
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<ProxyStateAspect>();
  }

protected:

  /// Reconfigure the Aspect to link it to this Aspect's new Composite
  void setComposite(Composite* newComposite) override
  {
    Base::setComposite(newComposite);

    // Check if the Composite is the correct Owner type
    typename State::Owner* owner =
        dynamic_cast<typename State::Owner*>(newComposite);

    if(owner && mProxyState.getOwner() != owner)
    {
      // Link the ProxyState to its new owner
      mProxyState = State(owner);
    }
  }

  /// Reconfigure the Aspect to unlink it from this Aspect's old Composite
  void loseComposite(Composite* oldComposite) override
  {
    mProxyState = State(mProxyState.get());
    Base::loseComposite(oldComposite);
  }

  /// Proxy state for this Aspect
  State mProxyState;

};

//==============================================================================
template <class BaseT, class CompositeT, typename PropertiesT>
class ProxyPropertiesAspect : public BaseT
{
public:

  using Base = BaseT;
  using CompositeType = CompositeT;
  using Properties = PropertiesT;

  virtual ~ProxyPropertiesAspect() = default;

  /// General constructor
  template <typename... Args>
  ProxyPropertiesAspect(Args&&... args)
    : Base(std::forward<Args>(args)...),
      mProxyProperties()
  {
    // Do nothing
  }

  // Documentation inherited
  void setAspectProperties(const Aspect::Properties& properties) override final
  {
    mProxyProperties.set(static_cast<const Properties&>(properties));
  }

  // Documentation inherited
  const Aspect::Properties* getAspectProperties() const override final
  {
    return &mProxyProperties;
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<ProxyPropertiesAspect>();
  }

protected:

  /// Reconfigure the Aspect to link it to this Aspect's new Composite
  void setComposite(Composite* newComposite) override
  {
    Base::setComposite(newComposite);
    typename Properties::Owner* comp =
        dynamic_cast<typename Properties::Owner*>(newComposite);

    if(comp && mProxyProperties.getOwner() != comp)
    {
      // Link the ProxyProperties to its new owner
      mProxyProperties = Properties(comp);
    }
  }

  /// Reconfigure the Aspect to unlink it from this Aspect's old Composite
  void loseComposite(Composite* oldComposite) override
  {
    mProxyProperties = Properties(mProxyProperties.get());
    Base::loseComposite(oldComposite);
  }

  /// Proxy properties for this Aspect
  Properties mProxyProperties;

};

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_PROXYASPECT_HPP_
