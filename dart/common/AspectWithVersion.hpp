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

#ifndef DART_COMMON_ASPECTWITHVERSION_HPP_
#define DART_COMMON_ASPECTWITHVERSION_HPP_

#include "dart/common/detail/AspectWithVersion.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class DerivedT,
          typename StateDataT,
          class CompositeT = Composite,
          void (*updateState)(DerivedT*) = &detail::NoOp<DerivedT*> >
using AspectWithState =
    detail::AspectWithState<CompositeTrackingAspect<CompositeT>, DerivedT, StateDataT, CompositeT, updateState>;

//==============================================================================
template <class DerivedT,
          typename PropertiesDataT,
          class CompositeT = Composite,
          void (*updateProperties)(DerivedT*) = &detail::NoOp<DerivedT*> >
using AspectWithVersionedProperties =
    detail::AspectWithVersionedProperties<CompositeTrackingAspect<CompositeT>, DerivedT, PropertiesDataT, CompositeT, updateProperties>;

//==============================================================================
template <class DerivedT,
          typename StateDataT,
          typename PropertiesDataT,
          class CompositeT = Composite,
          void (*updateState)(DerivedT*) = &detail::NoOp<DerivedT*>,
          void (*updateProperties)(DerivedT*) = updateState>
class AspectWithStateAndVersionedProperties :
    public detail::AspectWithVersionedProperties<
        AspectWithState<DerivedT, StateDataT, CompositeT, updateState>,
        DerivedT, PropertiesDataT, CompositeT, updateProperties>
{
public:

  using Derived = DerivedT;
  using StateData = StateDataT;
  using PropertiesData = PropertiesDataT;
  using CompositeType = CompositeT;
  using State = common::Aspect::MakeState<StateData>;
  using Properties = common::Aspect::MakeProperties<PropertiesData>;
  constexpr static void (*UpdateState)(Derived*) = updateState;
  constexpr static void (*UpdateProperties)(Derived*) = updateProperties;

  using AspectStateImpl = AspectWithState<
      Derived, StateData, CompositeType, updateState>;

  using AspectPropertiesImpl = detail::AspectWithVersionedProperties<
      AspectStateImpl,
      Derived, PropertiesData, CompositeType, updateProperties>;

  using AspectImpl = AspectWithStateAndVersionedProperties<
      DerivedT, StateDataT, PropertiesDataT, CompositeT,
      updateState, updateProperties>;

  AspectWithStateAndVersionedProperties() = delete;
  AspectWithStateAndVersionedProperties(
      const AspectWithStateAndVersionedProperties&) = delete;

  /// Construct using a StateData and a PropertiesData instance
  AspectWithStateAndVersionedProperties(
      const StateData& state = StateData(),
      const PropertiesData& properties = PropertiesData())
    : AspectPropertiesImpl(properties, state)
  {
    // Do nothing
  }

  /// Construct using a PropertiesData and a StateData instance
  AspectWithStateAndVersionedProperties(
      const PropertiesData& properties,
      const StateData& state = StateData())
    : AspectPropertiesImpl(properties, state)
  {
    // Do nothing
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<Derived>(this->getState(), this->getProperties());
  }

};

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <class DerivedT,
          typename StateDataT,
          typename PropertiesDataT,
          class CompositeT,
          void (*updateState)(DerivedT*),
          void (*updateProperties)(DerivedT*)>
constexpr void (*AspectWithStateAndVersionedProperties<DerivedT, StateDataT,
    PropertiesDataT, CompositeT, updateState, updateProperties>::UpdateState)
    (DerivedT*);

//==============================================================================
template <class DerivedT,
          typename StateDataT,
          typename PropertiesDataT,
          class CompositeT,
          void (*updateState)(DerivedT*),
          void (*updateProperties)(DerivedT*)>
constexpr void (*AspectWithStateAndVersionedProperties<DerivedT, StateDataT,
    PropertiesDataT, CompositeT, updateState, updateProperties>::UpdateProperties)
    (DerivedT*);

} // namespace common
} // namespace dart

#endif // DART_COMMON_ASPECTWITHVERSION_HPP_
