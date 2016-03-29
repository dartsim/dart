/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COMMON_ADDONWITHVERSION_HPP_
#define DART_COMMON_ADDONWITHVERSION_HPP_

#include "dart/common/detail/AddonWithVersion.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class DerivedT,
          typename StateDataT,
          class ManagerT = AddonManager,
          void (*updateState)(DerivedT*) = &detail::NoOp<DerivedT*> >
using AddonWithState =
    detail::AddonWithState<ManagerTrackingAddon<ManagerT>, DerivedT, StateDataT, ManagerT, updateState>;

//==============================================================================
template <class DerivedT,
          typename PropertiesDataT,
          class ManagerT = AddonManager,
          void (*updateProperties)(DerivedT*) = &detail::NoOp<DerivedT*> >
using AddonWithVersionedProperties =
    detail::AddonWithVersionedProperties<ManagerTrackingAddon<ManagerT>, DerivedT, PropertiesDataT, ManagerT, updateProperties>;

//==============================================================================
template <class DerivedT,
          typename StateDataT,
          typename PropertiesDataT,
          class ManagerT = AddonManager,
          void (*updateState)(DerivedT*) = &detail::NoOp<DerivedT*>,
          void (*updateProperties)(DerivedT*) = updateState>
class AddonWithStateAndVersionedProperties :
    public detail::AddonWithVersionedProperties<
        AddonWithState<DerivedT, StateDataT, ManagerT, updateState>,
        DerivedT, PropertiesDataT, ManagerT, updateProperties>
{
public:

  using Derived = DerivedT;
  using StateData = StateDataT;
  using PropertiesData = PropertiesDataT;
  using ManagerType = ManagerT;
  using State = common::Addon::StateMixer<StateData>;
  using Properties = common::Addon::PropertiesMixer<PropertiesData>;
  constexpr static void (*UpdateState)(Derived*) = updateState;
  constexpr static void (*UpdateProperties)(Derived*) = updateProperties;

  using AddonStateImplementation = AddonWithState<
      Derived, StateData, ManagerType, updateState>;

  using AddonPropertiesImplementation = detail::AddonWithVersionedProperties<
      AddonStateImplementation,
      Derived, PropertiesData, ManagerType, updateProperties>;

  using AddonImplementation = AddonWithStateAndVersionedProperties<
      DerivedT, StateDataT, PropertiesDataT, ManagerT,
      updateState, updateProperties>;

  AddonWithStateAndVersionedProperties() = delete;
  AddonWithStateAndVersionedProperties(
      const AddonWithStateAndVersionedProperties&) = delete;

  /// Construct using a StateData and a PropertiesData instance
  AddonWithStateAndVersionedProperties(
      common::AddonManager* mgr,
      const StateData& state = StateData(),
      const PropertiesData& properties = PropertiesData())
    : AddonPropertiesImplementation(mgr, properties, state)
  {
    // Do nothing
  }

  /// Construct using a PropertiesData and a StateData instance
  AddonWithStateAndVersionedProperties(
      common::AddonManager* mgr,
      const PropertiesData& properties,
      const StateData& state = StateData())
    : AddonPropertiesImplementation(mgr, properties, state)
  {
    // Do nothing
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
          class ManagerT,
          void (*updateState)(DerivedT*),
          void (*updateProperties)(DerivedT*)>
constexpr void (*AddonWithStateAndVersionedProperties<DerivedT, StateDataT,
    PropertiesDataT, ManagerT, updateState, updateProperties>::UpdateState)
    (DerivedT*);

//==============================================================================
template <class DerivedT,
          typename StateDataT,
          typename PropertiesDataT,
          class ManagerT,
          void (*updateState)(DerivedT*),
          void (*updateProperties)(DerivedT*)>
constexpr void (*AddonWithStateAndVersionedProperties<DerivedT, StateDataT,
    PropertiesDataT, ManagerT, updateState, updateProperties>::UpdateProperties)
    (DerivedT*);

} // namespace common
} // namespace dart

#endif // DART_COMMON_ADDONWITHVERSION_HPP_
