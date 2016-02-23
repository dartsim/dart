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

#ifndef DART_COMMON_DETAIL_ADDONWITHVERSION_H_
#define DART_COMMON_DETAIL_ADDONWITHVERSION_H_

#include "dart/common/Addon.h"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
/// AddonWithProtectedState generates implementations of the State managing
/// functions for an Addon class.
template <class BaseT, class DerivedT, typename StateDataT,
          class ManagerT = AddonManager,
          void (*updateState)(DerivedT*) = &detail::NoOp<DerivedT*> >
class AddonWithState : public BaseT
{
public:

  using Base = BaseT;
  using Derived = DerivedT;
  using StateData = StateDataT;
  using ManagerType = ManagerT;
  using State = Addon::StateMixer<StateData>;
  constexpr static void (*UpdateState)(Derived*) = updateState;

  using AddonImplementation = AddonWithState<
      Base, Derived, StateData, ManagerT, UpdateState>;

  AddonWithState() = delete;
  AddonWithState(const AddonWithState&) = delete;

  /// Construct using a StateData instance
  AddonWithState(AddonManager* mgr, const StateData& state = StateData());

  /// Construct this Addon and pass args into the constructor of the Base class
  template <typename... BaseArgs>
  AddonWithState(AddonManager* mgr, const StateData& state,
                 BaseArgs&&... args)
    : Base(mgr, std::forward<BaseArgs>(args)...),
      mState(state)
  {
    // Do nothing
  }

  // Documentation inherited
  void setAddonState(const Addon::State& otherState) override final;

  // Documentation inherited
  const Addon::State* getAddonState() const override final;

  /// Set the State of this Addon
  void setState(const StateData& state);

  /// Get the State of this Addon
  const State& getState() const;

  // Documentation inherited
  std::unique_ptr<Addon> cloneAddon(
      AddonManager* newManager) const override;

protected:

  /// State of this Addon
  State mState;
};

//==============================================================================
/// AddonWithProtectedProperties generates implementations of the Property
/// managing functions for an Addon class.
template <class BaseT, class DerivedT, typename PropertiesDataT,
          class ManagerT = AddonManager,
          void (*updateProperties)(DerivedT*) = &detail::NoOp<DerivedT*> >
class AddonWithVersionedProperties : public BaseT
{
public:

  using Base = BaseT;
  using Derived = DerivedT;
  using PropertiesData = PropertiesDataT;
  using ManagerType = ManagerT;
  using Properties = Addon::PropertiesMixer<PropertiesData>;
  constexpr static void (*UpdateProperties)(Derived*) = updateProperties;

  using AddonImplementation = AddonWithVersionedProperties<
      Base, Derived, PropertiesData, ManagerT, UpdateProperties>;

  AddonWithVersionedProperties() = delete;
  AddonWithVersionedProperties(const AddonWithVersionedProperties&) = delete;

  /// Construct using a PropertiesData instance
  AddonWithVersionedProperties(
      AddonManager* mgr, const PropertiesData& properties = PropertiesData());

  /// Construct this Addon and pass args into the constructor of the Base class
  template <typename... BaseArgs>
  AddonWithVersionedProperties(
      AddonManager* mgr, const PropertiesData& properties, BaseArgs&&... args)
    : Base(mgr, std::forward<BaseArgs>(args)...),
      mProperties(properties)
  {
    // Do nothing
  }

  // Documentation inherited
  void setAddonProperties(const Addon::Properties& someProperties) override final;

  // Documentation inherited
  const Addon::Properties* getAddonProperties() const override final;

  /// Set the Properties of this Addon
  void setProperties(const PropertiesData& properties);

  /// Get the Properties of this Addon
  const Properties& getProperties() const;

  // Documentation inherited
  std::unique_ptr<Addon> cloneAddon(
      AddonManager* newManager) const override;

  size_t incrementVersion();

protected:

  /// Properties of this Addon
  Properties mProperties;

};

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <class BaseT, class DerivedT, typename StateDataT,
          class ManagerT, void (*updateState)(DerivedT*)>
constexpr void (*AddonWithState<
    BaseT, DerivedT, StateDataT, ManagerT, updateState>::UpdateState)(
    DerivedT*);

//==============================================================================
template <class BaseT, class DerivedT, typename StateDataT,
          class ManagerT, void (*updateState)(DerivedT*)>
AddonWithState<BaseT, DerivedT, StateDataT, ManagerT, updateState>::
AddonWithState(AddonManager* mgr, const StateDataT& state)
  : BaseT(mgr),
    mState(state)
{
  // Do nothing
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class ManagerT, void (*updateState)(DerivedT*)>
void AddonWithState<BaseT, DerivedT, StateData, ManagerT, updateState>::
setAddonState(const Addon::State& otherState)
{
  setState(static_cast<const State&>(otherState));
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class ManagerT, void (*updateState)(DerivedT*)>
const Addon::State*
AddonWithState<BaseT, DerivedT, StateData, ManagerT, updateState>::
getAddonState() const
{
  return &mState;
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class ManagerT, void (*updateState)(DerivedT*)>
void AddonWithState<BaseT, DerivedT, StateData, ManagerT, updateState>::
setState(const StateData& state)
{
  static_cast<StateData&>(mState) = state;
  UpdateState(static_cast<Derived*>(this));
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateDataT,
          class ManagerT, void (*updateState)(DerivedT*)>
auto AddonWithState<BaseT, DerivedT, StateDataT, ManagerT, updateState>::
getState() const -> const State&
{
  return mState;
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class ManagerT, void (*updateState)(DerivedT*)>
std::unique_ptr<Addon>
AddonWithState<BaseT, DerivedT, StateData, ManagerT, updateState>::
    cloneAddon(AddonManager* newManager) const
{
  return std::unique_ptr<Derived>(new Derived(newManager, mState));
}

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <class BaseT, class DerivedT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(DerivedT*)>
constexpr void (*AddonWithVersionedProperties<BaseT, DerivedT, PropertiesDataT,
                                              ManagerT, updateProperties>::
UpdateProperties)(DerivedT*);

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(DerivedT*)>
AddonWithVersionedProperties<BaseT, DerivedT, PropertiesDataT,
                             ManagerT, updateProperties>::
AddonWithVersionedProperties(
    AddonManager* mgr, const PropertiesData& properties)
  : BaseT(mgr),
    mProperties(properties)
{
  // Do nothing
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerT, void (*updateProperties)(DerivedT*)>
void AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  ManagerT, updateProperties>::
setAddonProperties(const Addon::Properties& someProperties)
{
  setProperties(static_cast<const Properties&>(someProperties));
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerT, void (*updateProperties)(DerivedT*)>
const Addon::Properties*
AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             ManagerT, updateProperties>::
getAddonProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerT, void (*updateProperties)(DerivedT*)>
void AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  ManagerT, updateProperties>::
setProperties(const PropertiesData& properties)
{
  static_cast<PropertiesData&>(mProperties) = properties;
  UpdateProperties(static_cast<Derived*>(this));
  this->incrementVersion();
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerT, void (*updateProperties)(DerivedT*)>
auto AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  ManagerT, updateProperties>::
getProperties() const -> const Properties&
{
  return mProperties;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerT, void (*updateProperties)(DerivedT*)>
std::unique_ptr<Addon>
AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             ManagerT, updateProperties>::
cloneAddon(AddonManager* newManager) const
{
  return std::unique_ptr<Derived>(new Derived(newManager, mProperties));
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerT, void (*updateProperties)(DerivedT*)>
size_t AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             ManagerT, updateProperties>::incrementVersion()
{
  if(ManagerType* mgr = this->getManager())
    return mgr->incrementVersion();

  return 0;
}

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ADDONWITHVERSION_H_
