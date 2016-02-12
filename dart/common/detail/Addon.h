/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_DETAIL_ADDON_H_
#define DART_COMMON_DETAIL_ADDON_H_

#include <cassert>

#include "dart/common/Console.h"
#include "dart/common/Addon.h"

namespace dart {
namespace common {

#define DART_COMMON_CAST_NEW_MANAGER_TYPE(Base, ManagerType, newManager,\
                                          castedManager, func)\
  ManagerType* castedManager = dynamic_cast<ManagerType*>(newManager);\
  if(nullptr == castedManager)\
  {\
    dterr << "[" << typeid(Base).name() << "::" << #func << "] Attempting to "\
          << "use a [" << typeid(newManager).name() << "] type manager, but "\
          << "this Addon is only designed to be attached to a ["\
          << typeid(ManagerType).name() << "] type manager. This may cause "\
          << "undefined behavior!\n";\
    assert(false);\
  }

#define DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(\
  Base, ManagerType, newManager, castedManager, func)\
  DART_COMMON_CAST_NEW_MANAGER_TYPE(Base, ManagerType, newManager, castedManager, func)\
  if(nullptr == castedManager) return nullptr;

namespace detail {

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
          class ManagerType, void (*updateState)(DerivedT*)>
void AddonWithState<BaseT, DerivedT, StateData, ManagerType, updateState>::
setAddonState(const Addon::State& otherState)
{
  setState(static_cast<const State&>(otherState));
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class ManagerType, void (*updateState)(DerivedT*)>
const Addon::State*
AddonWithState<BaseT, DerivedT, StateData, ManagerType, updateState>::
getAddonState() const
{
  return &mState;
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class ManagerType, void (*updateState)(DerivedT*)>
void AddonWithState<BaseT, DerivedT, StateData, ManagerType, updateState>::
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
          class ManagerType, void (*updateState)(DerivedT*)>
std::unique_ptr<Addon>
AddonWithState<BaseT, DerivedT, StateData, ManagerType, updateState>::
    cloneAddon(AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Base, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Derived>(new Derived(newManager, mState));
}

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
          class ManagerType, void (*updateProperties)(DerivedT*)>
void AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  ManagerType, updateProperties>::
setAddonProperties(const Addon::Properties& someProperties)
{
  setProperties(static_cast<const Properties&>(someProperties));
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerType, void (*updateProperties)(DerivedT*)>
const Addon::Properties*
AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             ManagerType, updateProperties>::
getAddonProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerType, void (*updateProperties)(DerivedT*)>
void AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  ManagerType, updateProperties>::
setProperties(const PropertiesData& properties)
{
  static_cast<PropertiesData&>(mProperties) = properties;
  UpdateProperties(static_cast<Derived*>(this));
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerType, void (*updateProperties)(DerivedT*)>
auto AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  ManagerType, updateProperties>::
getProperties() const -> const Properties&
{
  return mProperties;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class ManagerType, void (*updateProperties)(DerivedT*)>
std::unique_ptr<Addon>
AddonWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             ManagerType, updateProperties>::
cloneAddon(AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Derived, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Derived>(new Derived(newManager, mProperties));
}

} // namespace detail

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
AddonWithProtectedStateAndProperties<Base, StateData, PropertiesData,
                                     ManagerType,
                                     updateState, updateProperties>::
AddonWithProtectedStateAndProperties(
    AddonManager* mgr, const StateData& state, const PropertiesData& properties)
  : Addon(mgr),
    mState(state),
    mProperties(properties)
{
  // Do nothing
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
AddonWithProtectedStateAndProperties<Base, StateData, PropertiesData,
                                     ManagerType,
                                     updateState, updateProperties>::
AddonWithProtectedStateAndProperties(
    AddonManager* mgr, const PropertiesData& properties, const StateData& state)
  : Addon(mgr),
    mState(state),
    mProperties(properties)
{
  // Do nothing
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
void AddonWithProtectedStateAndProperties<Base, StateData, PropertiesData,
    ManagerType, updateState, updateProperties>::
setAddonState(const Addon::State& otherState)
{
  setState(static_cast<const State&>(otherState));
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
const Addon::State* AddonWithProtectedStateAndProperties<
    Base, StateData, PropertiesData, ManagerType,
    updateState, updateProperties>::getAddonState() const
{
  return &mState;
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
void AddonWithProtectedStateAndProperties<
    Base, StateData, PropertiesData, ManagerType,
    updateState, updateProperties>::setState(const StateData& state)
{
  static_cast<StateData&>(mState) = state;
  UpdateState(static_cast<Base*>(this));
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
auto AddonWithProtectedStateAndProperties<
    Base, StateData, PropertiesData, ManagerType,
    updateState, updateProperties>::getState() const -> const State&
{
  return mState;
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
void AddonWithProtectedStateAndProperties<Base, StateData, PropertiesData,
    ManagerType, updateState, updateProperties>::
setAddonProperties(const Addon::Properties& otherProperties)
{
  setProperties(static_cast<const Properties&>(otherProperties));
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
const Addon::Properties* AddonWithProtectedStateAndProperties<
    Base, StateData, PropertiesData, ManagerType,
    updateState, updateProperties>::getAddonProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
void AddonWithProtectedStateAndProperties<
    Base, StateData, PropertiesData, ManagerType,
    updateState, updateProperties>::
setProperties(const PropertiesData& properties)
{
  static_cast<PropertiesData&>(mProperties) = properties;
  UpdateProperties(static_cast<Base*>(this));
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
auto AddonWithProtectedStateAndProperties<
    Base, StateData, PropertiesData, ManagerType,
    updateState, updateProperties>::
getProperties() const -> const Properties&
{
  return mProperties;
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
std::unique_ptr<Addon>
AddonWithProtectedStateAndProperties<Base, StateData, PropertiesData,
                                     ManagerType,
                                     updateState, updateProperties>::
cloneAddon(AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Base, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Base>(new Base(castedManager, mState, mProperties));
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ADDON_H_
