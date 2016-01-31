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

#ifndef DART_DYNAMICS_DETAIL_ADDON_H_
#define DART_DYNAMICS_DETAIL_ADDON_H_

#include "dart/dynamics/Addon.h"

namespace dart {
namespace dynamics {

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
AddonWithProtectedPropertiesInSkeleton(
    common::AddonManager* mgr, const PropertiesData& properties)
  : Addon(mgr),
    mProperties(properties)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, mgr, castedManager, constructor);
  mManager = castedManager;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
std::unique_ptr<common::Addon> AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
cloneAddon(common::AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Base, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Base>(new Base(castedManager, mProperties));
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
setAddonProperties(const Addon::Properties& someProperties)
{
  setProperties(static_cast<const Properties&>(someProperties));
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
const common::Addon::Properties* AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
getAddonProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
setProperties(const PropertiesData& properties)
{
  static_cast<PropertiesData&>(mProperties) = properties;

  UpdateProperties(static_cast<Base*>(this));
  incrementSkeletonVersion();
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
auto AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
getProperties() const -> const Properties&
{
  return mProperties;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
bool AddonWithProtectedPropertiesInSkeleton<
  BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
isOptional(common::AddonManager* oldManager)
{
  if(Optional)
    return true;

  // If the Addon is not optional, we should check whether the Manager type is
  // the kind that this Addon belongs to.
  return (nullptr == dynamic_cast<ManagerType*>(oldManager));
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
SkeletonPtr AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::getSkeleton()
{
  if(mManager)
    return mManager->getSkeleton();

  return nullptr;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
ConstSkeletonPtr AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::getSkeleton() const
{
  if(mManager)
    return mManager->getSkeleton();

  return nullptr;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
ManagerT* AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::getManager()
{
  return mManager;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
const ManagerT* AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::getManager() const
{
  return mManager;
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
incrementSkeletonVersion()
{
  if(const SkeletonPtr& skel = getSkeleton())
    skel->incrementVersion();
}

//==============================================================================
template <class BaseT, typename PropertiesDataT,
          class ManagerT, void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedPropertiesInSkeleton<
    BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>::
setManager(common::AddonManager* newManager, bool /*transfer*/)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, newManager, castedManager, setManager);
  mManager = castedManager;

  if(mManager)
  {
    UpdateProperties(static_cast<Base*>(this));
    incrementSkeletonVersion();
  }
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
AddonWithProtectedStateAndPropertiesInSkeleton(
    common::AddonManager* mgr,
    const StateData& state,
    const PropertiesData& properties)
  : common::Addon(mgr),
    mState(state),
    mProperties(properties)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, mgr, castedManager, constructor);
  mManager = castedManager;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
AddonWithProtectedStateAndPropertiesInSkeleton(
    common::AddonManager* mgr,
    const PropertiesData& properties,
    const StateData& state)
  : common::Addon(mgr),
    mState(state),
    mProperties(properties)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, mgr, castedManager, constructor);
  mManager = castedManager;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
std::unique_ptr<common::Addon> AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
cloneAddon(common::AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Base, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Base>(new Base(castedManager, mState, mProperties));
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
setAddonState(const Addon::State& otherState)
{
  setState(static_cast<const State&>(otherState));
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
const common::Addon::State* AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getAddonState() const
{
  return &mState;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
setState(const StateData& state)
{
  static_cast<StateData&>(mState) = state;
  UpdateState(static_cast<Base*>(this));
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
auto AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getState() const -> const State&
{
  return mState;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
setAddonProperties(const Addon::Properties& properties)
{
  setProperties(static_cast<const Properties&>(properties));
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
const common::Addon::Properties* AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getAddonProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
setProperties(const PropertiesData& properties)
{
  static_cast<PropertiesData&>(mProperties) = properties;

  UpdateProperties(static_cast<Base*>(this));
  incrementSkeletonVersion();
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
auto AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getProperties() const -> const Properties&
{
  return mProperties;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
bool AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
isOptional(common::AddonManager* oldManager)
{
  if(Optional)
    return true;

  // If the Addon is not optional, we should check whether the Manager type is
  // the kind that this Addon belongs to.
  return (nullptr == dynamic_cast<ManagerType*>(oldManager));
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
SkeletonPtr AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getSkeleton()
{
  if(mManager)
    return mManager->getSkeleton();

  return nullptr;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
ConstSkeletonPtr AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getSkeleton() const
{
  if(mManager)
    return mManager->getSkeleton();

  return nullptr;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
ManagerT* AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getManager()
{
  return mManager;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
const ManagerT* AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
getManager() const
{
  return mManager;
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
incrementSkeletonVersion()
{
  if(const SkeletonPtr& skel = getSkeleton())
    skel->incrementVersion();
}

//==============================================================================
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT, void (*updateState)(BaseT*),
          void (*updateProperties)(BaseT*), bool OptionalT>
void AddonWithProtectedStateAndPropertiesInSkeleton<
    BaseT, StateDataT, PropertiesDataT,
    ManagerT, updateState, updateProperties, OptionalT>::
setManager(common::AddonManager* newManager, bool /*transfer*/)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, newManager, castedManager, setManager);

  mManager = castedManager;

  if(mManager)
  {
    UpdateState(static_cast<Base*>(this));
    UpdateProperties(static_cast<Base*>(this));
    incrementSkeletonVersion();
  }
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_ADDON_H_
