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
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::
AddonWithProtectedPropertiesInSkeleton(
    ManagerType* mgr, const PropertiesData& properties)
  : Addon(mgr),
    mProperties(properties)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, mgr, castedManager, constructor);
  mManager = castedManager;
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
void AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::
setAddonProperties(
    const std::unique_ptr<common::Addon::Properties>& someProperties)
{
  mProperties = static_cast<const PropertiesData&>(*someProperties);

  const SkeletonPtr& skel = getSkeleton();
  if(skel)
    skel->incrementVersion();

  (*updateProperties)(static_cast<Base*>(this));
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
const common::Addon::Properties* AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::
getAddonProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
SkeletonPtr AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::getSkeleton()
{
  if(mManager)
    return mManager->getSkeleton();

  return nullptr;
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
ConstSkeletonPtr AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::getSkeleton() const
{
  if(mManager)
    return mManager->getSkeleton();

  return nullptr;
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
ManagerType* AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::getManager()
{
  return mManager;
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
const ManagerType* AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::getManager() const
{
  return mManager;
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
std::unique_ptr<common::Addon> AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::
cloneAddon(common::AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Base, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Base>(new Base(castedManager, mProperties));
}

//==============================================================================
template <class Base, typename PropertiesData,
          class ManagerType, void (*updateProperties)(Base*)>
void AddonWithProtectedPropertiesInSkeleton<
    Base, PropertiesData, ManagerType, updateProperties>::
changeManager(common::AddonManager* newManager)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, newManager, castedManager, changeManager);
  mManager = castedManager;
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
AddonWithProtectedStateAndPropertiesInSkeleton<
    Base, StateData, PropertiesData,
    ManagerType, updateState, updateProperties>::
AddonWithProtectedStateAndPropertiesInSkeleton(
    ManagerType* mgr, const StateData& state, const PropertiesData& properties)
  : common::Addon(mgr),
    mState(state),
    mProperties(properties)
{
  // Do nothing
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
AddonWithProtectedStateAndPropertiesInSkeleton<
    Base, StateData, PropertiesData,
    ManagerType, updateState, updateProperties>::
AddonWithProtectedStateAndPropertiesInSkeleton(
    ManagerType* mgr, const PropertiesData& properties, const StateData& state)
  : common::Addon(mgr),
    mState(state),
    mProperties(properties)
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE(
        Base, ManagerType, mgr, castedManager, constructor);
  mManager = castedManager;
}

//==============================================================================
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType,
          void (*updateState)(Base*), void (*updateProperties)(Base*)>
std::unique_ptr<common::Addon> AddonWithProtectedStateAndPropertiesInSkeleton<
    Base, StateData, PropertiesData,
    ManagerType, updateState, updateProperties>::
cloneAddon(common::AddonManager* newManager) const
{
  DART_COMMON_CAST_NEW_MANAGER_TYPE_AND_RETURN_NULL_IF_BAD(
        Base, ManagerType, newManager, castedManager, clone);
  return std::unique_ptr<Base>(new Base(castedManager, mState, mProperties));
}

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_DETAIL_ADDON_H_
