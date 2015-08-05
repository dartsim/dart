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

#include <cassert>
#include <iostream>

#include "dart/common/Console.h"
#include "dart/common/AddonManager.h"

namespace dart {
namespace common {

//==============================================================================
void AddonManager::setAddonStates(const State& newStates)
{
  const StateMap& stateMap = newStates.getMap();

  AddonMap::iterator addons = mAddonMap.begin();
  StateMap::const_iterator states = stateMap.begin();

  while( mAddonMap.end() != addons && stateMap.end() != states )
  {
    if( addons->first == states->first )
    {
      Addon* addon = addons->second.get();
      if(addon && states->second)
        addon->setState(states->second);

      ++addons;
      ++states;
    }
    else if( addons->first < states->first )
    {
      ++addons;
    }
    else
    {
      ++states;
    }
  }
}

//==============================================================================
AddonManager::State AddonManager::getAddonStates() const
{
  StateMap states;
  for(const auto& addon : mAddonMap)
  {
    const Addon::State* state = addon.second->getState();
    if(state)
    {
      states[addon.first] = state->clone();
    }
  }

  return states;
}

//==============================================================================
void AddonManager::setAddonProperties(const Properties& newProperties)
{
  const PropertiesMap& propertiesMap = newProperties.getMap();

  AddonMap::iterator addons = mAddonMap.begin();
  PropertiesMap::const_iterator props = propertiesMap.begin();

  while( mAddonMap.end() != addons && propertiesMap.end() != props )
  {
    if( addons->first == props->first )
    {
      Addon* addon = addons->second.get();
      if(addon)
        addon->setProperties(props->second);

      ++addons;
      ++props;
    }
    else if( addons->first < props->first )
    {
      ++addons;
    }
    else
    {
      ++props;
    }
  }
}

//==============================================================================
AddonManager::Properties AddonManager::getAddonProperties() const
{
  PropertiesMap properties;
  for(const auto& addon : mAddonMap)
  {
    const Addon::Properties* prop = addon.second->getProperties();
    if(prop)
      properties[addon.first] = prop->clone();
  }

  return properties;
}

//==============================================================================
void AddonManager::duplicateAddons(const AddonManager* otherManager)
{
  if(nullptr == otherManager)
  {
    dterr << "[AddonManager::duplicateAddons] You have asked to duplicate the "
          << "Addons of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  const AddonMap& otherMap = otherManager->mAddonMap;

  AddonMap::iterator receiving = mAddonMap.begin();
  AddonMap::const_iterator incoming = otherMap.begin();

  while( otherMap.end() != incoming )
  {
    if( mAddonMap.end() == receiving )
    {
      // If we've reached the end of this Manager's AddonMap, then we should
      // just add each entry
      mAddonMap[incoming->first] = incoming->second->clone(this);
    }
    else if( receiving->first == incoming->first )
    {
      receiving->second = incoming->second->clone(this);
      ++receiving;
      ++incoming;
    }
    else if( receiving->first < incoming->first)
    {
      ++receiving;
    }
    else
    {
      // If this Manager does not have an entry corresponding to the incoming
      // Addon, then we must create it
      mAddonMap[incoming->first] = incoming->second->clone(this);
      ++incoming;
    }
  }
}

//==============================================================================
void AddonManager::matchAddons(const AddonManager* otherManager)
{
  if(nullptr == otherManager)
  {
    dterr << "[AddonManager::matchAddons] You have asked to match the Addons "
          << "of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  for(auto& addon : mAddonMap)
    addon.second = nullptr;

  duplicateAddons(otherManager);
}

//==============================================================================
void AddonManager::becomeManager(Addon* addon)
{
  addon->changeManager(this);
}

} // namespace common
} // namespace dart
