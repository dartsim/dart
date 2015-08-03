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

#include "dart/common/AddonManager.h"


#include <iostream>

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
      if(addon)
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
    std::unique_ptr<Addon::State> state = addon.second->getState();
    if(state)
    {
      states[addon.first] = std::move(state);
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
    std::unique_ptr<Addon::Properties> prop = addon.second->getProperties();
    if(prop)
      properties[addon.first] = std::move(prop);
  }

  return properties;
}

//==============================================================================
void AddonManager::becomeManager(Addon* addon)
{
  addon->changeManager(this);
}














} // namespace common
} // namespace dart
