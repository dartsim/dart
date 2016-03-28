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
#include "dart/common/Composite.h"

namespace dart {
namespace common {

//==============================================================================
void Composite::setAspectStates(const State& newStates)
{
  const StateMap& stateMap = newStates.getMap();

  AspectMap::iterator aspects = mAspectMap.begin();
  StateMap::const_iterator states = stateMap.begin();

  while( mAspectMap.end() != aspects && stateMap.end() != states )
  {
    if( aspects->first == states->first )
    {
      Aspect* aspect = aspects->second.get();
      if(aspect && states->second)
        aspect->setAspectState(*states->second);

      ++aspects;
      ++states;
    }
    else if( aspects->first < states->first )
    {
      ++aspects;
    }
    else
    {
      ++states;
    }
  }
}

//==============================================================================
template <typename MapType, class DataType, const DataType* (Aspect::*getData)() const>
void extractMapData(MapType& outgoingMap, const Composite::AspectMap& aspectMap)
{
  // TODO(MXG): Consider placing this function in a header so it can be utilized
  // by anything that needs to transfer data between maps of extensibles

  // This method allows us to avoid dynamic allocation (cloning) whenever possible.
  for(const auto& aspect : aspectMap)
  {
    if(nullptr == aspect.second)
      continue;

    const DataType* data = (aspect.second.get()->*getData)();
    if(data)
    {
      // Attempt to insert a nullptr to see whether this entry exists while also
      // creating an itertor to it if it did not already exist. This allows us
      // to search for a spot in the map once, instead of searching the map to
      // see if the entry already exists and then searching the map again in
      // order to insert the entry if it didn't already exist.
      std::pair<typename MapType::iterator, bool> insertion =
          outgoingMap.insert(typename MapType::value_type(aspect.first, nullptr));

      typename MapType::iterator& it = insertion.first;
      bool existed = !insertion.second;

      if(existed)
      {
        // The entry already existed
        if(it->second)
        {
          // The entry was not a nullptr, so we can do an efficient copy
          it->second->copy(*data);
        }
        else
        {
          // The entry was a nullptr, so we need to clone
          it->second = data->clone();
        }
      }
      else
      {
        // The entry did not already exist, so we need to clone
        it->second = data->clone();
      }
    }
  }
}

//==============================================================================
Composite::State Composite::getAspectStates() const
{
  State states;
  copyAspectStatesTo(states);

  return states;
}

//==============================================================================
void Composite::copyAspectStatesTo(State& outgoingStates) const
{
  StateMap& states = outgoingStates.getMap();
  extractMapData<StateMap, Aspect::State, &Aspect::getAspectState>(states, mAspectMap);
}

//==============================================================================
void Composite::setAspectProperties(const Properties& newProperties)
{
  const PropertiesMap& propertiesMap = newProperties.getMap();

  AspectMap::iterator aspects = mAspectMap.begin();
  PropertiesMap::const_iterator props = propertiesMap.begin();

  while( mAspectMap.end() != aspects && propertiesMap.end() != props )
  {
    if( aspects->first == props->first )
    {
      Aspect* aspect = aspects->second.get();
      if(aspect)
        aspect->setAspectProperties(*props->second);

      ++aspects;
      ++props;
    }
    else if( aspects->first < props->first )
    {
      ++aspects;
    }
    else
    {
      ++props;
    }
  }
}

//==============================================================================
Composite::Properties Composite::getAspectProperties() const
{
  Properties properties;
  copyAspectPropertiesTo(properties);

  return properties;
}

//==============================================================================
void Composite::copyAspectPropertiesTo(
    Properties& outgoingProperties) const
{
  PropertiesMap& properties = outgoingProperties.getMap();
  extractMapData<PropertiesMap, Aspect::Properties, &Aspect::getAspectProperties>(
        properties, mAspectMap);
}

//==============================================================================
void Composite::duplicateAspects(const Composite* fromManager)
{
  if(nullptr == fromManager)
  {
    dterr << "[Composite::duplicateAspects] You have asked to duplicate the "
          << "Aspects of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  if(this == fromManager)
    return;

  const AspectMap& otherMap = fromManager->mAspectMap;

  AspectMap::iterator receiving = mAspectMap.begin();
  AspectMap::const_iterator incoming = otherMap.begin();

  while( otherMap.end() != incoming )
  {
    if( mAspectMap.end() == receiving )
    {
      // If we've reached the end of this Manager's AspectMap, then we should
      // just add each entry
      _set(incoming->first, incoming->second.get());
      ++incoming;
    }
    else if( receiving->first == incoming->first )
    {
      if(incoming->second)
        _set(incoming->first, incoming->second.get());

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
      // Aspect, then we must create it
      _set(incoming->first, incoming->second.get());
      ++incoming;
    }
  }
}

//==============================================================================
void Composite::matchAspects(const Composite* otherManager)
{
  if(nullptr == otherManager)
  {
    dterr << "[Composite::matchAspects] You have asked to match the Aspects "
          << "of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  for(auto& aspect : mAspectMap)
    aspect.second = nullptr;

  duplicateAspects(otherManager);
}

//==============================================================================
void Composite::becomeManager(Aspect* aspect, bool transfer)
{
  if(aspect)
    aspect->setManager(this, transfer);
}

//==============================================================================
void Composite::_set(std::type_index type_idx, const Aspect* aspect)
{
  if(aspect)
  {
    mAspectMap[type_idx] = aspect->cloneAspect(this);
    becomeManager(mAspectMap[type_idx].get(), false);
  }
  else
  {
    mAspectMap[type_idx] = nullptr;
  }
}

//==============================================================================
void Composite::_set(std::type_index type_idx, std::unique_ptr<Aspect> aspect)
{
  mAspectMap[type_idx] = std::move(aspect);
  becomeManager(mAspectMap[type_idx].get(), true);
}

} // namespace common
} // namespace dart
