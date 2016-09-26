/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include <cassert>
#include <iostream>

#include "dart/common/Console.hpp"
#include "dart/common/Composite.hpp"

namespace dart {
namespace common {

//==============================================================================
/// Type maps are std::map containers which map an object's Type Info to some
/// instance or trait of that type. For example, an ObjectMap will map an
/// object's type to a std::unique_ptr of an instance of that object. a StateMap
/// will map an Object's type to a std::unique_ptr of a State instance for that
/// Object. Type maps are used for the dart::common::Aspect class.
///
/// This function will move data from an Object instance into a container where
/// the data is sorted by the Object type that it belongs to. If the DataMap
/// that is being filled with data already has an instance of the data for a
/// particular Object type, it will perform a copy instead of a clone to improve
/// performance.
template <typename ObjectType, class DataType,
          const DataType* (ObjectType::*getData)() const,
          typename ObjectMap = std::map< std::type_index, std::unique_ptr<ObjectType> >,
          typename DataMap = std::map< std::type_index, std::unique_ptr<DataType> > >
static void extractDataFromObjectTypeMap(
    DataMap& dataMap, const ObjectMap& objectMap)
{
  // This method allows us to avoid dynamic allocation (cloning) whenever possible.
  for(const auto& object : objectMap)
  {
    if(nullptr == object.second)
      continue;

    const DataType* data = (object.second.get()->*getData)();
    if(data)
    {
      // Attempt to insert a nullptr to see whether this data exists while also
      // creating an iterator to it if it did not already exist. This allows us
      // to search for a spot in the data map once, instead of searching the map
      // to see if the data entry already exists and then searching the map
      // again in order to insert the entry if it didn't already exist.
      std::pair<typename DataMap::iterator, bool> insertion =
          dataMap.insert(typename DataMap::value_type(object.first, nullptr));

      typename DataMap::iterator& it = insertion.first;
      const bool existed = !insertion.second;

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
/// Type maps are std::map containers which map an object's Type Info to some
/// instance or trait of that type. For example, an ObjectMap will map an
/// object's type to a std::unique_ptr of an instance of that object. a StateMap
/// will map an Object's type to a std::unique_ptr of a State instance for that
/// Object. Type maps are used for the dart::common::Aspect class.
///
/// This function will take a type map of Data and pass its contents into the
/// Objects contained in an ObjectMap for each corresponding Object type which
/// is available.
template <typename ObjectType, class DataType,
          void (ObjectType::*setData)(const DataType&),
          typename ObjectMap = std::map< std::type_index, std::unique_ptr<ObjectType> >,
          typename DataMap = std::map< std::type_index, std::unique_ptr<DataType> > >
static void setObjectsFromDataTypeMap(
    ObjectMap& objectMap, const DataMap& dataMap)
{
  typename ObjectMap::iterator objects = objectMap.begin();
  typename DataMap::const_iterator data = dataMap.begin();

  while( objectMap.end() != objects && dataMap.end() != data )
  {
    if( objects->first == data->first )
    {
      ObjectType* object = objects->second.get();
      if(object && data->second)
        (object->*setData)(*data->second);

      ++objects;
      ++data;
    }
    else if( objects->first < data->first )
    {
      ++objects;
    }
    else
    {
      ++data;
    }
  }
}

//==============================================================================
void Composite::setCompositeState(const State& newStates)
{
  setObjectsFromDataTypeMap<Aspect, Aspect::State, &Aspect::setAspectState>(
        mAspectMap, newStates.getMap());
}

//==============================================================================
Composite::State Composite::getCompositeState() const
{
  State states;
  copyCompositeStateTo(states);

  return states;
}

//==============================================================================
void Composite::copyCompositeStateTo(State& outgoingStates) const
{
  auto& states = outgoingStates.getMap();
  extractDataFromObjectTypeMap<Aspect, Aspect::State, &Aspect::getAspectState>(
        states, mAspectMap);
}

//==============================================================================
void Composite::setCompositeProperties(const Properties& newProperties)
{
  setObjectsFromDataTypeMap<
      Aspect, Aspect::Properties, &Aspect::setAspectProperties>(
        mAspectMap, newProperties.getMap());
}

//==============================================================================
Composite::Properties Composite::getCompositeProperties() const
{
  Properties properties;
  copyCompositePropertiesTo(properties);

  return properties;
}

//==============================================================================
void Composite::copyCompositePropertiesTo(
    Properties& outgoingProperties) const
{
  auto& properties = outgoingProperties.getMap();
  extractDataFromObjectTypeMap<Aspect, Aspect::Properties, &Aspect::getAspectProperties>(
        properties, mAspectMap);
}

//==============================================================================
void Composite::duplicateAspects(const Composite* fromComposite)
{
  if(nullptr == fromComposite)
  {
    dterr << "[Composite::duplicateAspects] You have asked to duplicate the "
          << "Aspects of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  if(this == fromComposite)
    return;

  const AspectMap& otherMap = fromComposite->mAspectMap;

  AspectMap::iterator receiving = mAspectMap.begin();
  AspectMap::const_iterator incoming = otherMap.begin();

  while( otherMap.end() != incoming )
  {
    if( mAspectMap.end() == receiving )
    {
      // If we've reached the end of this Composite's AspectMap, then we should
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
      // If this Composite does not have an entry corresponding to the incoming
      // Aspect, then we must create it
      _set(incoming->first, incoming->second.get());
      ++incoming;
    }
  }
}

//==============================================================================
void Composite::matchAspects(const Composite* otherComposite)
{
  if(nullptr == otherComposite)
  {
    dterr << "[Composite::matchAspects] You have asked to match the Aspects "
          << "of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  for(auto& aspect : mAspectMap)
    aspect.second = nullptr;

  duplicateAspects(otherComposite);
}

//==============================================================================
void Composite::addToComposite(Aspect* aspect)
{
  if(aspect)
    aspect->setComposite(this);
}

//==============================================================================
void Composite::removeFromComposite(Aspect* aspect)
{
  if(aspect)
    aspect->loseComposite(this);
}

//==============================================================================
void Composite::_set(std::type_index type_idx, const Aspect* aspect)
{
  if(aspect)
  {
    mAspectMap[type_idx] = aspect->cloneAspect();
    addToComposite(mAspectMap[type_idx].get());
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
  addToComposite(mAspectMap[type_idx].get());
}

} // namespace common
} // namespace dart
