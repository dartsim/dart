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

#ifndef DART_COMMON_DETAIL_EXTENSIBLE_H_
#define DART_COMMON_DETAIL_EXTENSIBLE_H_

#include "dart/common/Extensible.hpp"
#include "dart/common/StlHelpers.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>::ExtensibleMixer()
{
  // Do nothing
}

//==============================================================================
template <class T, class Mixin>
template <typename ... Args>
ExtensibleMixer<T, Mixin>::ExtensibleMixer(Args&&... args)
  : Mixin(std::forward<Args>(args)...)
{
  // Do nothing
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>::ExtensibleMixer(const Mixin& mixin)
  : Mixin(mixin)
{
  // Do nothing
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>::ExtensibleMixer(Mixin&& mixin)
  : Mixin(std::move(mixin))
{
  // Do nothing
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>::ExtensibleMixer(
    const ExtensibleMixer<T, Mixin>& other)
  : Mixin(other)
{
  // Do nothing
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>::ExtensibleMixer(ExtensibleMixer<T, Mixin>&& other)
  : Mixin(other)
{
  // Do nothing
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>& ExtensibleMixer<T, Mixin>::operator=(
    const Mixin& mixin)
{
  static_cast<Mixin&>(*this) = mixin;
  return *this;
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>& ExtensibleMixer<T, Mixin>::operator=(Mixin&& mixin)
{
  static_cast<Mixin&>(*this) = std::move(mixin);
  return *this;
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>& ExtensibleMixer<T, Mixin>::operator=(
    const ExtensibleMixer& other)
{
  static_cast<Mixin&>(*this) = static_cast<const Mixin&>(other);
  return *this;
}

//==============================================================================
template <class T, class Mixin>
ExtensibleMixer<T, Mixin>& ExtensibleMixer<T, Mixin>::operator=(
    ExtensibleMixer&& other)
{
  static_cast<Mixin&>(*this) = std::move(static_cast<Mixin&&>(other));
}

//==============================================================================
template <class T, class Mixin>
std::unique_ptr<T> ExtensibleMixer<T, Mixin>::clone() const
{
  return common::make_unique<ExtensibleMixer<T, Mixin>>(*this);
}

//==============================================================================
template <class T, class Mixin>
void ExtensibleMixer<T, Mixin>::copy(const T& other)
{
  *this = static_cast<const ExtensibleMixer<T, Mixin>&>(other);
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>::ExtensibleMapHolder(
    const ExtensibleMapHolder& otherHolder)
{
  *this = otherHolder;
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>::ExtensibleMapHolder(
    ExtensibleMapHolder&& otherHolder)
{
  *this = std::move(otherHolder);
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>::ExtensibleMapHolder(const MapType& otherMap)
{
  *this = otherMap;
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>::ExtensibleMapHolder(MapType&& otherMap)
{
  *this = std::move(otherMap);
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>& ExtensibleMapHolder<MapType>::operator=(
    const ExtensibleMapHolder& otherHolder)
{
  *this = otherHolder.getMap();

  return *this;
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>& ExtensibleMapHolder<MapType>::operator=(
    ExtensibleMapHolder&& otherHolder)
{
  mMap = std::move(otherHolder.mMap);

  return *this;
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>& ExtensibleMapHolder<MapType>::operator=(
    const MapType& otherMap)
{
  typename MapType::iterator receiver = mMap.begin();
  typename MapType::const_iterator sender = otherMap.begin();

  while( otherMap.end() != sender )
  {
    if( mMap.end() == receiver )
    {
      // If we've reached the end of this ExtensibleMapHolder's map, then we
      // should just add each entry
      mMap[sender->first] = sender->second->clone();
      ++sender;
    }
    else if( receiver->first == sender->first )
    {
      // We should copy the incoming object when possible so we can avoid the
      // memory allocation overhead of cloning.
      if(receiver->second)
        receiver->second->copy(*sender->second);
      else
        receiver->second = sender->second->clone();

      ++receiver;
      ++sender;
    }
    else if( receiver->first < sender->first )
    {
      // Clear this entry in the map, because it does not have an analog in the
      // map that we are copying
      receiver->second = nullptr;
      ++receiver;
    }
    else
    {
      mMap[sender->first] = sender->second->clone();
      ++sender;
    }
  }

  while( mMap.end() != receiver )
  {
    mMap.erase(receiver++);
  }

  return *this;
}

//==============================================================================
template <typename MapType>
ExtensibleMapHolder<MapType>& ExtensibleMapHolder<MapType>::operator=(
    MapType&& otherHolder)
{
  mMap = std::move(otherHolder);

  return *this;
}

//==============================================================================
template <typename MapType>
MapType& ExtensibleMapHolder<MapType>::getMap()
{
  return mMap;
}

//==============================================================================
template <typename MapType>
const MapType& ExtensibleMapHolder<MapType>::getMap() const
{
  return mMap;
}

//==============================================================================
template <typename T>
ExtensibleVector<T>::ExtensibleVector(const std::vector<T>& regularVector)
  : mVector(regularVector)
{
  // Do nothing
}

//==============================================================================
template <typename T>
ExtensibleVector<T>::ExtensibleVector(std::vector<T>&& regularVector)
{
  mVector = std::move(regularVector);
}

//==============================================================================
template <typename T>
std::unique_ptr< ExtensibleVector<T> > ExtensibleVector<T>::clone() const
{
  std::vector<T> clonedVector;
  clonedVector.reserve(mVector.size());

  for(const T& entry : mVector)
    clonedVector.push_back(entry->clone());

  return common::make_unique< ExtensibleVector<T> >( std::move(clonedVector) );
}

//==============================================================================
template <typename T>
void ExtensibleVector<T>::copy(const ExtensibleVector<T>& anotherVector)
{
  const std::vector<T>& other = anotherVector.getVector();
  mVector.resize(other.size());

  for(size_t i=0; i < other.size(); ++i)
  {
    if(mVector[i] && other[i])
      mVector[i]->copy(*other[i]);
    else if(other[i])
      mVector[i] = other[i]->clone();
    else
      mVector[i] = nullptr;
  }
}

//==============================================================================
template <typename T>
const std::vector<T>& ExtensibleVector<T>::getVector() const
{
  return mVector;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_EXTENSIBLE_H_
