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

#ifndef DART_COMMON_DETAIL_ADDONMANAGER_H_
#define DART_COMMON_DETAIL_ADDONMANAGER_H_

#include "dart/common/AddonManager.h"

namespace dart {
namespace common {

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>::MapHolder(const MapHolder& otherHolder)
{
  *this = otherHolder;
}

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>::MapHolder(MapHolder&& otherHolder)
{
  *this = std::move(otherHolder);
}

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>::MapHolder(const MapType& otherMap)
{
  *this = otherMap;
}

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>::MapHolder(MapType&& otherMap)
{
  *this = std::move(otherMap);
}

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>& AddonManager::MapHolder<MapType>::operator=(
    const MapHolder& otherHolder)
{
  *this = otherHolder.getMap();

  return *this;
}

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>& AddonManager::MapHolder<MapType>::operator=(
    MapHolder&& otherHolder)
{
  mMap = std::move(otherHolder.mMap);

  return *this;
}

//==============================================================================
template <typename MapType>
AddonManager::MapHolder<MapType>& AddonManager::MapHolder<MapType>::operator=(
    const MapType& otherMap)
{
  typename MapType::iterator receiver = mMap.begin();
  typename MapType::const_iterator sender = otherMap.begin();

  while( otherMap.end() != sender )
  {
    if( mMap.end() == receiver )
    {
      // If we've reached the end of this MapHolder's map, then we should just
      // add each entry
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
AddonManager::MapHolder<MapType>& AddonManager::MapHolder<MapType>::operator=(
    MapType&& otherHolder)
{
  mMap = std::move(otherHolder);

  return *this;
}

//==============================================================================
template <typename MapType>
const MapType& AddonManager::MapHolder<MapType>::getMap() const
{
  return mMap;
}

//==============================================================================
template <class T>
bool AddonManager::has() const
{
  return (get<T>() != nullptr);
}

//==============================================================================
template <class T>
T* AddonManager::get()
{
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  if(mAddonMap.end() == it)
    return nullptr;

  return static_cast<T*>(it->second.get());
}

//==============================================================================
template <class T>
const T* AddonManager::get() const
{
  return const_cast<AddonManager*>(this)->get<T>();
}

//==============================================================================
template <class T>
void AddonManager::set(const T* addon)
{
  mAddonMap[typeid(T)] = addon->clone(this);
}

//==============================================================================
template <class T>
void AddonManager::set(std::unique_ptr<T>&& addon)
{
  becomeManager(addon.get());
  mAddonMap[typeid(T)] = std::move(addon);
}

//==============================================================================
template <class T, typename ...Args>
T* AddonManager::construct(Args&&... args)
{
  T* addon = new T(this, std::forward(args)...);
  mAddonMap[typeid(T)] = std::unique_ptr<T>(addon);

  return addon;
}

//==============================================================================
template <class T>
void AddonManager::erase()
{
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  if(mAddonMap.end() != it)
    it->second = nullptr;
}

//==============================================================================
template <class T>
std::unique_ptr<T> AddonManager::release()
{
  std::unique_ptr<T> extraction = nullptr;
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  if(mAddonMap.end() != it)
    extraction = std::unique_ptr<T>(static_cast<T*>(it->second.release()));

  return extraction;
}

} // namespace common
} // namespace dart

//==============================================================================
#define DART_ENABLE_ADDON_SPECIALIZATION()                                                            \
  public:                                                                                             \
  template <class T> bool has() const { return AddonManager::has<T>(); }                              \
  template <class T> T* get() { return AddonManager::get<T>(); }                                      \
  template <class T> const T* get() const { return AddonManager::get<T>(); }                          \
  template <class T> void set(const T* addon) { AddonManager::set<T>(addon); }                        \
  template <class T> void set(std::unique_ptr<T>&& addon) { AddonManager::set<T>(std::move(addon)); } \
  template <class T> void erase() { AddonManager::erase<T>(); }                                       \
  template <class T> std::unique_ptr<T> release() { return AddonManager::release<T>(); }

//==============================================================================
#define DETAIL_DART_INSTANTIATE_SPECIALIZED_ADDON( AddonName, it )              \
  mAddonMap[typeid( AddonName )] = nullptr;                                     \
  it = mAddonMap.find(typeid( AddonName ));

//==============================================================================
#define DART_INSTANTIATE_SPECIALIZED_ADDON( AddonName )                               \
  DETAIL_DART_INSTANTIATE_SPECIALIZED_ADDON( AddonName, m ## AddonName ## Iterator );

//==============================================================================
#define DETAIL_DART_SPECIALIZE_ADDON_INTERNAL( AddonName, it )                  \
  private: AddonMap::iterator it ; public:                                      \
                                                                                \
  inline bool has ## AddonName () const                                         \
  { return (get ## AddonName () != nullptr); }                                  \
                                                                                \
  inline AddonName * get ## AddonName ()                                        \
  { return static_cast< AddonName *>( it ->second.get() ); }                    \
                                                                                \
  inline const AddonName* get ## AddonName () const                             \
  { return static_cast< AddonName *>( it ->second.get() ); }                    \
                                                                                \
  inline void set ## AddonName (const AddonName * addon)                        \
  { it ->second = addon->clone(this); }                                         \
                                                                                \
  inline void set ## AddonName (std::unique_ptr< AddonName >&& addon)           \
  { becomeManager(addon.get()); it ->second = std::move(addon); }               \
                                                                                \
  template <typename ...Args>                                                   \
  inline AddonName * construct ## AddonName (Args&&... args)                    \
  { it ->second = std::unique_ptr< AddonName >(                                 \
          new AddonName (this, std::forward(args)...));                         \
    return static_cast< AddonName *>( it ->second.get() ); }                    \
                                                                                \
  inline void erase ## AddonName ()                                             \
  { it ->second = nullptr; }                                                    \
                                                                                \
  inline std::unique_ptr< AddonName > release ## AddonName ()                   \
  { std::unique_ptr< AddonName > extraction = std::unique_ptr< AddonName >(     \
          static_cast< AddonName *>(it ->second.release()));                    \
    it ->second = nullptr; return extraction; }

//==============================================================================
#define DART_SPECIALIZE_ADDON_INTERNAL( AddonName )                             \
  DETAIL_DART_SPECIALIZE_ADDON_INTERNAL( AddonName, m ## AddonName ## Iterator )

//==============================================================================
#define DART_SPECIALIZE_ADDON_EXTERNAL( Manager, Addon )                                                          \
  template <> bool Manager :: has< Addon >() const { return has ## Addon (); }                                    \
  template <> Addon * Manager :: get< Addon >() { return get ## Addon (); }                                       \
  template <> const Addon * Manager :: get< Addon >() const { return get ## Addon (); }                           \
  template <> void Manager :: set< Addon >(const Addon * addon) { set ## Addon (addon); }                         \
  template <> void Manager :: set< Addon >(std::unique_ptr< Addon >&& addon) { set ## Addon (std::move(addon)); } \
  template <> void Manager :: erase< Addon >() { erase ## Addon (); }                                             \
  template <> std::unique_ptr< Addon > Manager :: release< Addon >() { return release ## Addon (); }

#endif // DART_COMMON_DETAIL_ADDONMANAGER_H_
