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

  return it->second;
}

//==============================================================================
template <class T>
const T* AddonManager::get() const
{
  return const_cast<AddonManager*>(this)->get<T>();
}

//==============================================================================
template <class T>
void AddonManager::set(const std::unique_ptr<T>& addon)
{
  mAddonMap[typeid(T)] = addon->clone(this);
}

//==============================================================================
template <class T>
void AddonManager::set(std::unique_ptr<T>&& addon)
{
  addon->changeManager(this);
  mAddonMap[typeid(T)] = addon;
}

//==============================================================================
template <class T, typename ...Args>
T* AddonManager::construct(Args&&... args)
{
   mAddonMap[typeid(T)] = std::unique_ptr<T>(
         new T(this, std::forward(args)...));
}

//==============================================================================
template <class T>
void AddonManager::erase()
{
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  if(mAddonMap.end() != it)
    it->second = nullptr;
}

} // namespace common
} // namespace dart


//==============================================================================
#define DART_SPECIALIZED_ADDON( AddonName, it )                                 \
  template <>                                                                   \
  bool has< AddonName >() const                                                 \
  { return (get< AddonName >() != nullptr); }                                   \
                                                                                \
  inline bool has ## AddonName () const                                         \
  { return has< AddonName >(); }                                                \
                                                                                \
  template <>                                                                   \
  AddonName * get< AddonName >()                                                \
  { return it ->second; }                                                       \
                                                                                \
  inline AddonName * get ## AddonName ()                                        \
  { return get< AddonName >(); }                                                \
                                                                                \
  template <>                                                                   \
  const AddonName * get< AddonName >() const                                    \
  { return it ->second; }                                                       \
                                                                                \
  inline const AddonName* get ## AddonName () const                             \
  { return it ->second; }                                                       \
                                                                                \
  template <>                                                                   \
  void set< AddonName >(const std::unique_ptr< AddonName >& addon)              \
  { it ->second = addon->clone(this); }                                         \
                                                                                \
  inline set ## AddonName (const std::unique_ptr< AddonName >& addon)           \
  { set< AddonName >(addon); }                                                  \
                                                                                \
  template <>                                                                   \
  void set< AddonName >(std::unique_ptr< AddonName >&& addon)                   \
  { addon->changeManager(this); it ->second = addon; }                          \
                                                                                \
  inline set ## AddonName (std::unique_ptr< AddonName >&& addon)                \
  { set< AddonName >(addon); }                                                  \
                                                                                \
  template <typename ...Args>                                                   \
  AddonName * construct<AddonName>(Args&&... args)                              \
  { it ->second = std::unique_ptr< AddonName >(                                 \
          new AddonName (this, std::forward(args)...)); }                       \
                                                                                \
  template <typename ...Args>                                                   \
  inline AddonName * construct ## AddonName (Args&&... args)                    \
  { construct< AddonName >(args); }                                             \
                                                                                \
  template <>                                                                   \
  void erase< AddonName >()                                                     \
  { it ->second = nullptr; }                                                    \
                                                                                \
  inline void erase ## AddonName ()                                             \
  { erase< AddonName >(); }

#endif // DART_COMMON_DETAIL_ADDONMANAGER_H_
