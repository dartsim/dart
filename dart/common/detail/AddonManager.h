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

#define DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE( Func, T, ReturnType )\
  if(requires< T >())\
  {\
    dterr << "[AddonManager::" #Func << "] Illegal request to remove required "\
          << "Addon [" << typeid(T).name() << "]!\n";\
    assert(false);\
    return ReturnType ;\
  }

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
  _set(typeid(T), addon);
}

//==============================================================================
template <class T>
void AddonManager::set(std::unique_ptr<T>&& addon)
{
  _set(typeid(T), std::move(addon));
}

//==============================================================================
template <class T, typename ...Args>
T* AddonManager::create(Args&&... args)
{
  T* addon = new T(this, std::forward<Args>(args)...);
  mAddonMap[typeid(T)] = std::unique_ptr<T>(addon);
  becomeManager(addon, false);

  return addon;
}

//==============================================================================
template <class T>
void AddonManager::erase()
{
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(erase, T, DART_BLANK)
  if(mAddonMap.end() != it)
    it->second = nullptr;
}

//==============================================================================
template <class T>
std::unique_ptr<T> AddonManager::release()
{
  std::unique_ptr<T> extraction = nullptr;
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(release, T, nullptr)
  if(mAddonMap.end() != it)
    extraction = std::unique_ptr<T>(static_cast<T*>(it->second.release()));

  return extraction;
}

//==============================================================================
template <class T>
constexpr bool AddonManager::isSpecializedFor()
{
  return false;
}

//==============================================================================
template <class T>
bool AddonManager::requires() const
{
  return (mRequiredAddons.find(typeid(T)) != mRequiredAddons.end());
}

//==============================================================================
template <class T>
void createAddons(T* /*mgr*/)
{
  // Do nothing
}

//==============================================================================
template <class T, class NextAddon, class... Addons>
void createAddons(T* mgr)
{
  mgr->template create<NextAddon>();

  createAddons<T, Addons...>(mgr);
}

} // namespace common
} // namespace dart

//==============================================================================
// Create non-template alternatives to AddonManager functions
#define DART_BAKE_SPECIALIZED_ADDON_IRREGULAR( TypeName, AddonName )     \
  inline bool has ## AddonName () const                                  \
  {                                                                      \
    return this->template has<TypeName>();                               \
  }                                                                      \
                                                                         \
  inline TypeName * get ## AddonName ()                                  \
  {                                                                      \
    return this->template get<TypeName>();                               \
  }                                                                      \
                                                                         \
  inline const TypeName* get ## AddonName () const                       \
  {                                                                      \
    return this->template get<TypeName>();                               \
  }                                                                      \
                                                                         \
  inline TypeName * get ## AddonName (const bool createIfNull)           \
  {                                                                      \
    TypeName* addon = get ## AddonName();                                \
                                                                         \
    if (createIfNull && nullptr == addon)                                \
      return create ## AddonName();                                      \
                                                                         \
    return addon;                                                        \
  }                                                                      \
                                                                         \
  inline void set ## AddonName (const TypeName * addon)                  \
  {                                                                      \
    this->template set<TypeName>(addon);                                 \
  }                                                                      \
                                                                         \
  inline void set ## AddonName (std::unique_ptr< TypeName >&& addon)     \
  {                                                                      \
    this->template set<TypeName>(std::move(addon));                      \
  }                                                                      \
                                                                         \
  template <typename ...Args>                                            \
  inline TypeName * create ## AddonName (Args&&... args)                 \
  {                                                                      \
    return this->template create<TypeName>(std::forward<Args>(args)...); \
  }                                                                      \
                                                                         \
  inline void erase ## AddonName ()                                      \
  {                                                                      \
    this->template erase<TypeName>();                                    \
  }                                                                      \
                                                                         \
  inline std::unique_ptr< TypeName > release ## AddonName ()             \
  {                                                                      \
    return this->template release<TypeName>();                           \
  }

//==============================================================================
#define DART_BAKE_SPECIALIZED_ADDON(AddonName)\
  DART_BAKE_SPECIALIZED_ADDON_IRREGULAR(AddonName, AddonName);

#endif // DART_COMMON_DETAIL_ADDONMANAGER_H_
