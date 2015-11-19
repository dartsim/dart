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

#define DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE( Func, it, ReturnType )\
  if(it ->second && !it ->second->isOptional(this))\
  {\
    dterr << "[AddonManager::" #Func << "] Illegal request to remove Addon!\n";\
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
  if(addon)
  {
    mAddonMap[typeid(T)] = addon->cloneAddon(this);
    becomeManager(mAddonMap[typeid(T)].get(), false);
  }
  else
  {
    mAddonMap[typeid(T)] = nullptr;
  }
}

//==============================================================================
template <class T>
void AddonManager::set(std::unique_ptr<T>&& addon)
{
  mAddonMap[typeid(T)] = std::move(addon);
  becomeManager(mAddonMap[typeid(T)].get(), true);
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
  DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(erase, it, DART_BLANK)
  if(mAddonMap.end() != it)
    it->second = nullptr;
}

//==============================================================================
template <class T>
std::unique_ptr<T> AddonManager::release()
{
  std::unique_ptr<T> extraction = nullptr;
  AddonMap::iterator it = mAddonMap.find( typeid(T) );
  DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(release, it, nullptr)
  if(mAddonMap.end() != it)
    extraction = std::unique_ptr<T>(static_cast<T*>(it->second.release()));

  return extraction;
}

//==============================================================================
template <class T>
constexpr bool AddonManager::isSpecializedFor()
{
  return _isSpecializedFor(type<T>());
}

//==============================================================================
template <class T>
constexpr bool AddonManager::_isSpecializedFor(type<T>)
{
  return false;
}

} // namespace common
} // namespace dart

//==============================================================================
#define DART_ENABLE_ADDON_SPECIALIZATION()\
  public:\
  template <class T> bool has() const { return AddonManager::has<T>(); }\
  template <class T> T* get() { return AddonManager::get<T>(); }\
  template <class T> const T* get() const { return AddonManager::get<T>(); }\
  template <class T> void set(const T* addon) { AddonManager::set<T>(addon); }\
  template <class T> void set(std::unique_ptr<T>&& addon) { AddonManager::set<T>(std::move(addon)); }\
  template <class T> void erase() { AddonManager::erase<T>(); }\
  template <class T> std::unique_ptr<T> release() { return AddonManager::release<T>(); }\
  template <class T, typename ...Args> T* create(Args&&... args)\
  {\
    T* addon = new T(this, std::forward<Args>(args)...);\
    mAddonMap[typeid(T)] = std::unique_ptr<T>(addon); return addon;\
  }

//==============================================================================
#define DETAIL_DART_SPECIALIZED_ADDON_INSTANTIATE_IMPLEMENTATION( TypeName, it )              \
  mAddonMap[typeid( TypeName )] = nullptr;                                     \
  it = mAddonMap.find(typeid( TypeName ));

//==============================================================================
#define DART_IRREGULAR_SPECIALIZED_ADDON_INSTANTIATE( TypeName, HomogenizedName )\
  DETAIL_DART_SPECIALIZED_ADDON_INSTANTIATE_IMPLEMENTATION( TypeName, m ## HomogenizedName ## Iterator )

//==============================================================================
#define DART_SPECIALIZED_ADDON_INSTANTIATE( AddonName )                               \
  DART_IRREGULAR_SPECIALIZED_ADDON_INSTANTIATE( AddonName, AddonName );

//==============================================================================
#define DART_NESTED_SPECIALIZED_ADDON_INSTANTIATE( ParentName, AddonName )      \
  DART_IRREGULAR_SPECIALIZED_ADDON_INSTANTIATE( ParentName :: AddonName, ParentName ## AddonName )

//==============================================================================
#define DETAIL_DART_SPECIALIZED_ADDON_INLINE_IMPLEMENTATION( TypeName, AddonName, it, CreationCallback )\
  private: AddonMap::iterator it ; public:\
  \
  inline bool has ## AddonName () const\
  { return (get ## AddonName () != nullptr);\
  }\
  inline TypeName * get ## AddonName ()\
  { return static_cast< TypeName *>( it ->second.get() );\
  }\
  inline const TypeName* get ## AddonName () const\
  { return static_cast< TypeName *>( it ->second.get() );\
  }\
  inline void set ## AddonName (const TypeName * addon)\
  { it ->second = addon->cloneAddon(this); becomeManager(it ->second.get(), false);\
  }\
  inline void set ## AddonName (std::unique_ptr< TypeName >&& addon)\
  { it ->second = std::move(addon); becomeManager(it ->second.get(), true);\
  }\
  template <typename ...Args>\
  inline TypeName * create ## AddonName (Args&&... args)\
  { it ->second = std::unique_ptr< TypeName >(\
          new TypeName (this, std::forward<Args>(args)...));\
    becomeManager(it ->second.get(), false);\
    return static_cast< TypeName *>( it ->second.get() );\
  }\
  inline void erase ## AddonName ()\
  { DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(erase ## AddonName, it, DART_BLANK)\
    it ->second = nullptr;\
  }\
  inline std::unique_ptr< TypeName > release ## AddonName ()\
  { DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(release ## AddonName, it, nullptr)\
    std::unique_ptr< TypeName > extraction = std::unique_ptr< TypeName >(\
          static_cast< TypeName *>(it ->second.release()));\
    it ->second = nullptr; return extraction;\
  }

//==============================================================================
#define DETAIL_DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( TypeName, HomogenizedName, CreationCallback )\
  DETAIL_DART_SPECIALIZED_ADDON_INLINE_IMPLEMENTATION( TypeName, HomogenizedName, m ## HomogenizedName ## Iterator, CreationCallback )

//==============================================================================
#define DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( TypeName, AddonName )\
  DETAIL_DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( TypeName, AddonName, DART_BLANK )

//==============================================================================
#define DETAIL_DART_SPECIALIZED_ADDON_INLINE( AddonName, CreationCallback )\
  DETAIL_DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( AddonName, AddonName, CreationCallback )

//==============================================================================
#define DART_SPECIALIZED_ADDON_INLINE( AddonName )                              \
  DETAIL_DART_SPECIALIZED_ADDON_INLINE( AddonName, DART_BLANK )

//==============================================================================
#define DETAIL_DART_NESTED_SPECIALIZED_ADDON_INLINE( ParentName, AddonName, CreationCallback )\
  DETAIL_DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( ParentName :: AddonName, ParentName ## AddonName, CreationCallback )

//==============================================================================
#define DART_NESTED_SPECIALIZED_ADDON_INLINE( ParentName, AddonName )\
  DETAIL_DART_NESTED_SPECIALIZED_ADDON_INLINE( ParentName, AddonName, DART_BLANK )

//==============================================================================
#define DART_IRREGULAR_SPECIALIZED_ADDON_TEMPLATE( Manager, TypeName, HomogenizedName, LinePrefix )\
  LinePrefix template <> inline bool Manager :: has< TypeName >() const { return has ## HomogenizedName (); }\
  LinePrefix template <> inline TypeName * Manager :: get< TypeName >() { return get ## HomogenizedName (); }\
  LinePrefix template <> inline const TypeName * Manager :: get< TypeName >() const { return get ## HomogenizedName (); }\
  LinePrefix template <> inline void Manager :: set< TypeName >(const TypeName * addon) { set ## HomogenizedName (addon); }\
  LinePrefix template <> inline void Manager :: set< TypeName >(std::unique_ptr< TypeName >&& addon) { set ## HomogenizedName (std::move(addon)); }\
  LinePrefix template <> inline void Manager :: erase< TypeName >() { erase ## HomogenizedName (); }\
  LinePrefix template <> inline std::unique_ptr< TypeName > Manager :: release< TypeName >() { return release ## HomogenizedName (); }

//==============================================================================
#define DART_SPECIALIZED_ADDON_TEMPLATE( Manager, AddonName )\
  DART_IRREGULAR_SPECIALIZED_ADDON_TEMPLATE( Manager, AddonName, AddonName, DART_BLANK )

//==============================================================================
#define DART_NESTED_SPECIALIZED_ADDON_TEMPLATE( Manager, ParentName, AddonName )\
  DART_IRREGULAR_SPECIALIZED_ADDON_TEMPLATE( Manager, ParentName :: AddonName, ParentName ## AddonName, DART_BLANK )

#endif // DART_COMMON_DETAIL_ADDONMANAGER_H_
