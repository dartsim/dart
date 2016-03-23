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

#ifndef DART_COMMON_DETAIL_SPECIALIZEDFORADDON_H_
#define DART_COMMON_DETAIL_SPECIALIZEDFORADDON_H_

#include "dart/common/SpecializedForAddon.h"

// This preprocessor token should only be used by the unittest that is
// responsible for checking that the specialized routines are being used to
// access specialized Addons
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
bool usedSpecializedAddonAccess;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

namespace dart {
namespace common {

//==============================================================================
template <class SpecAddon>
SpecializedForAddon<SpecAddon>::SpecializedForAddon()
{
  mAddonMap[typeid( SpecAddon )] = nullptr;
  mSpecAddonIterator = mAddonMap.find(typeid( SpecAddon ));
}

//==============================================================================
template <class SpecAddon>
template <class T>
bool SpecializedForAddon<SpecAddon>::has() const
{
  return _has(type<T>());
}

//==============================================================================
template <class SpecAddon>
template <class T>
T* SpecializedForAddon<SpecAddon>::get()
{
  return _get(type<T>());
}

//==============================================================================
template <class SpecAddon>
template <class T>
const T* SpecializedForAddon<SpecAddon>::get() const
{
  return _get(type<T>());
}

//==============================================================================
template <class SpecAddon>
template <class T>
void SpecializedForAddon<SpecAddon>::set(const T* addon)
{
  _set(type<T>(), addon);
}

//==============================================================================
template <class SpecAddon>
template <class T>
void SpecializedForAddon<SpecAddon>::set(std::unique_ptr<T>&& addon)
{
  _set(type<T>(), std::move(addon));
}

//==============================================================================
template <class SpecAddon>
template <class T, typename ...Args>
T* SpecializedForAddon<SpecAddon>::create(Args&&... args)
{
  return _create(type<T>(), std::forward<Args>(args)...);
}

//==============================================================================
template <class SpecAddon>
template <class T>
void SpecializedForAddon<SpecAddon>::erase()
{
  _erase(type<T>());
}

//==============================================================================
template <class SpecAddon>
template <class T>
std::unique_ptr<T> SpecializedForAddon<SpecAddon>::release()
{
  return _release(type<T>());
}

//==============================================================================
template <class SpecAddon>
template <class T>
constexpr bool SpecializedForAddon<SpecAddon>::isSpecializedFor()
{
  return _isSpecializedFor(type<T>());
}

//==============================================================================
template <class SpecAddon>
template <class T>
bool SpecializedForAddon<SpecAddon>::_has(type<T>) const
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  return AddonManager::has<T>();
}

//==============================================================================
template <class SpecAddon>
bool SpecializedForAddon<SpecAddon>::_has(type<SpecAddon>) const
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  return (mSpecAddonIterator->second.get() != nullptr);
}

//==============================================================================
template <class SpecAddon>
template <class T>
T* SpecializedForAddon<SpecAddon>::_get(type<T>)
{
  return AddonManager::get<T>();
}

//==============================================================================
template <class SpecAddon>
SpecAddon* SpecializedForAddon<SpecAddon>::_get(type<SpecAddon>)
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  return static_cast<SpecAddon*>(mSpecAddonIterator->second.get());
}

//==============================================================================
template <class SpecAddon>
template <class T>
const T* SpecializedForAddon<SpecAddon>::_get(type<T>) const
{
  return AddonManager::get<T>();
}

//==============================================================================
template <class SpecAddon>
const SpecAddon* SpecializedForAddon<SpecAddon>::_get(type<SpecAddon>) const
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  return static_cast<SpecAddon*>(mSpecAddonIterator->second.get());
}

//==============================================================================
template <class SpecAddon>
template <class T>
void SpecializedForAddon<SpecAddon>::_set(type<T>, const T* addon)
{
  AddonManager::set<T>(addon);
}

//==============================================================================
template <class SpecAddon>
void SpecializedForAddon<SpecAddon>::_set(
    type<SpecAddon>, const SpecAddon* addon)
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  if(addon)
  {
    mSpecAddonIterator->second = addon->cloneAddon(this);
    becomeManager(mSpecAddonIterator->second.get(), false);
  }
  else
  {
    mSpecAddonIterator->second = nullptr;
  }
}

//==============================================================================
template <class SpecAddon>
template <class T>
void SpecializedForAddon<SpecAddon>::_set(type<T>, std::unique_ptr<T>&& addon)
{
  AddonManager::set<T>(std::move(addon));
}

//==============================================================================
template <class SpecAddon>
void SpecializedForAddon<SpecAddon>::_set(
    type<SpecAddon>, std::unique_ptr<SpecAddon>&& addon)
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  mSpecAddonIterator->second = std::move(addon);
  becomeManager(mSpecAddonIterator->second.get(), true);
}

//==============================================================================
template <class SpecAddon>
template <class T, typename ...Args>
T* SpecializedForAddon<SpecAddon>::_create(type<T>, Args&&... args)
{
  return AddonManager::create<T>(std::forward<Args>(args)...);
}

//==============================================================================
template <class SpecAddon>
template <typename ...Args>
SpecAddon* SpecializedForAddon<SpecAddon>::_create(
    type<SpecAddon>, Args&&... args)
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  SpecAddon* addon = new SpecAddon(this, std::forward<Args>(args)...);
  mSpecAddonIterator->second = std::unique_ptr<SpecAddon>(addon);
  becomeManager(addon, false);

  return addon;
}

//==============================================================================
template <class SpecAddon>
template <class T>
void SpecializedForAddon<SpecAddon>::_erase(type<T>)
{
  AddonManager::erase<T>();
}

//==============================================================================
template <class SpecAddon>
void SpecializedForAddon<SpecAddon>::_erase(type<SpecAddon>)
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(erase, SpecAddon, DART_BLANK);
  mSpecAddonIterator->second = nullptr;
}

//==============================================================================
template <class SpecAddon>
template <class T>
std::unique_ptr<T> SpecializedForAddon<SpecAddon>::_release(type<T>)
{
  return AddonManager::release<T>();
}

//==============================================================================
template <class SpecAddon>
std::unique_ptr<SpecAddon> SpecializedForAddon<SpecAddon>::_release(
    type<SpecAddon>)
{
#ifdef DART_UNITTEST_SPECIALIZED_ADDON_ACCESS
  usedSpecializedAddonAccess = true;
#endif // DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

  DART_COMMON_CHECK_ILLEGAL_ADDON_ERASE(release, SpecAddon, nullptr);
  std::unique_ptr<SpecAddon> extraction(
        static_cast<SpecAddon*>(mSpecAddonIterator->second.release()));

  return extraction;
}

//==============================================================================
template <class SpecAddon>
template <class T>
constexpr bool SpecializedForAddon<SpecAddon>::_isSpecializedFor(type<T>)
{
  return false;
}

//==============================================================================
template <class SpecAddon>
constexpr bool SpecializedForAddon<SpecAddon>::_isSpecializedFor(type<SpecAddon>)
{
  return true;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_SPECIALIZEDFORADDON_H_
