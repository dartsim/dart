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

#ifndef DART_COMMON_SPECIALIZEDFORADDON_H_
#define DART_COMMON_SPECIALIZEDFORADDON_H_

#include "dart/common/AddonManager.h"
#include "dart/common/AddonManagerJoiner.h"
#include "dart/common/Virtual.h"

namespace dart {
namespace common {

/// Declaration of the variadic template
template <class... OtherSpecAddons>
class SpecializedForAddon { };

//==============================================================================
/// SpecializedForAddon allows classes that inherit AddonManager to have
/// constant-time access to a specific type of Addon
template <class SpecAddon>
class SpecializedForAddon<SpecAddon> : public virtual AddonManager
{
public:

  /// Default Constructor
  SpecializedForAddon();

  /// Check if this AddonManager currently has a certain type of Addon
  template <class T>
  bool has() const;

  /// Get a certain type of Addon from this AddonManager
  template <class T>
  T* get();

  /// Get a certain type of Addon from this AddonManager
  template <class T>
  const T* get() const;

  /// Make a clone of the addon and place the clone into this AddonManager. If
  /// an Addon of the same type already exists in this AddonManager, the
  /// existing Addon will be destroyed.
  template <class T>
  void set(const T* addon);

  /// Use move semantics to place an addon into this AddonManager. If an Addon
  /// of the same type already exists in this AddonManager, the existing Addon
  /// will be destroyed.
  template <class T>
  void set(std::unique_ptr<T>&& addon);

  /// Construct an Addon inside of this AddonManager
  template <class T, typename ...Args>
  T* create(Args&&... args);

  /// Remove an Addon from this AddonManager
  template <class T>
  void erase();

  /// Remove an Addon from this AddonManager, but return its unique_ptr instead
  /// of letting it be deleted. This allows you to safely use move semantics to
  /// transfer an Addon between two AddonManagers.
  template <class T>
  std::unique_ptr<T> release();

  /// Check if this Manager is specialized for a specific type of Addon
  template <class T>
  static constexpr bool isSpecializedFor();

protected:

  /// Redirect to AddonManager::has()
  template <class T>
  bool _has(type<T>) const;

  /// Specialized implementation of has()
  bool _has(type<SpecAddon>) const;

  /// Redirect to AddonManager::get()
  template <class T>
  T* _get(type<T>);

  /// Specialized implementation of get()
  SpecAddon* _get(type<SpecAddon>);

  /// Redirect to AddonManager::get()
  template <class T>
  const T* _get(type<T>) const;

  /// Specialized implementation of get()
  const SpecAddon* _get(type<SpecAddon>) const;

  /// Redirect to AddonManager::set()
  ///
  /// Using the type<T> tag for this is not be necessary, but it helps to avoid
  /// confusion between the set() versus _set() function.
  template <class T>
  void _set(type<T>, const T* addon);

  /// Specialized implementation of set()
  void _set(type<SpecAddon>, const SpecAddon* addon);

  /// Redirect to AddonManager::set()
  ///
  /// Using the type<T> tag for this is not be necessary, but it helps to avoid
  /// confusion between the set() versus _set() function.
  template <class T>
  void _set(type<T>, std::unique_ptr<T>&& addon);

  /// Specialized implementation of set()
  void _set(type<SpecAddon>, std::unique_ptr<SpecAddon>&& addon);

  /// Redirect to AddonManager::create()
  template <class T, typename ...Args>
  T* _create(type<T>, Args&&... args);

  /// Specialized implementation of create()
  template <typename ...Args>
  SpecAddon* _create(type<SpecAddon>, Args&&... args);

  /// Redirect to AddonManager::erase()
  template <class T>
  void _erase(type<T>);

  /// Specialized implementation of erase()
  void _erase(type<SpecAddon>);

  /// Redirect to AddonManager::release()
  template <class T>
  std::unique_ptr<T> _release(type<T>);

  /// Specialized implementation of release()
  std::unique_ptr<SpecAddon> _release(type<SpecAddon>);

  /// Return false
  template <class T>
  static constexpr bool _isSpecializedFor(type<T>);

  /// Return true
  static constexpr bool _isSpecializedFor(type<SpecAddon>);

  /// Iterator that points to the Addon of this SpecializedForAddon
  AddonManager::AddonMap::iterator mSpecAddonIterator;

};

//==============================================================================
/// This is the variadic version of the SpecializedForAddon class which
/// allows you to include arbitrarily many specialized types in the
/// specialization.
template <class SpecAddon1, class... OtherSpecAddons>
class SpecializedForAddon<SpecAddon1, OtherSpecAddons...> :
    public AddonManagerJoiner< Virtual< SpecializedForAddon<SpecAddon1> >,
                               Virtual< SpecializedForAddon<OtherSpecAddons...> > > { };

} // namespace common
} // namespace dart

#include "dart/common/detail/SpecializedForAddon.h"

#endif // DART_COMMON_SPECIALIZEDFORADDON_H_
