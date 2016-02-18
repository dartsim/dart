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

#ifndef DART_COMMON_ADDONMANAGER_H_
#define DART_COMMON_ADDONMANAGER_H_

#include <map>
#include <typeinfo>
#include <typeindex>

#include "dart/common/Addon.h"

namespace dart {
namespace common {

/// AddonManager is a base class that should be virtually inherited by any class
/// that wants to be able to manage Addons.
///
/// The base AddonManager class is completely agnostic to what kind of Addons it
/// is given. Addons are stored in a std::map, so access to its Addons happens
/// on average in log(N) time. Most often, a class that accepts Addons will have
/// certain Addon types that it will need to access frequently, and it would be
/// beneficial to have constant-time access to those Addon types. To get
/// constant-time access to specific Addon types, there are FOUR macros that you
/// should use in your derived class:
///
/// DART_ENABLE_ADDON_SPECIALIZATION() must be declared once in the derived
/// class's definition, under a 'public:' declaration range.
///
/// DART_SPECIALIZE_ADDON_INTERNAL( AddonType ) must be declared once for each
/// AddonType that you want to specialize. It should be placed in the derived
/// class's definition, under a 'public:' declaration range.
///
/// DART_SPECIALIZE_ADDON_EXTERNAL( Derived, AddonType ) must be declared once
/// for each AddonType that you want to specialize. It should be placed
/// immediately after the class's definition in the same header file as the
/// derived class, inside of the same namespace as the derived class. This macro
/// defines a series of templated functions, so it should go in a header, and
/// not in a source file.
///
/// DART_INSTANTIATE_SPECIALIZED_ADDON( AddonType ) must be declared once for
/// each AddonType that you want to specialize. It should be placed inside the
/// constructor of the derived class, preferably before anything else is done
/// inside the body of the constructor.
class AddonManager
{
public:

  using StateMap = std::map< std::type_index, std::unique_ptr<Addon::State> >;
  using State = ExtensibleMapHolder<StateMap>;

  using PropertiesMap = std::map< std::type_index, std::unique_ptr<Addon::Properties> >;
  using Properties = ExtensibleMapHolder<PropertiesMap>;

  using AddonMap = std::map< std::type_index, std::unique_ptr<Addon> >;

  /// Virtual destructor
  virtual ~AddonManager() = default;

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

  /// Use move semantics to place addon into this AddonManager. If an Addon of
  /// the same type already exists in this AddonManager, the existing Addon will
  /// be destroyed.
  template <class T>
  void set(std::unique_ptr<T>&& addon);

  /// Construct an Addon inside of this AddonManager
  template <class T, typename ...Args>
  T* create(Args&&... args);

  /// Remove an Addon from this AddonManager.
  template <class T>
  void erase();

  /// Remove an Addon from this AddonManager, but return its unique_ptr instead
  /// of letting it be deleted. This allows you to safely use move semantics to
  /// transfer an Addon between two AddonManagers.
  template <class T>
  std::unique_ptr<T> release();

  /// Check if this Manager is specialized for a specific type of Addon.
  ///
  /// By default, this simply returns false.
  template <class T>
  static constexpr bool isSpecializedFor();

  /// Set the states of the addons in this AddonManager based on the given
  /// AddonManager::State. The states of any Addon types that do not exist
  /// within this manager will be ignored.
  void setAddonStates(const State& newStates);

  /// Get the states of the addons inside of this AddonManager
  State getAddonStates() const;

  /// Fill outgoingStates with the states of the addons inside this AddonManager
  void copyAddonStatesTo(State& outgoingStates) const;

  /// Set the properties of the addons in this AddonManager based on the given
  /// AddonManager::Properties. The properties of any Addon types that do not
  /// exist within this manager will be ignored.
  void setAddonProperties(const Properties& newProperties);

  /// Get the properties of the addons inside of this AddonManager
  Properties getAddonProperties() const;

  /// Fill outgoingProperties with the properties of the addons inside this
  /// AddonManager
  void copyAddonPropertiesTo(Properties& outgoingProperties) const;

  /// Give this AddonManager a copy of each Addon from otherManager
  void duplicateAddons(const AddonManager* otherManager);

  /// Make the Addons of this AddonManager match the Addons of otherManager. Any
  /// Addons in this AddonManager which do not exist in otherManager will be
  /// erased.
  void matchAddons(const AddonManager* otherManager);

protected:

  template <class T> struct type { };

  /// Become the AddonManager of the given Addon. This allows derived
  /// AddonManager types to call the protected Addon::setManager function.
  void becomeManager(Addon* addon, bool transfer);

  /// A map that relates the type of Addon to its pointer
  AddonMap mAddonMap;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/AddonManager.h"

#endif // DART_COMMON_ADDONMANAGER_H_
