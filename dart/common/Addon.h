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

#ifndef DART_COMMON_ADDON_H_
#define DART_COMMON_ADDON_H_

#include <string>

#include "dart/common/Extensible.h"

namespace dart {
namespace common {

class AddonManager;

class Addon
{
public:

  friend class AddonManager;

  /// If your Addon has a State class, then that State class should inherit this
  /// Addon::State class. This allows us to safely serialize, store, and clone
  /// the states of arbitrary Addon extensions. If your Addon is stateless, then
  /// you do not have to worry about extending this class, because
  /// Addon::getState() will simply return a nullptr by default, which is taken
  /// to indicate that it is stateless.
  ///
  /// The distinction between the State class and the Properties class is that
  /// State will get stored in AddonManager::State whereas Properties will get
  /// stored in AddonManager::Properties. Typically Properties are values that
  /// only change rarely if ever, whereas State contains values that might
  /// change as often as every time step.
  ///
  /// If your Addon has a State, be sure to call setStatePtr() during
  /// the construction of your Addon.
  class State : public Extensible<State> { };

  /// Use the StateMixer class to easily create a State extension from an
  /// existing class or struct.
  template <class Mixin>
  using StateMixer = ExtensibleMixer<State, Mixin>;

  /// If your Addon has a Properties class, then it should inherit this
  /// Addon::Properties class. This allows us to safely serialize, store, and
  /// clone the properties of arbitrary Addon extensions. If your Addon has no
  /// properties, then you do not have to worry about extending this class,
  /// because Addon::getProperties() will simply return a nullptr by default,
  /// which is taken to indicate that it has no properties.
  ///
  /// The distinction between the State class and the Properties class is that
  /// State will get stored in AddonManager::State whereas Properties will get
  /// stored in AddonManager::Properties. Typically Properties are values that
  /// only change rarely if ever, whereas State contains values that might
  /// change as often as every time step.
  ///
  /// If your Addon has Properties, be sure to call setPropertiesPtr()
  /// during the construction of your Addon.
  class Properties : public Extensible<Properties> { };

  /// Use the PropertiesMixer class to easily create a Properties extension
  /// from an existing class or struct.
  template <class Mixin>
  using PropertiesMixer = ExtensibleMixer<Properties, Mixin>;

  /// Virtual destructor
  virtual ~Addon() = default;

  /// Clone this Addon into a new manager
  virtual std::unique_ptr<Addon> clone(AddonManager* newManager) const = 0;

  /// Set the State of this Addon. By default, this does nothing.
  virtual void setState(const std::unique_ptr<State>& otherState);

  /// Get the State of this Addon. By default, this returns a nullptr which
  /// implies that the Addon is stateless.
  virtual const State* getState() const;

  /// Set the Properties of this Addon. By default, this does nothing.
  virtual void setProperties(const std::unique_ptr<Properties>& someProperties);

  /// Get the Properties of this Addon. By default, this returns a nullptr
  /// which implies that the Addon has no properties.
  virtual const Properties* getProperties() const;

  /// Get the type of this Addon
  const std::string& getType() const;

protected:

  /// Constructor
  ///
  // We require the AddonManager argument in this constructor to make it clear
  // to extensions that they must have an AddonManager argument in their
  // constructors.
  Addon(AddonManager* manager, const std::string& type);

  /// Set the State pointer for this Addon.
  ///
  /// This should be called during construction of your Addon, if your Addon has
  /// a State.
  void setStatePtr(State* ptr = nullptr);

  /// Set the Properties pointer for this Addon.
  ///
  /// This should be called during construction of your Addon, if your Addon has
  /// a Properties structure.
  void setPropertiesPtr(Properties* ptr = nullptr);

  /// This function should be overriden if your Addon needs to do any special
  /// handling when its AddonManager gets changed.
  virtual void changeManager(AddonManager* newManager);

  /// Type of this Addon
  std::string mType;

private:

  /// Pointer to the State object of this Addon
  State* mStatePtr;

  /// Pointer to the Properties object of this Addon
  Properties* mPropertiesPtr;
};

//==============================================================================
/// AddonWithProtectedState generates implementations of the State managing
/// functions for an Addon class.
template <class StateData>
class AddonWithProtectedState : virtual public Addon
{
  using State = Addon::StateMixer<StateData>;

  /// Constructor that can use an Addon with an identical State type
  AddonWithProtectedState(
      AddonManager* mgr, const AddonWithProtectedState<StateData>& otherAddon);

  /// Construct using a State
  AddonWithProtectedState(
      AddonManager* mgr, const StateData& state = StateData());

  AddonWithProtectedState(const AddonWithProtectedState&) = delete;

  // Documentation inherited
  void setState(const std::unique_ptr<Addon::State>& otherState) override;

protected:

  /// State of this Addon
  State mState;
};

//==============================================================================
/// AddonWithProtectedProperties generates implementations of the Property
/// managing functions for an Addon class.
//template <class PropertiesData>
//class AddonWithProtectedProperties : virtual public Addon
//{

//};

} // namespace common
} // namespace dart

#endif // DART_COMMON_ADDON_H_
