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
namespace dynamics {
// Forward declare Skeleton for the sake of Skeleton-dependent Addons
class Skeleton;
} // namespace dart
} // namespace dynamics

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
  class Properties : public Extensible<Properties> { };

  /// Use the PropertiesMixer class to easily create a Properties extension
  /// from an existing class or struct.
  template <class Mixin>
  using PropertiesMixer = ExtensibleMixer<Properties, Mixin>;

  /// Virtual destructor
  virtual ~Addon() = default;

  /// Clone this Addon into a new manager
  virtual std::unique_ptr<Addon> cloneAddon(AddonManager* newManager) const = 0;

  /// Set the State of this Addon. By default, this does nothing.
  virtual void setAddonState(const std::unique_ptr<State>& otherState);

  /// Get the State of this Addon. By default, this returns a nullptr which
  /// implies that the Addon is stateless.
  virtual const State* getAddonState() const;

  /// Set the Properties of this Addon. By default, this does nothing.
  virtual void setAddonProperties(
      const std::unique_ptr<Properties>& someProperties);

  /// Get the Properties of this Addon. By default, this returns a nullptr
  /// which implies that the Addon has no properties.
  virtual const Properties* getAddonProperties() const;

  /// Get the type of this Addon
  const std::string& getAddonType() const;

protected:

  /// Constructor
  ///
  // We require the AddonManager argument in this constructor to make it clear
  // to extensions that they must have an AddonManager argument in their
  // constructors.
  Addon(AddonManager* manager, const std::string& type);

  /// This function should be overriden if your Addon needs to do any special
  /// handling when its AddonManager gets changed.
  virtual void changeManager(AddonManager* newManager);

  /// Type of this Addon
  std::string mType;
};

//==============================================================================
/// AddonWithProtectedState generates implementations of the State managing
/// functions for an Addon class.
template <typename StateData>
class AddonWithProtectedState : virtual public Addon
{
public:

  using State = Addon::StateMixer<StateData>;

  AddonWithProtectedState() = delete;
  AddonWithProtectedState(const AddonWithProtectedState&) = delete;

  /// Construct using a StateData instance
  AddonWithProtectedState(
      AddonManager* mgr, const StateData& state = StateData());

  /// Constructor that can use an Addon with an identical State type
  AddonWithProtectedState(
      AddonManager* mgr, const AddonWithProtectedState<StateData>& otherAddon);

  // Documentation inherited
  void setAddonState(
      const std::unique_ptr<Addon::State>& otherState) override final;

  // Documentation inherited
  const Addon::State* getAddonState() const override final;

protected:

  /// State of this Addon
  State mState;
};

//==============================================================================
/// AddonWithProtectedProperties generates implementations of the Property
/// managing functions for an Addon class.
template <typename PropertiesData>
class AddonWithProtectedProperties : virtual public Addon
{
public:

  using Properties = Addon::PropertiesMixer<PropertiesData>;

  AddonWithProtectedProperties() = delete;
  AddonWithProtectedProperties(const AddonWithProtectedProperties&) = delete;

  /// Construct using a PropertiesData instance
  AddonWithProtectedProperties(
      AddonManager* mgr, const PropertiesData& properties = PropertiesData());

  /// Constructor that can use an Addon with an identical Properties type
  AddonWithProtectedProperties(
      AddonManager* mgr,
      const AddonWithProtectedProperties<PropertiesData>& otherAddon);

  // Documentation inherited
  void setAddonProperties(
      const std::unique_ptr<Addon::Properties>& someProperties) override final;

  // Documentation inherited
  const Addon::Properties* getAddonProperties() const override final;

protected:

  /// Properties of this Addon
  Properties mProperties;
};

//==============================================================================
/// AddonWithProtectedPropertiesInSkeleton generates implementations of the
/// Property managing functions for an Addon class which is embedded within a
/// Skeleton or a component of a Skeleton (such as a BodyNode, Joint, or custom
/// Node class). This will increment the version count any time the
/// Addon::setProperties function is called.
template <typename PropertiesData, class ManagerType>
class AddonWithProtectedPropertiesInSkeleton : virtual public Addon
{
public:

  using Properties = Addon::PropertiesMixer<PropertiesData>;

  AddonWithProtectedPropertiesInSkeleton() = delete;
  AddonWithProtectedPropertiesInSkeleton(
      const AddonWithProtectedPropertiesInSkeleton&) = delete;

  /// Construct using a PropertiesData instance
  AddonWithProtectedPropertiesInSkeleton(
      ManagerType* mgr, const PropertiesData& properties = PropertiesData());

  /// Constructor that can use an Addon with an identical Properties type
  AddonWithProtectedPropertiesInSkeleton(
      AddonManager* mgr,
      const AddonWithProtectedPropertiesInSkeleton<Properties, ManagerType>&
          otherAddon);

  // Documentation inherited
  void setAddonProperties(
      const std::unique_ptr<Addon::Properties>& someProperties) override final;

  // Documentation inherited
  const Addon::Properties* getAddonProperties() const override final;

protected:

  /// Properties of this Addon
  Properties mProperties;

  /// Skeleton for this Addon
  std::weak_ptr<dart::dynamics::Skeleton> mSkeleton;

};

//==============================================================================
/// AddonWithProtectedStateAndProperties combines the
/// AddonWithProtectedState and AddonWithProtectedProperties classes into a
/// single templated class
template <typename StateData, typename PropertiesData>
class AddonWithProtectedStateAndProperties :
    public AddonWithProtectedState<StateData>,
    public AddonWithProtectedProperties<PropertiesData>
{
public:

  AddonWithProtectedStateAndProperties() = delete;
  AddonWithProtectedStateAndProperties(
      const AddonWithProtectedStateAndProperties&) = delete;

  /// Construct using a StateData and a PropertiesData instance
  AddonWithProtectedStateAndProperties(
      AddonManager* mgr,
      const StateData& state = StateData(),
      const PropertiesData& properties = PropertiesData());

  /// Construct using just a PropertiesData instance
  AddonWithProtectedStateAndProperties(
      AddonManager* mgr,
      const PropertiesData& properties);

  /// Construct using a StateData and a PropertiesData instance, flipped
  AddonWithProtectedStateAndProperties(
      AddonManager* mgr,
      const PropertiesData& properties,
      const StateData& state);

  /// Constructor that can use an Addon with identical types
  AddonWithProtectedStateAndProperties(
      AddonManager* mgr,
      const AddonWithProtectedStateAndProperties<StateData, PropertiesData>&
          otherAddon);
};

//==============================================================================
/// AddonWithProtectedStateAndPropertiesInSkeleton combines the
/// AddonWithProtectedState and AddonWithProtectedPropertiesInSkeleton classes
/// into a single templated class. This should be the base class of any Addon
/// with both a State and Properties that will be embedded in a Skeleton or a
/// component of a Skeleton (such as a BodyNode, Joint, or custom Node).
template <typename StateData, typename PropertiesData, class ManagerType>
class AddonWithProtectedStateAndPropertiesInSkeleton :
    public AddonWithProtectedState<StateData>,
    public AddonWithProtectedProperties<PropertiesData>
{
public:

  AddonWithProtectedStateAndPropertiesInSkeleton() = delete;
  AddonWithProtectedStateAndPropertiesInSkeleton(
      const AddonWithProtectedStateAndPropertiesInSkeleton&) = delete;

  /// Construct using a StateData and a PropertiesData instance
  AddonWithProtectedStateAndPropertiesInSkeleton(
      ManagerType* mgr,
      const StateData& state = StateData(),
      const PropertiesData& properties = PropertiesData());

  /// Construct using just a PropertiesData instance
  AddonWithProtectedStateAndPropertiesInSkeleton(
      ManagerType* mgr,
      const PropertiesData& properties);

  /// Construct using a StateData and a PropertiesData instance, flipped
  AddonWithProtectedStateAndPropertiesInSkeleton(
      ManagerType* mgr,
      const PropertiesData& properties,
      const StateData& state);

  /// Constructor that can use an Addon with identical types
  AddonWithProtectedStateAndPropertiesInSkeleton(
      ManagerType* mgr,
      const AddonWithProtectedStateAndPropertiesInSkeleton<
          StateData, PropertiesData, ManagerType>& otherAddon);
};

} // namespace common
} // namespace dart

#endif // DART_COMMON_ADDON_H_
