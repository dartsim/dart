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

#ifndef DART_DYNAMICS_ADDON_H_
#define DART_DYNAMICS_ADDON_H_

#include "dart/common/Addon.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace dynamics {

//==============================================================================
/// AddonWithProtectedPropertiesInSkeleton generates implementations of the
/// Property managing functions for an Addon class which is embedded within a
/// Skeleton or a component of a Skeleton (such as a BodyNode, Joint, or custom
/// Node class). This will increment the version count any time the
/// Addon::setProperties function is called.
template <class Base, typename PropertiesData, class ManagerType = Node,
          void (*updateProperties)(Base*) = &common::detail::NoOp<Base*> >
class AddonWithProtectedPropertiesInSkeleton : public common::Addon
{
public:

  using Properties = common::Addon::PropertiesMixer<PropertiesData>;
  constexpr static void (*UpdateProperties)(Base*) = updateProperties;

  AddonWithProtectedPropertiesInSkeleton() = delete;
  AddonWithProtectedPropertiesInSkeleton(
      const AddonWithProtectedPropertiesInSkeleton&) = delete;

  /// Construct using a PropertiesData instance
  AddonWithProtectedPropertiesInSkeleton(
      ManagerType* mgr, const PropertiesData& properties = PropertiesData());

  // Documentation inherited
  void setAddonProperties(
      const std::unique_ptr<Addon::Properties>& someProperties) override final;

  // Documentation inherited
  const Addon::Properties* getAddonProperties() const override final;

  /// Get the Skeleton that this Addon is embedded in
  SkeletonPtr getSkeleton();

  /// Get the Skeleton that this Addon is embedded in
  ConstSkeletonPtr getSkeleton() const;

  /// Get the AddonManager that this Addon is embedded in
  ManagerType* getManager();

  /// Get the AddonManager that this Addon is embedded in
  const ManagerType* getManager() const;

  // Documentation inherited
  std::unique_ptr<common::Addon> cloneAddon(
      common::AddonManager* newManager) const override final;

protected:

  // Documentation inherited
  void changeManager(common::AddonManager* newManager) override;

  /// Properties of this Addon
  Properties mProperties;

  /// Manager that this Addon is embedded in
  ManagerType* mManager;

};

//==============================================================================
/// AddonWithProtectedStateAndPropertiesInSkeleton combines the
/// AddonWithProtectedState and AddonWithProtectedPropertiesInSkeleton classes
/// into a single templated class. This should be the base class of any Addon
/// with both a State and Properties that will be embedded in a Skeleton or a
/// component of a Skeleton (such as a BodyNode, Joint, or custom Node).
template <class Base, typename StateData, typename PropertiesData,
          class ManagerType = Node,
          void (*updateState)(Base*) = &common::detail::NoOp<Base*>,
          void (*updateProperties)(Base*) = updateState>
class AddonWithProtectedStateAndPropertiesInSkeleton : public common::Addon
{
public:

  using State = common::Addon::StateMixer<StateData>;
  using Properties = common::Addon::PropertiesMixer<PropertiesData>;
  constexpr static void (*UpdateState)(Base*) = updateState;
  constexpr static void (*UpdateProperties)(Base*) = updateProperties;

  AddonWithProtectedStateAndPropertiesInSkeleton() = delete;
  AddonWithProtectedStateAndPropertiesInSkeleton(
      const AddonWithProtectedStateAndPropertiesInSkeleton&) = delete;

  /// Construct using a StateData and a PropertiesData instance
  AddonWithProtectedStateAndPropertiesInSkeleton(
      ManagerType* mgr,
      const StateData& state = StateData(),
      const PropertiesData& properties = PropertiesData());

  /// Construct using a StateData and a PropertiesData instance, flipped
  AddonWithProtectedStateAndPropertiesInSkeleton(
      ManagerType* mgr,
      const PropertiesData& properties,
      const StateData& state = StateData());

  // Documentation inherited
  std::unique_ptr<common::Addon> cloneAddon(
      common::AddonManager* newManager) const override final;

protected:

  /// State of this Addon
  State mState;

  /// Properties of this Addon
  Properties mProperties;

  /// Manager that this Addon is embedded in
  ManagerType* mManager;

};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/Addon.h"

#endif // DART_DYNAMICS_ADDON_H_
