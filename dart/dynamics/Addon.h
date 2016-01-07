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
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace dynamics {

//==============================================================================
/// AddonWithProtectedPropertiesInSkeleton generates implementations of the
/// Property managing functions for an Addon class which is embedded within a
/// Skeleton or a component of a Skeleton (such as a BodyNode, Joint, or custom
/// Node class). This will increment the version count any time the
/// Addon::setProperties function is called.
template <class BaseT, typename PropertiesDataT, class ManagerT = Node,
          void (*updateProperties)(BaseT*) = common::detail::NoOp<BaseT*>,
          bool OptionalT = true>
class AddonWithProtectedPropertiesInSkeleton : public common::Addon
{
public:

  using Base = BaseT;
  using PropertiesData = PropertiesDataT;
  using ManagerType = ManagerT;
  using Properties = common::Addon::PropertiesMixer<PropertiesData>;
  constexpr static void (*UpdateProperties)(Base*) = updateProperties;
  constexpr static bool Optional = OptionalT;

  using Implementation = AddonWithProtectedPropertiesInSkeleton<
      BaseT, PropertiesDataT, ManagerT, updateProperties, OptionalT>;

  AddonWithProtectedPropertiesInSkeleton() = delete;
  AddonWithProtectedPropertiesInSkeleton(
      const AddonWithProtectedPropertiesInSkeleton&) = delete;

  /// Construct using a PropertiesData instance
  AddonWithProtectedPropertiesInSkeleton(
      common::AddonManager* mgr,
      const PropertiesData& properties = PropertiesData());

  // Documentation inherited
  std::unique_ptr<common::Addon> cloneAddon(
      common::AddonManager* newManager) const override final;

  // Documentation inherited
  void setAddonProperties(const Addon::Properties& someProperties) override final;

  // Documentation inherited
  const Addon::Properties* getAddonProperties() const override final;

  /// Set the Properties of this Addon
  void setProperties(const PropertiesData& properties);

  /// Get the Properties of this Addon
  const Properties& getProperties() const;

  // Documentation inherited
  bool isOptional(common::AddonManager* oldManager) override final;

  /// Get the Skeleton that this Addon is embedded in
  SkeletonPtr getSkeleton();

  /// Get the Skeleton that this Addon is embedded in
  ConstSkeletonPtr getSkeleton() const;

  /// Get the AddonManager that this Addon is embedded in
  ManagerType* getManager();

  /// Get the AddonManager that this Addon is embedded in
  const ManagerType* getManager() const;

  /// Increment the version number of the Skeleton this Addon is attached to
  void incrementSkeletonVersion();

protected:

  // Documentation inherited
  void setManager(common::AddonManager* newManager, bool transfer) override;

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
template <class BaseT, typename StateDataT, typename PropertiesDataT,
          class ManagerT = Node,
          void (*updateState)(BaseT*) = &common::detail::NoOp<BaseT*>,
          void (*updateProperties)(BaseT*) = updateState,
          bool OptionalT = true>
class AddonWithProtectedStateAndPropertiesInSkeleton : public common::Addon
{
public:

  using Base = BaseT;
  using StateData = StateDataT;
  using PropertiesData = PropertiesDataT;
  using ManagerType = ManagerT;
  using State = common::Addon::StateMixer<StateData>;
  using Properties = common::Addon::PropertiesMixer<PropertiesData>;
  constexpr static void (*UpdateState)(Base*) = updateState;
  constexpr static void (*UpdateProperties)(Base*) = updateProperties;
  constexpr static bool Optional = OptionalT;

  AddonWithProtectedStateAndPropertiesInSkeleton() = delete;
  AddonWithProtectedStateAndPropertiesInSkeleton(
      const AddonWithProtectedStateAndPropertiesInSkeleton&) = delete;

  /// Construct using a StateData and a PropertiesData instance
  AddonWithProtectedStateAndPropertiesInSkeleton(
      common::AddonManager* mgr,
      const StateDataT& state = StateData(),
      const PropertiesDataT& properties = PropertiesData());

  /// Construct using a StateData and a PropertiesData instance, flipped
  AddonWithProtectedStateAndPropertiesInSkeleton(
      common::AddonManager* mgr,
      const PropertiesData& properties,
      const StateData& state = StateData());

  // Documentation inherited
  std::unique_ptr<common::Addon> cloneAddon(
      common::AddonManager* newManager) const override final;

  // Documentation inherited
  void setAddonState(const Addon::State& otherState) override final;

  // Documentation inherited
  const Addon::State* getAddonState() const override final;

  /// Set the State of this Addon
  void setState(const StateData& state);

  /// Get the State of this Addon
  const State& getState() const;

  // Documentation inherited
  void setAddonProperties(const Addon::Properties& properties) override final;

  // Documentation inherited
  const Addon::Properties* getAddonProperties() const override final;

  /// Set the Properties of this Addon
  void setProperties(const PropertiesData& properties);

  /// Get the Properties of this Addon
  const Properties& getProperties() const;

  // Documentation inherited
  bool isOptional(common::AddonManager* oldManager) override final;

  /// Get the Skeleton that this Addon is embedded in
  SkeletonPtr getSkeleton();

  /// Get the Skeleton that this Addon is embedded in
  ConstSkeletonPtr getSkeleton() const;

  /// Get the AddonManager that this Addon is embedded in
  ManagerType* getManager();

  /// Get the AddonManager that this Addon is embedded in
  const ManagerType* getManager() const;

  /// Increment the version number of the Skeleton this Addon is attached to
  void incrementSkeletonVersion();

protected:

  // Documentation inherited
  void setManager(common::AddonManager* newManager, bool transfer) override;

  /// State of this Addon
  State mState;

  /// Properties of this Addon
  Properties mProperties;

  /// Manager that this Addon is embedded in
  ManagerType* mManager;

};

} // namespace dynamics
} // namespace dart

//==============================================================================
#define DART_DYNAMICS_ADDON_PROPERTY_CONSTRUCTOR( ClassName, UpdatePropertiesMacro )\
  ClassName (const ClassName &) = delete;\
  inline ClassName (dart::common::AddonManager* mgr, const PropertiesData& properties)\
    : AddonWithProtectedPropertiesInSkeleton< Base, PropertiesData, ManagerType, UpdatePropertiesMacro, Optional>(mgr, properties) { }

//==============================================================================
#define DART_DYNAMICS_JOINT_ADDON_CONSTRUCTOR( ClassName )\
  DART_DYNAMICS_ADDON_PROPERTY_CONSTRUCTOR( ClassName, &detail::JointPropertyUpdate )

//==============================================================================
#define DART_DYNAMICS_ADDON_STATE_PROPERTY_CONSTRUCTORS( ClassName, UpdateStateMacro, UpdatePropertiesMacro )\
  ClassName (const ClassName &) = delete;\
  inline ClassName (dart::common::AddonManager* mgr, const StateData& state = StateData(), const PropertiesData& properties = PropertiesData())\
    : AddonWithProtectedStateAndPropertiesInSkeleton< Base, StateData, PropertiesData, ManagerType, UpdateStateMacro, UpdatePropertiesMacro, Optional >(mgr, state, properties) { }\
  inline ClassName (dart::common::AddonManager* mgr, const PropertiesData& properties, const StateData state = StateData())\
    : AddonWithProtectedStateAndPropertiesInSkeleton< Base, StateData, PropertiesData, ManagerType, UpdateStateMacro, UpdatePropertiesMacro, Optional >(mgr, properties, state) { }

//==============================================================================
#define DART_DYNAMICS_SET_ADDON_PROPERTY_CUSTOM( Type, Name, Update )\
  inline void set ## Name (const Type & value)\
  { mProperties.m ## Name = value; Update(this); incrementSkeletonVersion(); }

//==============================================================================
#define DART_DYNAMICS_SET_ADDON_PROPERTY( Type, Name )\
  DART_DYNAMICS_SET_ADDON_PROPERTY_CUSTOM( Type, Name, UpdateProperties )

//==============================================================================
#define DART_DYNAMICS_GET_ADDON_PROPERTY( Type, Name )\
  inline const Type& get ## Name () const\
  { return mProperties.m ## Name; }

//==============================================================================
#define DART_DYNAMICS_SET_GET_ADDON_PROPERTY( Type, Name )\
  DART_DYNAMICS_SET_ADDON_PROPERTY( Type, Name )\
  DART_DYNAMICS_GET_ADDON_PROPERTY( Type, Name )

//==============================================================================
#define DART_DYNAMICS_SET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size, UpdatePrefix )\
  void set ## SingleName (size_t index, const SingleType & value)\
  {\
    if( index >= Size )\
    {\
      dterr << "[" #Class << "::set" #SingleName << "] Invalid index (" << index << "). "\
            << "The specified index must be less than " #Size << "!\n";\
      assert(false); return;\
    }\
    this->mProperties.m ## PluralName [index] = value;\
    UpdatePrefix :: UpdateProperties(this);\
    this->incrementSkeletonVersion();\
  }\
  void set ## PluralName (const VectorType & vec)\
  {\
    this->mProperties.m ## PluralName = vec;\
    UpdatePrefix :: UpdateProperties(this);\
    this->incrementSkeletonVersion();\
  }

//==============================================================================
#define DART_DYNAMICS_GET_ADDON_PROPERTY_ARRAY(Class, SingleType, VectorType, SingleName, PluralName, Size)\
  inline const SingleType& get ## SingleName (size_t index) const\
  {\
    if(index >= Size)\
    {\
      dterr << "[" #Class << "::get" #SingleName << "] Invalid index (" << index << "). "\
            << "The specified index must be less than " #Size << "!\n";\
      assert(false); index = 0;\
    }\
    return this->mProperties.m ## PluralName [index];\
  }\
  inline const VectorType& get ## PluralName () const\
  {\
    return this->mProperties.m ## PluralName;\
  }

//==============================================================================
#define DART_DYNAMICS_IRREGULAR_SET_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size, UpdatePrefix )\
  DART_DYNAMICS_SET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size, UpdatePrefix )\
  DART_DYNAMICS_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size )

//==============================================================================
#define DART_DYNAMICS_SET_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, Size, UpdatePrefix )\
  DART_DYNAMICS_IRREGULAR_SET_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, SingleName ## s, Size, UpdatePrefix )

//==============================================================================
#define DART_DYNAMICS_IRREGULAR_SET_GET_MULTIDOF_ADDON( SingleType, VectorType, SingleName, PluralName )\
  DART_DYNAMICS_IRREGULAR_SET_GET_ADDON_PROPERTY_ARRAY( MultiDofJointAddon, SingleType, VectorType, SingleName, PluralName, DOF, MultiDofJoint<DOF>::Addon )

//==============================================================================
#define DART_DYNAMICS_SET_GET_MULTIDOF_ADDON( SingleType, VectorType, SingleName )\
  DART_DYNAMICS_IRREGULAR_SET_GET_MULTIDOF_ADDON( SingleType, VectorType, SingleName, SingleName ## s )

//==============================================================================
#define DETAIL_DART_ADDON_PROPERTIES_UPDATE( AddonName, GetAddon )\
  AddonName :: UpdateProperties( GetAddon () );\
  GetAddon ()->incrementSkeletonVersion();

//==============================================================================
#define DETAIL_DART_ADDON_STATE_PROPERTIES_UPDATE( AddonName, GetAddon )\
  AddonName :: UpdateState( GetAddon () );\
  DETAIL_DART_ADDON_PROPERTIES_UPDATE( AddonName, GetAddon );

//==============================================================================
// Used for Addons that have Properties (but no State) inside of a Skeleton
#define DART_DYNAMICS_SKEL_PROPERTIES_ADDON_INLINE( AddonName )\
  DETAIL_DART_SPECIALIZED_ADDON_INLINE( AddonName,\
      DETAIL_DART_ADDON_PROPERTIES_UPDATE( AddonName, get ## AddonName ) )

//==============================================================================
// Used for Addons that have both State and Properties inside of a Skeleton
#define DART_DYNAMICS_SKEL_ADDON_INLINE( AddonName )\
  DETAIL_DART_SPECIALIZED_ADDON_INLINE( AddonName,\
      DETAIL_DART_ADDON_STATE_PROPERTIES_UPDATE( AddonName, get ## AddonName ) )

//==============================================================================
// Used for edge cases, such as nested template classes, that have Properties
// (but no State) inside of a Skeleton
#define DART_DYNAMICS_IRREGULAR_SKEL_PROPERTIES_ADDON_INLINE( TypeName, HomogenizedName )\
  DETAIL_DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( TypeName, HomogenizedName,\
    DETAIL_DART_ADDON_PROPERTIES_UPDATE( TypeName, get ## HomogenizedName ) )

//==============================================================================
// Used for edge cases, such as nested template classes, that have both State
// and Properties inside of a Skeleton
#define DART_DYNAMICS_IRREGULAR_SKEL_ADDON_INLINE( TypeName, HomogenizedName )\
  DETAIL_DART_IRREGULAR_SPECIALIZED_ADDON_INLINE( TypeName, HomogenizedName,\
    DETAIL_DART_ADDON_STATE_PROPERTIES_UPDATE( TypeName, get ## HomogenizedName ) )

//==============================================================================
// Used for nested-class Addons that have Properties (but no State) inside of a Skeleton
#define DART_DYNAMICS_NESTED_SKEL_PROPERTIES_ADDON_INLINE( ParentName, AddonName )\
  DART_DYNAMICS_IRREGULAR_SKEL_PROPERTIES_ADDON_INLINE( ParentName :: AddonName, ParentName ## AddonName )

//==============================================================================
// Used for nested-class Addons that have both State and Properties inside of a Skeleton
#define DART_DYNAMICS_NESTED_SKEL_ADDON_INLINE( ParentName, AddonName )\
  DART_DYNAMICS_IRREGULAR_SKEL_ADDON_INLINE( ParentName :: AddonName, ParentName ## AddonName )

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/detail/Addon.h"

#endif // DART_DYNAMICS_ADDON_H_
