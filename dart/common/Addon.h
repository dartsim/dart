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
#include "dart/common/detail/NoOp.h"

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
  virtual void setAddonState(const State& otherState);

  /// Get the State of this Addon. By default, this returns a nullptr which
  /// implies that the Addon is stateless.
  virtual const State* getAddonState() const;

  /// Set the Properties of this Addon. By default, this does nothing.
  virtual void setAddonProperties(const Properties& someProperties);

  /// Get the Properties of this Addon. By default, this returns a nullptr
  /// which implies that the Addon has no properties.
  virtual const Properties* getAddonProperties() const;

  /// This function will be called if the user is attempting to delete the Addon
  /// but not immediately replacing it with another Addon of the same type. The
  /// incoming argument will point to the AddonManager that had been holding
  /// this Addon.
  ///
  /// If your Addon is mandatory for the AddonManager type that is passed in
  /// here, then you should perform error handling in this function, and you
  /// should return false to indicate that the operation is not permitted. If
  /// you return false, then the Addon will NOT be removed from its Manager.
  ///
  /// By default, this simply returns true.
  virtual bool isOptional(AddonManager* oldManager);

protected:

  /// Constructor
  ///
  /// We require the AddonManager argument in this constructor to make it clear
  /// to extensions that they must have an AddonManager argument in their
  /// constructors.
  Addon(AddonManager* manager);

  /// This function will be triggered (1) after the Addon has been created
  /// [transfer will be false] and (2) after the Addon has been transferred
  /// to a new AddonManager [transfer will be true]. You should override this
  /// function if your Addon requires special handling in either of those cases.
  /// By default, this function does nothing.
  virtual void setManager(AddonManager* newManager, bool transfer);

};

//==============================================================================
template <class ManagerType>
class ManagerTrackingAddon : public Addon
{
public:

  /// Default constructor
  ManagerTrackingAddon(AddonManager* mgr);

  /// Get the Manager of this Addon
  ManagerType* getManager();

  /// Get the Manager of this Addon
  const ManagerType* getManager() const;

protected:

  /// Grab the new manager
  void setManager(AddonManager* newManager, bool transfer);

  /// Pointer to the current Manager of this Addon
  ManagerType* mManager;

};

} // namespace common
} // namespace dart

//==============================================================================
#define DART_COMMON_ADDON_PROPERTY_CONSTRUCTOR( ClassName, UpdatePropertiesMacro )\
  ClassName (const ClassName &) = delete;\
  inline ClassName (dart::common::AddonManager* mgr, const PropertiesData& properties = PropertiesData())\
    : AddonWithVersionedProperties< Base, Derived, PropertiesData, ManagerType, UpdatePropertiesMacro>(mgr, properties) { }

//==============================================================================
#define DART_COMMON_JOINT_ADDON_CONSTRUCTOR( ClassName )\
  DART_COMMON_ADDON_PROPERTY_CONSTRUCTOR( ClassName, &detail::JointPropertyUpdate )

//==============================================================================
#define DART_COMMON_ADDON_STATE_PROPERTY_CONSTRUCTORS(ClassName)\
  ClassName (const ClassName &) = delete;\
  inline ClassName (dart::common::AddonManager* mgr, const StateData& state = StateData(), const PropertiesData& properties = PropertiesData())\
    : AddonImplementation(mgr, state, properties) { }\
  inline ClassName (dart::common::AddonManager* mgr, const PropertiesData& properties, const StateData state = StateData())\
    : AddonImplementation(mgr, properties, state) { }

//==============================================================================
#define DART_COMMON_SET_ADDON_PROPERTY_CUSTOM( Type, Name, Update )\
  inline void set ## Name (const Type & value)\
  { mProperties.m ## Name = value; Update(this); }

//==============================================================================
#define DART_COMMON_SET_ADDON_PROPERTY( Type, Name )\
  DART_COMMON_SET_ADDON_PROPERTY_CUSTOM( Type, Name, UpdateProperties )

//==============================================================================
#define DART_COMMON_GET_ADDON_PROPERTY( Type, Name )\
  inline const Type& get ## Name () const\
  { return mProperties.m ## Name; }

//==============================================================================
#define DART_COMMON_SET_GET_ADDON_PROPERTY( Type, Name )\
  DART_COMMON_SET_ADDON_PROPERTY( Type, Name )\
  DART_COMMON_GET_ADDON_PROPERTY( Type, Name )

//==============================================================================
#define DART_COMMON_SET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size, UpdatePrefix )\
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
    this->incrementVersion();\
  }\
  void set ## PluralName (const VectorType & vec)\
  {\
    this->mProperties.m ## PluralName = vec;\
    UpdatePrefix :: UpdateProperties(this);\
    this->incrementVersion();\
  }

//==============================================================================
#define DART_COMMON_GET_ADDON_PROPERTY_ARRAY(Class, SingleType, VectorType, SingleName, PluralName, Size)\
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
#define DART_COMMON_IRREGULAR_SET_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size, UpdatePrefix )\
  DART_COMMON_SET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size, UpdatePrefix )\
  DART_COMMON_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size )

//==============================================================================
#define DART_COMMON_SET_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, Size, UpdatePrefix )\
  DART_COMMON_IRREGULAR_SET_GET_ADDON_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, SingleName ## s, Size, UpdatePrefix )

//==============================================================================
#define DART_COMMON_IRREGULAR_SET_GET_MULTIDOF_ADDON( SingleType, VectorType, SingleName, PluralName )\
  DART_COMMON_IRREGULAR_SET_GET_ADDON_PROPERTY_ARRAY( MultiDofJointAddon, SingleType, VectorType, SingleName, PluralName, DOF, MultiDofJoint<DOF>::Addon )

//==============================================================================
#define DART_COMMON_SET_GET_MULTIDOF_ADDON( SingleType, VectorType, SingleName )\
  DART_COMMON_IRREGULAR_SET_GET_MULTIDOF_ADDON( SingleType, VectorType, SingleName, SingleName ## s )

//==============================================================================
#define DETAIL_DART_ADDON_PROPERTIES_UPDATE( AddonName, GetAddon )\
  AddonName :: UpdateProperties( GetAddon () );\
  GetAddon ()->incrementVersion();

//==============================================================================
#define DETAIL_DART_ADDON_STATE_PROPERTIES_UPDATE( AddonName, GetAddon )\
  AddonName :: UpdateState( GetAddon () );\
  DETAIL_DART_ADDON_PROPERTIES_UPDATE( AddonName, GetAddon );

#include "dart/common/detail/Addon.h"

#endif // DART_COMMON_ADDON_H_
