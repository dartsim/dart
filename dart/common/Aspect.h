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

#ifndef DART_COMMON_ASPECT_H_
#define DART_COMMON_ASPECT_H_

#include <string>

#include "dart/common/Extensible.h"
#include "dart/common/detail/NoOp.h"

namespace dart {
namespace common {

class Composite;

class Aspect
{
public:

  friend class Composite;

  /// If your Aspect has a State class, then that State class should inherit this
  /// Aspect::State class. This allows us to safely serialize, store, and clone
  /// the states of arbitrary Aspect extensions. If your Aspect is stateless, then
  /// you do not have to worry about extending this class, because
  /// Aspect::getState() will simply return a nullptr by default, which is taken
  /// to indicate that it is stateless.
  ///
  /// The distinction between the State class and the Properties class is that
  /// State will get stored in Composite::State whereas Properties will get
  /// stored in Composite::Properties. Typically Properties are values that
  /// only change rarely if ever, whereas State contains values that might
  /// change as often as every time step.
  class State : public Extensible<State> { };

  /// Use the StateMixer class to easily create a State extension from an
  /// existing class or struct.
  template <class Mixin>
  using StateMixer = ExtensibleMixer<State, Mixin>;

  /// If your Aspect has a Properties class, then it should inherit this
  /// Aspect::Properties class. This allows us to safely serialize, store, and
  /// clone the properties of arbitrary Aspect extensions. If your Aspect has no
  /// properties, then you do not have to worry about extending this class,
  /// because Aspect::getProperties() will simply return a nullptr by default,
  /// which is taken to indicate that it has no properties.
  ///
  /// The distinction between the State class and the Properties class is that
  /// State will get stored in Composite::State whereas Properties will get
  /// stored in Composite::Properties. Typically Properties are values that
  /// only change rarely if ever, whereas State contains values that might
  /// change as often as every time step.
  class Properties : public Extensible<Properties> { };

  /// Use the PropertiesMixer class to easily create a Properties extension
  /// from an existing class or struct.
  template <class Mixin>
  using PropertiesMixer = ExtensibleMixer<Properties, Mixin>;

  /// Virtual destructor
  virtual ~Aspect() = default;

  /// Clone this Aspect into a new manager
  virtual std::unique_ptr<Aspect> cloneAspect(Composite* newManager) const = 0;

  /// Set the State of this Aspect. By default, this does nothing.
  virtual void setAspectState(const State& otherState);

  /// Get the State of this Aspect. By default, this returns a nullptr which
  /// implies that the Aspect is stateless.
  virtual const State* getAspectState() const;

  /// Set the Properties of this Aspect. By default, this does nothing.
  virtual void setAspectProperties(const Properties& someProperties);

  /// Get the Properties of this Aspect. By default, this returns a nullptr
  /// which implies that the Aspect has no properties.
  virtual const Properties* getAspectProperties() const;

protected:

  /// Constructor
  ///
  /// We require the Composite argument in this constructor to make it clear
  /// to extensions that they must have an Composite argument in their
  /// constructors.
  Aspect(Composite* manager);

  /// This function will be triggered (1) after the Aspect has been created
  /// [transfer will be false] and (2) after the Aspect has been transferred
  /// to a new Composite [transfer will be true]. You should override this
  /// function if your Aspect requires special handling in either of those cases.
  /// By default, this function does nothing.
  virtual void setManager(Composite* newManager, bool transfer);

};

//==============================================================================
template <class ManagerType>
class ManagerTrackingAspect : public Aspect
{
public:

  /// Default constructor
  ManagerTrackingAspect(Composite* mgr);

  /// Get the Manager of this Aspect
  ManagerType* getManager();

  /// Get the Manager of this Aspect
  const ManagerType* getManager() const;

protected:

  /// Grab the new manager
  void setManager(Composite* newManager, bool transfer);

  /// Pointer to the current Manager of this Aspect
  ManagerType* mManager;

};

} // namespace common
} // namespace dart

//==============================================================================
#define DART_COMMON_ASPECT_PROPERTY_CONSTRUCTOR( ClassName, UpdatePropertiesMacro )\
  ClassName (const ClassName &) = delete;\
  inline ClassName (dart::common::Composite* mgr, const PropertiesData& properties = PropertiesData())\
    : AspectWithVersionedProperties< Base, Derived, PropertiesData, ManagerType, UpdatePropertiesMacro>(mgr, properties) { }

//==============================================================================
#define DART_COMMON_JOINT_ASPECT_CONSTRUCTOR( ClassName )\
  DART_COMMON_ASPECT_PROPERTY_CONSTRUCTOR( ClassName, &detail::JointPropertyUpdate )

//==============================================================================
#define DART_COMMON_ASPECT_STATE_PROPERTY_CONSTRUCTORS(ClassName)\
  ClassName (const ClassName &) = delete;\
  inline ClassName (dart::common::Composite* mgr, const StateData& state = StateData(), const PropertiesData& properties = PropertiesData())\
    : AspectImplementation(mgr, state, properties) { }\
  inline ClassName (dart::common::Composite* mgr, const PropertiesData& properties, const StateData state = StateData())\
    : AspectImplementation(mgr, properties, state) { }

//==============================================================================
#define DART_COMMON_SET_ASPECT_PROPERTY_CUSTOM( Type, Name, Update )\
  inline void set ## Name (const Type & value)\
  { mProperties.m ## Name = value; Update(); }

//==============================================================================
#define DART_COMMON_SET_ASPECT_PROPERTY( Type, Name )\
  DART_COMMON_SET_ASPECT_PROPERTY_CUSTOM( Type, Name, notifyPropertiesUpdate )

//==============================================================================
#define DART_COMMON_GET_ASPECT_PROPERTY( Type, Name )\
  inline const Type& get ## Name () const\
  { return mProperties.m ## Name; }

//==============================================================================
#define DART_COMMON_SET_GET_ASPECT_PROPERTY( Type, Name )\
  DART_COMMON_SET_ASPECT_PROPERTY( Type, Name )\
  DART_COMMON_GET_ASPECT_PROPERTY( Type, Name )

//==============================================================================
#define DART_COMMON_SET_ASPECT_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size )\
  void set ## SingleName (size_t index, const SingleType & value)\
  {\
    if( index >= Size )\
    {\
      dterr << "[" #Class << "::set" #SingleName << "] Invalid index (" << index << "). "\
            << "The specified index must be less than " #Size << "!\n";\
      assert(false); return;\
    }\
    this->mProperties.m ## PluralName [index] = value;\
    this->notifyPropertiesUpdate();\
  }\
  void set ## PluralName (const VectorType & vec)\
  {\
    this->mProperties.m ## PluralName = vec;\
    this->notifyPropertiesUpdate();\
  }

//==============================================================================
#define DART_COMMON_GET_ASPECT_PROPERTY_ARRAY(Class, SingleType, VectorType, SingleName, PluralName, Size)\
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
#define DART_COMMON_IRREGULAR_SET_GET_ASPECT_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size )\
  DART_COMMON_SET_ASPECT_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size )\
  DART_COMMON_GET_ASPECT_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, PluralName, Size )

//==============================================================================
#define DART_COMMON_SET_GET_ASPECT_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, Size )\
  DART_COMMON_IRREGULAR_SET_GET_ASPECT_PROPERTY_ARRAY( Class, SingleType, VectorType, SingleName, SingleName ## s, Size )

//==============================================================================
#define DART_COMMON_IRREGULAR_SET_GET_MULTIDOF_ASPECT( SingleType, VectorType, SingleName, PluralName )\
  DART_COMMON_IRREGULAR_SET_GET_ASPECT_PROPERTY_ARRAY( MultiDofJointAspect, SingleType, VectorType, SingleName, PluralName, DOF )

//==============================================================================
#define DART_COMMON_SET_GET_MULTIDOF_ASPECT( SingleType, VectorType, SingleName )\
  DART_COMMON_IRREGULAR_SET_GET_MULTIDOF_ASPECT( SingleType, VectorType, SingleName, SingleName ## s )

#include "dart/common/detail/Aspect.h"

#endif // DART_COMMON_ASPECT_H_
