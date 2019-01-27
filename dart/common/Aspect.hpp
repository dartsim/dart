/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_COMMON_ASPECT_HPP_
#define DART_COMMON_ASPECT_HPP_

#include <string>

#include "dart/common/Cloneable.hpp"
#include "dart/common/detail/NoOp.hpp"

namespace dart {
namespace common {

class Composite;

class Aspect
{
public:

  friend class Composite;

  /// If your Aspect has a State, then that State class should inherit this
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
  class State : public Cloneable<State> { };

  /// Use the MakeState class to easily create a State extension from an
  /// existing class or struct.
  template <class Mixin>
  using MakeState = MakeCloneable<State, Mixin>;

  /// If your Aspect has Properties, then that Properties class should inherit this
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
  class Properties : public Cloneable<Properties> { };

  /// Use the MakeProperties class to easily create a Properties extension
  /// from an existing class or struct.
  template <class Mixin>
  using MakeProperties = MakeCloneable<Properties, Mixin>;

  /// Virtual destructor
  virtual ~Aspect() = default;

  /// Clone this Aspect into a new composite
  virtual std::unique_ptr<Aspect> cloneAspect() const = 0;

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

  /// This function will be triggered (1) after the Aspect has been created
  /// [transfer will be false] and (2) after the Aspect has been transferred
  /// to a new Composite [transfer will be true]. You should override this
  /// function if your Aspect requires special handling in either of those cases.
  /// By default, this function does nothing.
  virtual void setComposite(Composite* newComposite);

  /// This function will be triggered if your Aspect is about to be removed from
  /// its Composite. While this function is being called, the Aspect is still a
  /// valid part of the Composite; it will be removed immediately after this
  /// function call. By default, this function does nothing.
  virtual void loseComposite(Composite* oldComposite);
};

//==============================================================================
template <class CompositeType>
class CompositeTrackingAspect : public Aspect
{
public:

  /// Default constructor
  CompositeTrackingAspect();

  /// Get the Composite of this Aspect
  CompositeType* getComposite();

  /// Get the Composite of this Aspect
  const CompositeType* getComposite() const;

  /// Returns true if this Aspect has a Composite that matches CompositeType
  bool hasComposite() const;

protected:

  /// Grab the new Composite
  void setComposite(Composite* newComposite) override;

  /// Clear the old Composite
  void loseComposite(Composite* oldComposite) override;

  /// Pointer to the current Composite of this Aspect
  CompositeType* mComposite;

};

} // namespace common
} // namespace dart

//==============================================================================
#define DART_COMMON_ASPECT_PROPERTY_CONSTRUCTOR( ClassName, UpdatePropertiesMacro )\
  ClassName (const ClassName &) = delete;\
  inline ClassName (const PropertiesData& properties = PropertiesData())\
    : AspectWithVersionedProperties< Base, Derived, PropertiesData, CompositeType, UpdatePropertiesMacro>(properties) { }

//==============================================================================
#define DART_COMMON_ASPECT_STATE_PROPERTY_CONSTRUCTORS(ClassName)\
  ClassName (const ClassName &) = delete;\
  inline ClassName (const StateData& state = StateData(), const PropertiesData& properties = PropertiesData())\
    : AspectImpl(state, properties) { }\
  inline ClassName (const PropertiesData& properties, const StateData state = StateData())\
    : AspectImpl(properties, state) { }

//==============================================================================
#define DART_COMMON_SET_ASPECT_PROPERTY_CUSTOM( Type, Name, Update )\
  inline void set ## Name (const Type & value)\
  { mProperties.m ## Name = value; Update(); }

//==============================================================================
#define DART_COMMON_SET_ASPECT_PROPERTY( Type, Name )\
  DART_COMMON_SET_ASPECT_PROPERTY_CUSTOM( Type, Name, notifyPropertiesUpdated )

//==============================================================================
#define DART_COMMON_GET_ASPECT_PROPERTY( Type, Name )\
  inline const Type& get ## Name () const\
  { return mProperties.m ## Name; }

//==============================================================================
#define DART_COMMON_SET_GET_ASPECT_PROPERTY( Type, Name )\
  DART_COMMON_SET_ASPECT_PROPERTY( Type, Name )\
  DART_COMMON_GET_ASPECT_PROPERTY( Type, Name )

#include "dart/common/detail/Aspect.hpp"

#endif // DART_COMMON_ASPECT_HPP_
