/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COMMON_DETAIL_EMBEDDEDASPECT_HPP_
#define DART_COMMON_DETAIL_EMBEDDEDASPECT_HPP_

#include "dart/common/Aspect.hpp"
#include "dart/common/StlHelpers.hpp"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
template <class AspectT, typename StateT>
void DefaultSetEmbeddedState(AspectT* aspect, const StateT& state)
{
  aspect->getComposite()->setAspectState(state);
}

//==============================================================================
template <class AspectT, typename StateT>
const StateT& DefaultGetEmbeddedState(const AspectT* aspect)
{
  return aspect->getComposite()->getAspectState();
}

//==============================================================================
template <class AspectT, typename PropertiesT>
void DefaultSetEmbeddedProperties(AspectT* aspect, const PropertiesT& properties)
{
  aspect->getComposite()->setAspectProperties(properties);
}

//==============================================================================
template <class AspectT, typename PropertiesT>
const PropertiesT& DefaultGetEmbeddedProperties(const AspectT* aspect)
{
  return aspect->getComposite()->getAspectProperties();
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateDataT,
          typename StateT = common::Aspect::MakeState<StateDataT>,
          void (*setEmbeddedState)(DerivedT*, const StateT&) =
              &DefaultSetEmbeddedState<DerivedT, StateT>,
          const StateT& (*getEmbeddedState)(const DerivedT*) =
              &DefaultGetEmbeddedState<DerivedT, StateT> >
class EmbeddedStateAspect : public BaseT
{
public:

  using Base = BaseT;
  using Derived = DerivedT;
  using State = StateT;
  using StateData = StateDataT;
  constexpr static void (*SetEmbeddedState)(Derived*, const State&) = setEmbeddedState;
  constexpr static const State& (*GetEmbeddedState)(const Derived*) = getEmbeddedState;

  enum DelegateTag { Delegate };

  EmbeddedStateAspect(const EmbeddedStateAspect&) = delete;

  virtual ~EmbeddedStateAspect() = default;

  /// Used to identify constructor arguments that can be used as a State
  template <typename T>
  struct ConvertIfState
  {
    using type = typename std::conditional<
        std::is_base_of<StateData, T>::value,
        StateData, T>::type;
  };

  /// Construct this Aspect without affecting the State.
  EmbeddedStateAspect()
    : Base()
  {
    // Do nothing
  }

  /// Construct this Aspect. If the first argument contains StateData, then it
  /// will be used by this Aspect. Otherwise, all arguments will be forwarded to
  /// the Base class.
  //
  // Dev Note: The complex construction pattern used here allows us to satisfy
  // three simultaneous design constraints:
  //   1. We can identify when the user has passed in relevant State information
  //      and capture that information. The type can be of **any** class type
  //      that inherits StateData.
  //   2. We can ignore any non-State information that the user has passed in,
  //      and move that information along to the Base class.
  //   3. We can handle arbitrary numbers of arguments of any type to pass along
  //      to the Base class.
  // If anyone can come up with a cleaner way of accomplishing all three of
  // these constraints, I would gladly replace this implementation. -(MXG)
  template <typename T, typename... RemainingArgs>
  EmbeddedStateAspect(
      const T& arg1,
      RemainingArgs&&... remainingArgs)
    : EmbeddedStateAspect(
        Delegate,
        static_cast<const typename ConvertIfState<T>::type&>(arg1),
        std::forward<RemainingArgs>(remainingArgs)...)
  {
    // Do nothing
  }

  // Documentation inherited
  void setAspectState(const Aspect::State& state) override final
  {
    setState(static_cast<const State&>(state));
  }

  /// Set the State of this Aspect
  void setState(const State& state)
  {
    if(this->hasComposite())
    {
      SetEmbeddedState(static_cast<Derived*>(this), state);
      return;
    }

    // If the correct type of Composite is not available, we store this on the
    // heap until this Aspect is moved.
    mTemporaryState = make_unique<State>(state);
  }

  // Documentation inherited
  const Aspect::State* getAspectState() const override final
  {
    return &getState();
  }

  /// Get the State of this Aspect
  const State& getState() const
  {
    if(this->hasComposite())
    {
      return GetEmbeddedState(static_cast<const Derived*>(this));
    }

    if(!mTemporaryState)
    {
      dterr << "[detail::EmbeddedStateAspect::getState] This Aspect is not in "
            << "a Composite, but it also does not have a temporary State "
            << "available. This should not happen! Please report this as a "
            << "bug!\n";
      assert(false);
    }

    return *mTemporaryState;
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<Derived>(this->getState());
  }

protected:

  /// Construct this Aspect using the StateData, and pass the remaining
  /// arguments into the constructor of the Base class.
  template <typename... RemainingArgs>
  EmbeddedStateAspect(
      DelegateTag, const StateData& state,
      RemainingArgs&&... remainingArgs)
    : Base(std::forward<RemainingArgs>(remainingArgs)...),
      mTemporaryState(make_unique<State>(state))
  {
    // Do nothing
  }

  /// Construct this Aspect without affecting the State, and pass all the
  /// arguments into the constructor of the Base class.
  template <typename... BaseArgs>
  EmbeddedStateAspect(
      DelegateTag, BaseArgs&&... args)
    : Base(std::forward<BaseArgs>(args)...)
  {
    // Do nothing
  }

  /// Pass the temporary State of this Aspect into the new Composite
  void setComposite(Composite* newComposite) override
  {
    assert(nullptr == this->getComposite());

    Base::setComposite(newComposite);
    if(mTemporaryState)
      SetEmbeddedState(static_cast<Derived*>(this), *mTemporaryState);

    mTemporaryState = nullptr;
  }

  /// Save the embedded State of this Composite before we remove the Aspect
  void loseComposite(Composite* oldComposite) override
  {
    mTemporaryState = make_unique<State>(
          GetEmbeddedState(static_cast<const Derived*>(this)));
    Base::loseComposite(oldComposite);
  }

  /// After this Aspect is constructed and during transitions between Composite
  /// objects, this will hold the State of the Aspect. Once the Aspect has been
  /// moved into a new Composite, this State will be pushed into the Composite
  /// and cleared.
  std::unique_ptr<State> mTemporaryState;

};

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesDataT,
          typename PropertiesT = common::Aspect::MakeProperties<PropertiesDataT>,
          void (*setEmbeddedProperties)(DerivedT*, const PropertiesT&) =
              &DefaultSetEmbeddedProperties<DerivedT, PropertiesT>,
          const PropertiesT& (*getEmbeddedProperties)(const DerivedT*) =
              &DefaultGetEmbeddedProperties<DerivedT, PropertiesT> >
class EmbeddedPropertiesAspect : public BaseT
{
protected:

  enum DelegateTag { Delegate };

public:

  using Base = BaseT;
  using Derived = DerivedT;
  using Properties = PropertiesT;
  using PropertiesData = PropertiesDataT;
  constexpr static void (*SetEmbeddedProperties)(Derived*, const Properties&) = setEmbeddedProperties;
  constexpr static const Properties& (*GetEmbeddedProperties)(const Derived*) = getEmbeddedProperties;

  EmbeddedPropertiesAspect(const EmbeddedPropertiesAspect&) = delete;

  virtual ~EmbeddedPropertiesAspect() = default;

  /// Used to identify constructor arguments that can be used as Properties
  template <typename T>
  struct ConvertIfProperties
  {
    using type = typename std::conditional<
        std::is_base_of<PropertiesData, T>::value,
        PropertiesData, T>::type;
  };

  /// Construct this Aspect without affecting the Properties.
  EmbeddedPropertiesAspect()
    : Base()
  {
    // Do nothing
  }

  /// Construct this Aspect. If the first argument contains PropertiesData, then
  /// it will be used by this Aspect. Otherwise, all arguments will be forwarded
  /// to the Base class.
  //
  // Dev Note: The complex construction pattern used here allows us to satisfy
  // three simultaneous design constraints:
  //   1. We can identify when the user has passed in relevant Properties
  //      information and capture that information. The type can be of **any**
  //      class type that inherits PropertiesData.
  //   2. We can ignore any non-Properties information that the user has passed
  //      in, and move that information along to the Base class.
  //   3. We can handle arbitrary numbers of arguments of any type to pass along
  //      to the Base class.
  // If anyone can come up with a cleaner way of accomplishing all three of
  // these constraints, I would gladly replace this implementation. -(MXG)
  template <typename T, typename... RemainingArgs>
  EmbeddedPropertiesAspect(
      const T& arg1,
      RemainingArgs&&... remainingArgs)
    : EmbeddedPropertiesAspect(
        Delegate,
        static_cast<const typename ConvertIfProperties<T>::type&>(arg1),
        std::forward<RemainingArgs>(remainingArgs)...)
  {
    // Do nothing
  }

  // Documentation inherited
  void setAspectProperties(const Aspect::Properties& properties) override final
  {
    setProperties(static_cast<const Properties&>(properties));
  }

  // Documentation inherited
  void setProperties(const Properties& properties)
  {
    if(this->hasComposite())
    {
      SetEmbeddedProperties(static_cast<Derived*>(this), properties);
      return;
    }

    // If the correct type of Composite is not available, we store this on the
    // heap until this Aspect is moved.
    mTemporaryProperties = make_unique<Properties>(properties);
  }

  // Documentation inherited
  const Aspect::Properties* getAspectProperties() const override final
  {
    return &getProperties();
  }

  // Documentation inherited
  const Properties& getProperties() const
  {
    if(this->hasComposite())
    {
      return GetEmbeddedProperties(static_cast<const Derived*>(this));
    }

    if(!mTemporaryProperties)
    {
      dterr << "[detail::EmbeddedPropertiesAspect::getProperties] This Aspect "
            << "is not in a Composite, but it also does not have temporary "
            << "Properties available. This should not happen! Please report "
            << "this as a bug!\n";
      assert(false);
    }

    return *mTemporaryProperties;
  }

  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<Derived>(this->getProperties());
  }

protected:

  /// Construct this Aspect using the PropertiesData, and pass the remaining
  /// arguments into the constructor of the Base class.
  template <typename... RemainingArgs>
  EmbeddedPropertiesAspect(
      DelegateTag, const PropertiesData& properties,
      RemainingArgs&&... remainingArgs)
    : Base(std::forward<RemainingArgs>(remainingArgs)...),
      mTemporaryProperties(make_unique<Properties>(properties))
  {
    // Do nothing
  }

  /// Construct this Aspect without affecting the Properties, and pass all the
  /// arguments into the constructor of the Base class.
  template <typename... BaseArgs>
  EmbeddedPropertiesAspect(
      DelegateTag, BaseArgs&&... args)
    : Base(std::forward<BaseArgs>(args)...)
  {
    // Do nothing
  }

  /// Pass the temporary Properties of this Aspect into the new Composite
  void setComposite(Composite* newComposite) override
  {
    assert(nullptr == this->getComposite());

    Base::setComposite(newComposite);
    if(mTemporaryProperties)
      SetEmbeddedProperties(static_cast<Derived*>(this), *mTemporaryProperties);

    // Release the temporary memory
    mTemporaryProperties = nullptr;
  }

  /// Save the embedded Properties of this Composite before we remove the Aspect
  void loseComposite(Composite* oldComposite) override
  {
    mTemporaryProperties = make_unique<Properties>(
          GetEmbeddedProperties(static_cast<Derived*>(this)));
    Base::loseComposite(oldComposite);
  }

  /// After this Aspect is constructed and during transitions between Composite
  /// objects, this will hold the Properties of the Aspect. Once the Aspect has
  /// been moved into a new Composite, these Properties will be pushed into the
  /// Composite and cleared.
  std::unique_ptr<Properties> mTemporaryProperties;

};


//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <class BaseT, class DerivedT, typename StateDataT,
          typename StateT,
          void (*setEmbeddedState)(DerivedT*, const StateT&),
          const StateT& (*getEmbeddedState)(const DerivedT*)>
constexpr void (*EmbeddedStateAspect<
    BaseT, DerivedT, StateDataT, StateT, setEmbeddedState,
    getEmbeddedState>::SetEmbeddedState)(DerivedT*, const StateT&);

//==============================================================================
template <class BaseT, class DerivedT, typename StateDataT,
          typename StateT,
          void (*setEmbeddedState)(DerivedT*, const StateT&),
          const StateT& (*getEmbeddedState)(const DerivedT*)>
constexpr const StateT& (*EmbeddedStateAspect<
    BaseT, DerivedT, StateDataT, StateT, setEmbeddedState,
    getEmbeddedState>::GetEmbeddedState)(const DerivedT*);

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesDataT,
          typename PropertiesT,
          void (*setEmbeddedProperties)(DerivedT*, const PropertiesT&),
          const PropertiesT& (*getEmbeddedProperties)(const DerivedT*)>
constexpr void (*EmbeddedPropertiesAspect<
    BaseT, DerivedT, PropertiesDataT, PropertiesT, setEmbeddedProperties,
    getEmbeddedProperties>::SetEmbeddedProperties)(DerivedT*, const PropertiesT&);

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesDataT,
          typename PropertiesT,
          void (*setEmbeddedProperties)(DerivedT*, const PropertiesT&),
          const PropertiesT& (*getEmbeddedProperties)(const DerivedT*)>
constexpr const PropertiesT& (*EmbeddedPropertiesAspect<
    BaseT, DerivedT, PropertiesDataT, PropertiesT, setEmbeddedProperties,
    getEmbeddedProperties>::GetEmbeddedProperties)(const DerivedT*);

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_EMBEDDEDASPECT_HPP_
