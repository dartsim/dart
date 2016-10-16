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

#ifndef DART_COMMON_DETAIL_ASPECTWITHVERSION_HPP_
#define DART_COMMON_DETAIL_ASPECTWITHVERSION_HPP_

#include "dart/common/Aspect.hpp"
#include "dart/common/StlHelpers.hpp"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
/// AspectWithProtectedState generates implementations of the State managing
/// functions for an Aspect class.
template <class BaseT, class DerivedT, typename StateDataT,
          class CompositeT = Composite,
          void (*updateState)(DerivedT*) = &NoOp<DerivedT*> >
class AspectWithState : public BaseT
{
public:

  using Base = BaseT;
  using Derived = DerivedT;
  using StateData = StateDataT;
  using CompositeType = CompositeT;
  using State = Aspect::MakeState<StateData>;
  constexpr static void (*UpdateState)(Derived*) = updateState;

  using AspectImplementation = AspectWithState<
      Base, Derived, StateData, CompositeT, updateState>;

  AspectWithState(const AspectWithState&) = delete;

  /// Construct using a StateData instance
  AspectWithState(const StateData& state = StateData());

  /// Construct this Aspect and pass args into the constructor of the Base class
  template <typename... BaseArgs>
  AspectWithState(const StateData& state,
                 BaseArgs&&... args)
    : Base(std::forward<BaseArgs>(args)...),
      mState(state)
  {
    // Do nothing
  }

  // Documentation inherited
  void setAspectState(const Aspect::State& otherState) override final;

  // Documentation inherited
  const Aspect::State* getAspectState() const override final;

  /// Set the State of this Aspect
  void setState(const StateData& state);

  /// Get the State of this Aspect
  const State& getState() const;

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override;

protected:

  /// State of this Aspect
  State mState;
};

//==============================================================================
/// AspectWithProtectedProperties generates implementations of the Property
/// managing functions for an Aspect class.
template <class BaseT, class DerivedT, typename PropertiesDataT,
          class CompositeT = Composite,
          void (*updateProperties)(DerivedT*) = &NoOp<DerivedT*> >
class AspectWithVersionedProperties : public BaseT
{
public:

  using Base = BaseT;
  using Derived = DerivedT;
  using PropertiesData = PropertiesDataT;
  using CompositeType = CompositeT;
  using Properties = Aspect::MakeProperties<PropertiesData>;
  constexpr static void (*UpdateProperties)(Derived*) = updateProperties;

  using AspectImplementation = AspectWithVersionedProperties<
      Base, Derived, PropertiesData, CompositeT, updateProperties>;

  AspectWithVersionedProperties() = delete;
  AspectWithVersionedProperties(const AspectWithVersionedProperties&) = delete;

  /// Construct using a PropertiesData instance
  AspectWithVersionedProperties(
      const PropertiesData& properties = PropertiesData());

  /// Construct this Aspect and pass args into the constructor of the Base class
  template <typename... BaseArgs>
  AspectWithVersionedProperties(
      const PropertiesData& properties, BaseArgs&&... args)
    : Base(std::forward<BaseArgs>(args)...),
      mProperties(properties)
  {
    // Do nothing
  }

  // Documentation inherited
  void setAspectProperties(const Aspect::Properties& someProperties) override final;

  // Documentation inherited
  const Aspect::Properties* getAspectProperties() const override final;

  /// Set the Properties of this Aspect
  void setProperties(const PropertiesData& properties);

  /// Get the Properties of this Aspect
  const Properties& getProperties() const;

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override;

  /// Increment the version of this Aspect and its Composite
  std::size_t incrementVersion();

  /// Call UpdateProperties(this) and incrementVersion()
  void notifyPropertiesUpdate();

protected:

  /// Properties of this Aspect
  Properties mProperties;

};

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <class BaseT, class DerivedT, typename StateDataT,
          class CompositeT, void (*updateState)(DerivedT*)>
constexpr void (*AspectWithState<
    BaseT, DerivedT, StateDataT, CompositeT, updateState>::UpdateState)(
    DerivedT*);

//==============================================================================
template <class BaseT, class DerivedT, typename StateDataT,
          class CompositeT, void (*updateState)(DerivedT*)>
AspectWithState<BaseT, DerivedT, StateDataT, CompositeT, updateState>::
AspectWithState(const StateDataT& state)
  : BaseT(),
    mState(state)
{
  // Do nothing
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class CompositeT, void (*updateState)(DerivedT*)>
void AspectWithState<BaseT, DerivedT, StateData, CompositeT, updateState>::
setAspectState(const Aspect::State& otherState)
{
  setState(static_cast<const State&>(otherState));
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class CompositeT, void (*updateState)(DerivedT*)>
const Aspect::State*
AspectWithState<BaseT, DerivedT, StateData, CompositeT, updateState>::
getAspectState() const
{
  return &mState;
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class CompositeT, void (*updateState)(DerivedT*)>
void AspectWithState<BaseT, DerivedT, StateData, CompositeT, updateState>::
setState(const StateData& state)
{
  static_cast<StateData&>(mState) = state;
  UpdateState(static_cast<Derived*>(this));
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateDataT,
          class CompositeT, void (*updateState)(DerivedT*)>
auto AspectWithState<BaseT, DerivedT, StateDataT, CompositeT, updateState>::
getState() const -> const State&
{
  return mState;
}

//==============================================================================
template <class BaseT, class DerivedT, typename StateData,
          class CompositeT, void (*updateState)(DerivedT*)>
std::unique_ptr<Aspect>
AspectWithState<BaseT, DerivedT, StateData, CompositeT, updateState>::
    cloneAspect() const
{
  return common::make_unique<Derived>(mState);
}

//==============================================================================
//
// These namespace-level definitions are required to enable ODR-use of static
// constexpr member variables.
//
// See this StackOverflow answer: http://stackoverflow.com/a/14396189/111426
//
template <class BaseT, class DerivedT, typename PropertiesDataT,
          class CompositeT, void (*updateProperties)(DerivedT*)>
constexpr void (*AspectWithVersionedProperties<BaseT, DerivedT, PropertiesDataT,
                                              CompositeT, updateProperties>::
UpdateProperties)(DerivedT*);

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesDataT,
          class CompositeT, void (*updateProperties)(DerivedT*)>
AspectWithVersionedProperties<BaseT, DerivedT, PropertiesDataT,
                             CompositeT, updateProperties>::
AspectWithVersionedProperties(const PropertiesData& properties)
  : BaseT(),
    mProperties(properties)
{
  // Do nothing
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
void AspectWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  CompositeT, updateProperties>::
setAspectProperties(const Aspect::Properties& someProperties)
{
  setProperties(static_cast<const Properties&>(someProperties));
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
const Aspect::Properties*
AspectWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             CompositeT, updateProperties>::
getAspectProperties() const
{
  return &mProperties;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
void AspectWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  CompositeT, updateProperties>::
setProperties(const PropertiesData& properties)
{
  static_cast<PropertiesData&>(mProperties) = properties;
  this->notifyPropertiesUpdate();
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
auto AspectWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                                  CompositeT, updateProperties>::
getProperties() const -> const Properties&
{
  return mProperties;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
std::unique_ptr<Aspect>
AspectWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             CompositeT, updateProperties>::
cloneAspect() const
{
  return common::make_unique<Derived>(mProperties);
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
std::size_t AspectWithVersionedProperties<BaseT, DerivedT, PropertiesData,
                             CompositeT, updateProperties>::incrementVersion()
{
  if(CompositeType* comp = this->getComposite())
    return comp->incrementVersion();

  return 0;
}

//==============================================================================
template <class BaseT, class DerivedT, typename PropertiesData,
          class CompositeT, void (*updateProperties)(DerivedT*)>
void AspectWithVersionedProperties<
    BaseT, DerivedT, PropertiesData,
    CompositeT, updateProperties>::notifyPropertiesUpdate()
{
  UpdateProperties(static_cast<Derived*>(this));
  this->incrementVersion();
}

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ASPECTWITHVERSION_HPP_
