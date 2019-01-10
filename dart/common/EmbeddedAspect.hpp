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

#ifndef DART_COMMON_EMBEDDEDASPECT_HPP_
#define DART_COMMON_EMBEDDEDASPECT_HPP_

#include "dart/common/detail/EmbeddedAspect.hpp"
#include "dart/common/RequiresAspect.hpp"
#include "dart/common/CompositeJoiner.hpp"

namespace dart {
namespace common {

//==============================================================================
/// This is the implementation of a standard embedded-state Aspect. Inherit
/// the EmbedState class (next class down in the header) to use this Aspect
/// implementation.
///
/// For more control over how your embedded-state Aspect is implemented, you can
/// use the detail::EmbeddedStateAspect class.
template <class CompositeT, typename StateDataT>
class EmbeddedStateAspect : public detail::EmbeddedStateAspect<
    CompositeTrackingAspect<CompositeT>,
    EmbeddedStateAspect<CompositeT, StateDataT>, StateDataT>
{
public:

  using Base = CompositeTrackingAspect<CompositeT>;
  using Derived = EmbeddedStateAspect<CompositeT, StateDataT>;
  using Impl = detail::EmbeddedStateAspect<Base, Derived, StateDataT>;
  using State = typename Impl::State;
  using StateData = typename Impl::StateData;

  template <typename... Args>
  EmbeddedStateAspect(Args&&... args)
    : Impl(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbeddedStateAspect() = default;

};

//==============================================================================
/// Inherit this class to embed a State into your Composite object. DerivedT is
/// the name of your class and StateDataT is a "plain-old data" structure that
/// holds your state information.
///
/// Your derived class must implement the following functions:
///
/// \code{.cpp}
/// void setAspectState(const AspectState& state);
/// \endcode
///
/// To embed both state and properties information, use EmbedStateAndProperties.
///
/// It is possible to customize the way an EmbeddedStateAspect interacts with
/// your Composite by using the dart::common::detail::EmbeddedStateAspect class
/// directly instead of inheriting this class.
template <class DerivedT, typename StateDataT>
class EmbedState : public virtual common::RequiresAspect<
    common::EmbeddedStateAspect<DerivedT, StateDataT> >
{
public:

  using Derived = DerivedT;
  using Aspect = common::EmbeddedStateAspect<Derived, StateDataT>;
  using AspectState = typename Aspect::State;
  using AspectStateData = typename Aspect::StateData;
  using Base = common::RequiresAspect<Aspect>;

  // Forwarding constructor
  template <typename... Args>
  EmbedState(Args&&... args)
    : Base(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbedState() = default;

  const AspectState& getAspectState() const
  {
    return mAspectState;
  }

protected:

  /// Aspect::State data, directly accessible to your derived class
  AspectState mAspectState;

};

//==============================================================================
/// This is an alternative to EmbedState which allows your class to also inherit
/// other Composite objects by listing them as the third (and later) template
/// arguments.
template <class DerivedT, typename StateDataT, typename... BaseComposites>
class EmbedStateOnTopOf : public CompositeJoiner<
    EmbedState<DerivedT, StateDataT>, BaseComposites...>
{
public:

  using Impl = EmbedState<DerivedT, StateDataT>;
  using Derived = typename Impl::Derived;
  using AspectStateData = typename Impl::AspectStateData;
  using AspectState = typename Impl::AspectState;
  using Aspect = typename Impl::Aspect;
  using Base = CompositeJoiner<Impl, BaseComposites...>;
  using Impl::getAspectState;

  // Forwarding constructor
  template <typename... Args>
  EmbedStateOnTopOf(Args&&... args)
    : Base(NoArg, std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbedStateOnTopOf() = default;

protected:

  using Impl::mAspectState;

};

//==============================================================================
/// This is the implementation of a standard embedded-properties Aspect. Inherit
/// the EmbedProperties (next class down in the header) to use this Aspect
/// implementation.
///
/// For more control over how your embedded-properties Aspect is implemented,
/// you can use the detail::EmbeddedPropertiesAspect class.
template <class CompositeT, typename PropertiesDataT>
class EmbeddedPropertiesAspect : public detail::EmbeddedPropertiesAspect<
    CompositeTrackingAspect<CompositeT>,
    EmbeddedPropertiesAspect<CompositeT, PropertiesDataT>, PropertiesDataT>
{
public:

  using Base = CompositeTrackingAspect<CompositeT>;
  using Derived = EmbeddedPropertiesAspect<CompositeT, PropertiesDataT>;
  using Impl = detail::EmbeddedPropertiesAspect<Base, Derived, PropertiesDataT>;
  using Properties = typename Impl::Properties;
  using PropertiesData = typename Impl::PropertiesData;

  // Forwarding constructor
  template <typename... Args>
  EmbeddedPropertiesAspect(Args&&... args)
    : Impl(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbeddedPropertiesAspect() = default;

};

//==============================================================================
/// Inherit this class to embed Properties into your Composite object. DerivedT
/// is the name of your class and PropertiesDataT is a "plain-old data"
/// structure that holds your properties information.
///
/// Your derived class must implement the following function:
///
/// \code{.cpp}
/// void setAspectProperties(const AspectProperties& state);
/// \endcode
///
/// To embed both state and properties information, use EmbedStateAndProperties.
///
/// It is possible to customize the way an EmbeddedPropertiesAspect interacts
/// with your Composite by using the
/// dart::common::detail::EmbeddedPropertiesAspect class directly instead of
/// inheriting this class.
template <class DerivedT, typename PropertiesDataT>
class EmbedProperties : public virtual common::RequiresAspect<
    common::EmbeddedPropertiesAspect<DerivedT, PropertiesDataT> >
{
public:

  using Derived = DerivedT;
  using Aspect = common::EmbeddedPropertiesAspect<Derived, PropertiesDataT>;
  using AspectProperties = typename Aspect::Properties;
  using AspectPropertiesData = typename Aspect::PropertiesData;
  using Base = common::RequiresAspect<Aspect>;

  // Forwarding constructor
  template <typename... Args>
  EmbedProperties(Args&&... args)
    : Base(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbedProperties() = default;

  const AspectProperties& getAspectProperties() const
  {
    return mAspectProperties;
  }

protected:

  /// Aspect::Properties data, directly accessible to your derived class
  AspectProperties mAspectProperties;

};

//==============================================================================
/// This is an alternative to EmbedProperties which allows your class to also
/// inherit other Composite objects by listing them as the third (and later)
/// template arguments.
template <class DerivedT, typename PropertiesDataT, typename... CompositeBases>
class EmbedPropertiesOnTopOf : public CompositeJoiner<
    EmbedProperties<DerivedT, PropertiesDataT>, CompositeBases...>
{
public:

  using Impl = EmbedProperties<DerivedT, PropertiesDataT>;
  using Derived = typename Impl::Derived;
  using AspectPropertiesData = typename Impl::AspectPropertiesData;
  using AspectProperties = typename Impl::AspectProperties;
  using Aspect = typename Impl::Aspect;
  using Base = CompositeJoiner<Impl, CompositeBases...>;
  using Impl::getAspectProperties;

  // Forwarding constructor
  template <typename... Args>
  EmbedPropertiesOnTopOf(Args&&... args)
    : Base(NoArg, std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbedPropertiesOnTopOf() = default;

protected:

  using Impl::mAspectProperties;

};

//==============================================================================
/// This is the implementation of a standard combination of embedded-state and
/// embedded-properties Aspect. Inherit the EmbedStateAndProperties (next class
/// down in the header) to use this Aspect implementation.
//
// Dev Note: We achieve "multiple inheritance" without the diamond of death
// issue by specifying detail::EmbeddedStateAspect as the base class of
// detail::EmbeddedPropertiesAspect. This allows their implementations to stack
// on top of each other without the conflict that would arise from both of them
// inheriting from common::Aspect.
template <class CompositeT, typename StateDataT, typename PropertiesDataT>
class EmbeddedStateAndPropertiesAspect :
    public detail::EmbeddedPropertiesAspect<
        detail::EmbeddedStateAspect<
            CompositeTrackingAspect<CompositeT>,
            EmbeddedStateAndPropertiesAspect<CompositeT, StateDataT, PropertiesDataT>,
            StateDataT>,
        EmbeddedStateAndPropertiesAspect<CompositeT, StateDataT, PropertiesDataT>,
        PropertiesDataT>
{
public:

  using Derived = EmbeddedStateAndPropertiesAspect<CompositeT, StateDataT, PropertiesDataT>;

  using AspectStateImpl = detail::EmbeddedStateAspect<
      CompositeTrackingAspect<CompositeT>, Derived, StateDataT>;

  using AspectPropertiesImpl = detail::EmbeddedPropertiesAspect<
      AspectStateImpl, Derived, PropertiesDataT>;

  using AspectImpl = Derived;

  using State = typename AspectStateImpl::State;
  using StateData = typename AspectStateImpl::StateData;

  using Properties = typename AspectPropertiesImpl::Properties;
  using PropertiesData = typename AspectPropertiesImpl::PropertiesData;

  using CompositeType = CompositeT;

  EmbeddedStateAndPropertiesAspect(
      const EmbeddedStateAndPropertiesAspect&) = delete;

  virtual ~EmbeddedStateAndPropertiesAspect() = default;

  /// Construct using nothing. The object will remain unaffected.
  EmbeddedStateAndPropertiesAspect()
    : AspectPropertiesImpl()
  {
    // Do nothing
  }

  /// Construct using a State. The object's Properties will remain unaffected.
  EmbeddedStateAndPropertiesAspect(
      const StateData& state)
    : AspectPropertiesImpl(state)
  {
    // Do nothing
  }

  /// Construct using Properties. The object's State will remain unaffected.
  EmbeddedStateAndPropertiesAspect(
      const PropertiesData& properties)
    : AspectPropertiesImpl(properties)
  {
    // Do nothing
  }

  /// Construct using a State and Properties instance
  EmbeddedStateAndPropertiesAspect(
      const StateData& state,
      const PropertiesData& properties)
    : AspectPropertiesImpl(properties, state)
  {
    // Do nothing
  }

  /// Construct using a Properties and State instance
  EmbeddedStateAndPropertiesAspect(
      const PropertiesData& properties,
      const StateData& state)
    : AspectPropertiesImpl(properties, state)
  {
    // Do nothing
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return make_unique<Derived>(this->getState(), this->getProperties());
  }

};

//==============================================================================
/// Inherit this class to embed both State and Properties into your Composite
/// object. DerivedT is the name of your class, StateDataT is a "plain-old data"
/// structure that holds your state information, and PropertiesDataT is a
/// "plain-old data" structure that holds your properties information.
///
/// Your derived class must implement the following functions:
///
/// \code{.cpp}
/// void setAspectState(const AspectState& state);
/// void setAspectProperties(const AspectProperties& state);
/// \endcode
template <class DerivedT, typename StateDataT, typename PropertiesDataT>
class EmbedStateAndProperties : public virtual common::RequiresAspect<
    common::EmbeddedStateAndPropertiesAspect<
        DerivedT, StateDataT, PropertiesDataT> >
{
public:

  using Derived = DerivedT;
  using Aspect = common::EmbeddedStateAndPropertiesAspect<
      DerivedT, StateDataT, PropertiesDataT>;

  using AspectState = typename Aspect::State;
  using AspectStateData = typename Aspect::StateData;

  using AspectProperties = typename Aspect::Properties;
  using AspectPropertiesData = typename Aspect::PropertiesData;
  using Base = common::RequiresAspect<Aspect>;

  // Forwarding constructor
  template <typename... Args>
  EmbedStateAndProperties(Args&&... args)
    : Base(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbedStateAndProperties() = default;

  const AspectState& getAspectState() const
  {
    return mAspectState;
  }

  const AspectProperties& getAspectProperties() const
  {
    return mAspectProperties;
  }

protected:

  /// Aspect::State data, directly accessible to your derived class
  AspectState mAspectState;

  /// Aspect::Properties data, directly accessible to your derived class
  AspectProperties mAspectProperties;

};

//==============================================================================
/// This is an alternative to EmbedStateAndProperties which allows your class to
/// also inherit other Composite objects by listing them as the fourth (and
/// later) template arguments.
template <class DerivedT, typename StateDataT, typename PropertiesDataT,
          typename... CompositeBases>
class EmbedStateAndPropertiesOnTopOf : public CompositeJoiner<
    EmbedStateAndProperties<DerivedT, StateDataT, PropertiesDataT>,
    CompositeBases...>
{
public:

  using Impl = EmbedStateAndProperties<DerivedT, StateDataT, PropertiesDataT>;
  using Derived = typename Impl::Derived;
  using AspectStateData = typename Impl::AspectStateData;
  using AspectState = typename Impl::AspectState;
  using AspectPropertiesData = typename Impl::AspectPropertiesData;
  using AspectProperties = typename Impl::AspectProperties;
  using Aspect = typename Impl::Aspect;
  using Impl::getAspectState;
  using Impl::getAspectProperties;
  using Base = CompositeJoiner<Impl, CompositeBases...>;

  // Forwarding constructor
  template <typename... Args>
  EmbedStateAndPropertiesOnTopOf(Args&&... args)
    : Base(NoArg, std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~EmbedStateAndPropertiesOnTopOf() = default;

protected:

  using Impl::mAspectState;
  using Impl::mAspectProperties;

};

} // namespace common
} // namespace dart

#endif // DART_COMMON_EMBEDDEDASPECT_HPP_
