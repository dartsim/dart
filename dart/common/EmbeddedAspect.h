/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_EMBEDDEDASPECT_H_
#define DART_COMMON_EMBEDDEDASPECT_H_

#include "dart/common/detail/EmbeddedAspect.h"
#include "dart/common/RequiresAspect.h"

namespace dart {
namespace common {

//==============================================================================
/// This is the implementation of a standard embedded-state Aspect. Inherit
/// the EmbedState class (next class down in the header) to use this Aspect
/// implementation.
///
/// For more control over how your embedded-state Aspect is implemented, you can
/// use the detail::EmbeddedStateAspect class.
template <class CompositeT, typename StateT>
class EmbeddedStateAspect : public detail::EmbeddedStateAspect<
    CompositeTrackingAspect<CompositeT>,
    EmbeddedStateAspect<CompositeT, StateT>, StateT>
{
  // Do nothing
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
template <class DerivedT, typename StateDataT>
class EmbedState : public virtual common::RequiresAspect<
    common::EmbeddedStateAspect<DerivedT, common::Aspect::StateMixer<StateDataT>> >
{
public:

  using Derived = DerivedT;
  using AspectStateData = StateDataT;
  using AspectState = common::Aspect::StateMixer<StateDataT>;
  using Aspect = common::EmbeddedStateAspect<Derived, AspectState>;

  const AspectState& getAspectState() const
  {
    return mAspectState;
  }

protected:

  /// Aspect::State data, directly accessible to your derived class
  AspectState mAspectState;

};

//==============================================================================
/// This is the implementation of a standard embedded-properties Aspect. Inherit
/// the EmbedProperties (next class down in the header) to use this Aspect
/// implementation.
///
/// For more control over how your embedded-properties Aspect is implemented,
/// you can use the detail::EmbeddedPropertiesAspect class.
template <class CompositeT, typename PropertiesT>
class EmbeddedPropertiesAspect : public detail::EmbeddedPropertiesAspect<
    CompositeTrackingAspect<CompositeT>,
    EmbeddedPropertiesAspect<CompositeT, PropertiesT>, PropertiesT>
{
  // Do nothing
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
template <class DerivedT, typename PropertiesDataT>
class EmbedProperties : public virtual common::RequiresAspect<
    common::EmbeddedPropertiesAspect<
        DerivedT, common::Aspect::PropertiesMixer<PropertiesDataT>> >
{
public:

  using Derived = DerivedT;
  using AspectPropertiesData = PropertiesDataT;
  using AspectProperties = common::Aspect::PropertiesMixer<PropertiesDataT>;
  using Aspect = common::EmbeddedPropertiesAspect<Derived, AspectProperties>;

  const AspectProperties& getAspectProperties() const
  {
    return mAspectProperties;
  }

protected:

  /// Aspect::Properties data, directly accessible to your derived class
  AspectProperties mAspectProperties;

};

//==============================================================================
/// This is the implementation of a standard combination of embedded-state and
/// embedded-properties Aspect. Inherit the EmbedStateAndProperties (next class
/// down in the header) to use this Aspect implementation.
template <class CompositeT, typename StateT, typename PropertiesT>
class EmbeddedStateAndPropertiesAspect :
    public detail::EmbeddedPropertiesAspect<
        detail::EmbeddedStateAspect<
            CompositeTrackingAspect<CompositeT>,
            EmbeddedStateAndPropertiesAspect<CompositeT, StateT, PropertiesT>,
            StateT>,
        EmbeddedStateAndPropertiesAspect<CompositeT, StateT, PropertiesT>,
        PropertiesT>
{
public:

  using Derived = EmbeddedStateAndPropertiesAspect;
  using State = StateT;
  using Properties = PropertiesT;
  using CompositeType = CompositeT;

  using AspectStateImpl = detail::EmbeddedStateAspect<
      CompositeTrackingAspect<CompositeT>, Derived, State>;

  using AspectPropertiesImpl = detail::EmbeddedPropertiesAspect<
      AspectStateImpl, Derived, Properties>;

  using AspectImpl = EmbeddedStateAndPropertiesAspect<
      CompositeType, State, Properties>;

  EmbeddedStateAndPropertiesAspect() = delete;
  EmbeddedStateAndPropertiesAspect(
      const EmbeddedStateAndPropertiesAspect&) = delete;

  /// Construct using a State and Properties instance
  EmbeddedStateAndPropertiesAspect(
      common::Composite* comp,
      const State& state = State(),
      const Properties& properties = Properties())
    : AspectPropertiesImpl(comp, properties, state)
  {
    // Do nothing
  }

  /// Construct using a Properties and State instance
  EmbeddedStateAndPropertiesAspect(
      common::Composite* comp,
      const Properties& properties,
      const State& state = State())
    : AspectPropertiesImpl(comp, properties, state)
  {
    // Do nothing
  }

  // Documentation inherited
  std::unique_ptr<Aspect> cloneAspect(Composite* newComposite) const override
  {
    return make_unique<Derived>(
          newComposite, this->getState(), this->getProperties());
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
        DerivedT,
        common::Aspect::StateMixer<StateDataT>,
        common::Aspect::PropertiesMixer<PropertiesDataT>> >
{
public:

  using Derived = DerivedT;
  using AspectStateData = StateDataT;
  using AspectState = common::Aspect::StateMixer<StateDataT>;
  using AspectPropertiesData = PropertiesDataT;
  using AspectProperties = common::Aspect::PropertiesMixer<PropertiesDataT>;
  using Aspect = common::EmbeddedStateAndPropertiesAspect<
      Derived, AspectState, AspectProperties>;

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

} // namespace common
} // namespace dart

#endif // DART_COMMON_EMBEDDEDASPECT_H_
