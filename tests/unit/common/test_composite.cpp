/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "../../helpers/gtest_utils.hpp"
#include "dart/common/aspect.hpp"
#include "dart/common/composite.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::common;

namespace {

class TrackingComposite : public Composite
{
};

struct StatefulAspect : public CompositeTrackingAspect<TrackingComposite>
{
  using Base = CompositeTrackingAspect<TrackingComposite>;

  struct StateData
  {
    double value = 0.0;
    int visits = 0;
  };
  struct PropertiesData
  {
    double stiffness = 0.0;
  };

  using State = Aspect::MakeState<StateData>;
  using Properties = Aspect::MakeProperties<PropertiesData>;

  StatefulAspect()
  {
    mState.value = 0.0;
    mState.visits = 0;
    mProperties.stiffness = 0.0;
  }

  StatefulAspect(double stateValue, double stiffness)
  {
    mState.value = stateValue;
    mState.visits = 1;
    mProperties.stiffness = stiffness;
  }

  std::unique_ptr<Aspect> cloneAspect() const override
  {
    auto clone = std::make_unique<StatefulAspect>();
    clone->mState = mState;
    clone->mProperties = mProperties;
    clone->mSetCompositeCount = mSetCompositeCount;
    clone->mLoseCompositeCount = mLoseCompositeCount;
    return clone;
  }

  void setAspectState(const Aspect::State& other) override
  {
    mState = static_cast<const State&>(other);
  }

  const Aspect::State* getAspectState() const override
  {
    return &mState;
  }

  void setAspectProperties(const Aspect::Properties& properties) override
  {
    mProperties = static_cast<const Properties&>(properties);
  }

  const Aspect::Properties* getAspectProperties() const override
  {
    return &mProperties;
  }

  void setComposite(Composite* newComposite) override
  {
    ++mSetCompositeCount;
    Base::setComposite(newComposite);
  }

  void loseComposite(Composite* oldComposite) override
  {
    ++mLoseCompositeCount;
    Base::loseComposite(oldComposite);
  }

  State mState;
  Properties mProperties;
  int mSetCompositeCount = 0;
  int mLoseCompositeCount = 0;
};

} // namespace

//==============================================================================
TEST(CompositeTests, StateAndPropertiesRoundTrip)
{
  TrackingComposite composite;
  auto* aspect = composite.createAspect<StatefulAspect>();
  ASSERT_NE(aspect, nullptr);

  aspect->mState.value = 3.14;
  aspect->mState.visits = 2;
  aspect->mProperties.stiffness = 42.0;

  auto state = composite.getCompositeState();
  auto* stateCopy = state.get<StatefulAspect>();
  ASSERT_NE(stateCopy, nullptr);
  EXPECT_DOUBLE_EQ(stateCopy->value, 3.14);
  EXPECT_EQ(stateCopy->visits, 2);

  stateCopy->value = -1.0;
  stateCopy->visits = 7;
  composite.setCompositeState(state);
  EXPECT_DOUBLE_EQ(aspect->mState.value, -1.0);
  EXPECT_EQ(aspect->mState.visits, 7);

  auto properties = composite.getCompositeProperties();
  auto* propertiesCopy = properties.get<StatefulAspect>();
  ASSERT_NE(propertiesCopy, nullptr);
  EXPECT_DOUBLE_EQ(propertiesCopy->stiffness, 42.0);

  propertiesCopy->stiffness = 0.5;
  composite.setCompositeProperties(properties);
  EXPECT_DOUBLE_EQ(aspect->mProperties.stiffness, 0.5);
}

//==============================================================================
TEST(CompositeTests, DuplicateAspectsCloneState)
{
  TrackingComposite source;
  auto* aspect = source.createAspect<StatefulAspect>(5.0, 10.0);
  ASSERT_NE(aspect, nullptr);

  TrackingComposite destination;
  destination.duplicateAspects(&source);

  auto* cloned = destination.get<StatefulAspect>();
  ASSERT_NE(cloned, nullptr);
  EXPECT_NE(cloned, aspect);
  EXPECT_DOUBLE_EQ(cloned->mState.value, 5.0);
  EXPECT_DOUBLE_EQ(cloned->mProperties.stiffness, 10.0);

  cloned->mState.value = 1.0;
  EXPECT_DOUBLE_EQ(aspect->mState.value, 5.0);
}

//==============================================================================
TEST(CompositeTests, ReleaseAndMovePreservesAspectLifecycle)
{
  TrackingComposite first;
  auto* aspect = first.createAspect<StatefulAspect>();
  ASSERT_NE(aspect, nullptr);
  EXPECT_EQ(aspect->mSetCompositeCount, 1);

  auto released = first.releaseAspect<StatefulAspect>();
  ASSERT_NE(released, nullptr);
  EXPECT_EQ(released->mLoseCompositeCount, 1);
  EXPECT_FALSE(released->hasComposite());
  EXPECT_EQ(first.get<StatefulAspect>(), nullptr);

  TrackingComposite second;
  second.set(std::move(released));
  auto* moved = second.get<StatefulAspect>();
  ASSERT_NE(moved, nullptr);
  EXPECT_EQ(moved->mSetCompositeCount, 2);
  EXPECT_EQ(moved->getComposite(), &second);

  auto detachedAgain = second.releaseAspect<StatefulAspect>();
  ASSERT_NE(detachedAgain, nullptr);
  EXPECT_EQ(detachedAgain->mLoseCompositeCount, 2);
  EXPECT_FALSE(detachedAgain->hasComposite());
  EXPECT_EQ(second.get<StatefulAspect>(), nullptr);
}

//==============================================================================
TEST(CompositeTests, CloneableMapCopyHandlesNullEntries)
{
  Composite::State source;
  source.getMap().emplace(typeid(StatefulAspect), nullptr);
  EXPECT_EQ(source.getMap().size(), 1u);

  Composite::State destination;
  destination.copy(source);

  EXPECT_EQ(destination.getMap().size(), 1u);
  EXPECT_EQ(destination.get<StatefulAspect>(), nullptr);

  Composite::State nonEmpty;
  auto& nonEmptyState = nonEmpty.getOrCreate<StatefulAspect>();
  nonEmptyState.value = 2.0;
  nonEmptyState.visits = 3;

  Composite::State destinationNonEmpty;
  destinationNonEmpty.copy(nonEmpty);
  auto* copiedState = destinationNonEmpty.get<StatefulAspect>();
  ASSERT_NE(copiedState, nullptr);
  EXPECT_DOUBLE_EQ(copiedState->value, 2.0);
  EXPECT_EQ(copiedState->visits, 3);
}

//==============================================================================
TEST(CompositeTests, HasReturnsFalseForMissingAspect)
{
  TrackingComposite composite;

  // has<T>() should return false when aspect has not been added
  EXPECT_FALSE(composite.has<StatefulAspect>());
  EXPECT_EQ(composite.get<StatefulAspect>(), nullptr);
}

//==============================================================================
TEST(CompositeTests, HasReturnsTrueForPresentAspect)
{
  TrackingComposite composite;

  // Initially no aspect
  EXPECT_FALSE(composite.has<StatefulAspect>());

  // Create the aspect
  auto* aspect = composite.createAspect<StatefulAspect>();
  ASSERT_NE(aspect, nullptr);

  // has<T>() should now return true
  EXPECT_TRUE(composite.has<StatefulAspect>());
  EXPECT_NE(composite.get<StatefulAspect>(), nullptr);
}

//==============================================================================
TEST(CompositeTests, RemoveAspectDeletesAspect)
{
  TrackingComposite composite;

  // Create the aspect
  auto* aspect = composite.createAspect<StatefulAspect>();
  ASSERT_NE(aspect, nullptr);
  EXPECT_TRUE(composite.has<StatefulAspect>());

  // Remove the aspect
  composite.removeAspect<StatefulAspect>();

  // has<T>() should now return false
  EXPECT_FALSE(composite.has<StatefulAspect>());
  EXPECT_EQ(composite.get<StatefulAspect>(), nullptr);
}

//==============================================================================
TEST(CompositeTests, GetConstReturnsCorrectPointer)
{
  TrackingComposite composite;
  auto* aspect = composite.createAspect<StatefulAspect>(1.5, 2.5);
  ASSERT_NE(aspect, nullptr);

  // Get const reference to composite
  const TrackingComposite& constComposite = composite;

  // const get<T>() should return same pointer as non-const get<T>()
  const StatefulAspect* constAspect = constComposite.get<StatefulAspect>();
  ASSERT_NE(constAspect, nullptr);
  EXPECT_EQ(constAspect, aspect);

  // Verify we can read values through const pointer
  EXPECT_DOUBLE_EQ(constAspect->mState.value, 1.5);
  EXPECT_DOUBLE_EQ(constAspect->mProperties.stiffness, 2.5);
}

//==============================================================================
TEST(CompositeTests, IsSpecializedForReturnsFalseOnBaseComposite)
{
  // isSpecializedFor<T>() should always return false for the base Composite
  // class (it's a static constexpr function)
  EXPECT_FALSE(Composite::isSpecializedFor<StatefulAspect>());
  EXPECT_FALSE(TrackingComposite::isSpecializedFor<StatefulAspect>());

  // Verify at instance level as well (even though it's static)
  TrackingComposite composite;
  (void)composite; // Silence unused variable warning
  EXPECT_FALSE(TrackingComposite::isSpecializedFor<StatefulAspect>());
}
