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

#include <dart/common/Aspect.hpp>
#include <dart/common/AspectWithVersion.hpp>
#include <dart/common/Composite.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::common;

namespace {

//==============================================================================
class VersionedComposite : public Composite
{
public:
  std::size_t incrementVersion()
  {
    return ++mVersion;
  }

  std::size_t getVersion() const
  {
    return mVersion;
  }

private:
  std::size_t mVersion = 0;
};

//==============================================================================
class MinimalAspect : public CompositeTrackingAspect<Composite>
{
public:
  std::unique_ptr<Aspect> cloneAspect() const override
  {
    return std::make_unique<MinimalAspect>();
  }
};

//==============================================================================
struct TestStateData
{
  double position = 0.0;
  int counter = 0;
};

class StateOnlyAspect final
  : public AspectWithState<StateOnlyAspect, TestStateData, Composite>
{
public:
  StateOnlyAspect(const TestStateData& state = TestStateData())
    : AspectWithState(state)
  {
  }
};

//==============================================================================
struct TestPropertiesData
{
  std::string name = "default";
  double stiffness = 1.0;
};

class PropertiesOnlyAspect final : public AspectWithVersionedProperties<
                                       PropertiesOnlyAspect,
                                       TestPropertiesData,
                                       VersionedComposite>
{
public:
  PropertiesOnlyAspect(const TestPropertiesData& props = TestPropertiesData())
    : AspectWithVersionedProperties(props)
  {
  }
};

} // namespace

//==============================================================================
TEST(Aspect, CompositeTrackingDefaultConstruction)
{
  MinimalAspect aspect;
  EXPECT_FALSE(aspect.hasComposite());
  EXPECT_EQ(aspect.getComposite(), nullptr);
}

TEST(Aspect, CompositeTrackingSetComposite)
{
  Composite composite;
  auto* aspect = composite.createAspect<MinimalAspect>();

  ASSERT_NE(aspect, nullptr);
  EXPECT_TRUE(aspect->hasComposite());
  EXPECT_EQ(aspect->getComposite(), &composite);
}

TEST(Aspect, CompositeTrackingLoseComposite)
{
  Composite composite;
  auto* aspect = composite.createAspect<MinimalAspect>();
  ASSERT_TRUE(aspect->hasComposite());

  auto released = composite.releaseAspect<MinimalAspect>();
  ASSERT_NE(released, nullptr);
  EXPECT_FALSE(released->hasComposite());
  EXPECT_EQ(released->getComposite(), nullptr);
}

TEST(Aspect, CompositeTrackingClone)
{
  Composite composite;
  composite.createAspect<MinimalAspect>();

  auto* original = composite.get<MinimalAspect>();
  auto cloned = original->cloneAspect();

  ASSERT_NE(cloned, nullptr);
  auto* clonedMinimal = dynamic_cast<MinimalAspect*>(cloned.get());
  ASSERT_NE(clonedMinimal, nullptr);
  EXPECT_FALSE(clonedMinimal->hasComposite());
}

//==============================================================================
TEST(Aspect, StateOnlyDefaultConstruction)
{
  StateOnlyAspect aspect;
  EXPECT_DOUBLE_EQ(aspect.getState().position, 0.0);
  EXPECT_EQ(aspect.getState().counter, 0);
}

TEST(Aspect, StateOnlyCustomConstruction)
{
  TestStateData customState;
  customState.position = 5.5;
  customState.counter = 10;

  StateOnlyAspect aspect(customState);
  EXPECT_DOUBLE_EQ(aspect.getState().position, 5.5);
  EXPECT_EQ(aspect.getState().counter, 10);
}

TEST(Aspect, StateOnlySetState)
{
  StateOnlyAspect aspect;

  TestStateData newState;
  newState.position = -3.14;
  newState.counter = 42;
  aspect.setState(newState);

  EXPECT_DOUBLE_EQ(aspect.getState().position, -3.14);
  EXPECT_EQ(aspect.getState().counter, 42);
}

TEST(Aspect, StateOnlyGetAspectState)
{
  TestStateData initialState;
  initialState.position = 1.0;
  StateOnlyAspect aspect(initialState);

  const Aspect::State* statePtr = aspect.getAspectState();
  ASSERT_NE(statePtr, nullptr);

  const auto* typedState
      = dynamic_cast<const StateOnlyAspect::State*>(statePtr);
  ASSERT_NE(typedState, nullptr);
  EXPECT_DOUBLE_EQ(typedState->position, 1.0);
}

TEST(Aspect, StateOnlySetAspectState)
{
  StateOnlyAspect aspect;

  StateOnlyAspect::State newState;
  newState.position = 99.9;
  newState.counter = 123;
  aspect.setAspectState(newState);

  EXPECT_DOUBLE_EQ(aspect.getState().position, 99.9);
  EXPECT_EQ(aspect.getState().counter, 123);
}

TEST(Aspect, StateOnlyClone)
{
  TestStateData state;
  state.position = 7.77;
  state.counter = 777;

  StateOnlyAspect original(state);
  auto cloned = original.cloneAspect();

  auto* clonedAspect = dynamic_cast<StateOnlyAspect*>(cloned.get());
  ASSERT_NE(clonedAspect, nullptr);
  EXPECT_DOUBLE_EQ(clonedAspect->getState().position, 7.77);
  EXPECT_EQ(clonedAspect->getState().counter, 777);
}

//==============================================================================
TEST(Aspect, PropertiesOnlyDefaultConstruction)
{
  PropertiesOnlyAspect aspect;
  EXPECT_EQ(aspect.getProperties().name, "default");
  EXPECT_DOUBLE_EQ(aspect.getProperties().stiffness, 1.0);
}

TEST(Aspect, PropertiesOnlyCustomConstruction)
{
  TestPropertiesData props;
  props.name = "custom";
  props.stiffness = 50.0;

  PropertiesOnlyAspect aspect(props);
  EXPECT_EQ(aspect.getProperties().name, "custom");
  EXPECT_DOUBLE_EQ(aspect.getProperties().stiffness, 50.0);
}

TEST(Aspect, PropertiesOnlySetProperties)
{
  PropertiesOnlyAspect aspect;

  TestPropertiesData newProps;
  newProps.name = "updated";
  newProps.stiffness = 25.5;
  aspect.setProperties(newProps);

  EXPECT_EQ(aspect.getProperties().name, "updated");
  EXPECT_DOUBLE_EQ(aspect.getProperties().stiffness, 25.5);
}

TEST(Aspect, PropertiesOnlyGetAspectProperties)
{
  TestPropertiesData props;
  props.name = "test";
  PropertiesOnlyAspect aspect(props);

  const Aspect::Properties* propsPtr = aspect.getAspectProperties();
  ASSERT_NE(propsPtr, nullptr);

  const auto* typedProps
      = dynamic_cast<const PropertiesOnlyAspect::Properties*>(propsPtr);
  ASSERT_NE(typedProps, nullptr);
  EXPECT_EQ(typedProps->name, "test");
}

TEST(Aspect, PropertiesOnlySetAspectProperties)
{
  PropertiesOnlyAspect aspect;

  PropertiesOnlyAspect::Properties newProps;
  newProps.name = "via_aspect";
  newProps.stiffness = 100.0;
  aspect.setAspectProperties(newProps);

  EXPECT_EQ(aspect.getProperties().name, "via_aspect");
  EXPECT_DOUBLE_EQ(aspect.getProperties().stiffness, 100.0);
}

TEST(Aspect, PropertiesOnlyClone)
{
  TestPropertiesData props;
  props.name = "cloneable";
  props.stiffness = 3.14;

  PropertiesOnlyAspect original(props);
  auto cloned = original.cloneAspect();

  auto* clonedAspect = dynamic_cast<PropertiesOnlyAspect*>(cloned.get());
  ASSERT_NE(clonedAspect, nullptr);
  EXPECT_EQ(clonedAspect->getProperties().name, "cloneable");
  EXPECT_DOUBLE_EQ(clonedAspect->getProperties().stiffness, 3.14);
}

TEST(Aspect, PropertiesOnlyIncrementVersion)
{
  VersionedComposite composite;
  auto* aspect = composite.createAspect<PropertiesOnlyAspect>();

  EXPECT_EQ(composite.getVersion(), 0u);

  std::size_t newVersion = aspect->incrementVersion();
  EXPECT_EQ(newVersion, 1u);
  EXPECT_EQ(composite.getVersion(), 1u);

  aspect->incrementVersion();
  EXPECT_EQ(composite.getVersion(), 2u);
}

TEST(Aspect, PropertiesOnlyIncrementVersionWithoutComposite)
{
  PropertiesOnlyAspect aspect;
  std::size_t version = aspect.incrementVersion();
  EXPECT_EQ(version, 0u);
}

//==============================================================================
TEST(Aspect, BaseStateIsStateless)
{
  MinimalAspect aspect;
  const Aspect::State* state = aspect.getAspectState();
  EXPECT_EQ(state, nullptr);
}

TEST(Aspect, BasePropertiesAreEmpty)
{
  MinimalAspect aspect;
  const Aspect::Properties* props = aspect.getAspectProperties();
  EXPECT_EQ(props, nullptr);
}

//==============================================================================
TEST(Aspect, MakeStateCloneable)
{
  using TestState = Aspect::MakeState<TestStateData>;

  TestState state;
  state.position = 123.0;
  state.counter = 456;

  auto cloned = state.clone();
  ASSERT_NE(cloned, nullptr);

  auto* typedClone = dynamic_cast<TestState*>(cloned.get());
  ASSERT_NE(typedClone, nullptr);
  EXPECT_DOUBLE_EQ(typedClone->position, 123.0);
  EXPECT_EQ(typedClone->counter, 456);
}

TEST(Aspect, MakePropertiesCloneable)
{
  using TestProps = Aspect::MakeProperties<TestPropertiesData>;

  TestProps props;
  props.name = "cloneable_props";
  props.stiffness = 999.0;

  auto cloned = props.clone();
  ASSERT_NE(cloned, nullptr);

  auto* typedClone = dynamic_cast<TestProps*>(cloned.get());
  ASSERT_NE(typedClone, nullptr);
  EXPECT_EQ(typedClone->name, "cloneable_props");
  EXPECT_DOUBLE_EQ(typedClone->stiffness, 999.0);
}

TEST(Aspect, MakeStateCopyAssignment)
{
  using TestState = Aspect::MakeState<TestStateData>;

  TestState state1;
  state1.position = 1.0;
  state1.counter = 1;

  TestState state2;
  state2.position = 2.0;
  state2.counter = 2;

  state1.copy(state2);
  EXPECT_DOUBLE_EQ(state1.position, 2.0);
  EXPECT_EQ(state1.counter, 2);
}

TEST(Aspect, MakePropertiesCopyAssignment)
{
  using TestProps = Aspect::MakeProperties<TestPropertiesData>;

  TestProps props1;
  props1.name = "first";
  props1.stiffness = 1.0;

  TestProps props2;
  props2.name = "second";
  props2.stiffness = 2.0;

  props1.copy(props2);
  EXPECT_EQ(props1.name, "second");
  EXPECT_DOUBLE_EQ(props1.stiffness, 2.0);
}
