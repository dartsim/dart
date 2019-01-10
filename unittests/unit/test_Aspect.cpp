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

// Use this preprocessor token to allow us to test that the specialized versions
// of Aspect functions are being used appropriately. This preprocessor token
// should NOT be used anywhere outside of this file (testAspect.cpp).
#define DART_UNITTEST_SPECIALIZED_ASPECT_ACCESS

#include <gtest/gtest.h>

#include "TestHelpers.hpp"

#include "dart/common/Subject.hpp"
#include "dart/common/sub_ptr.hpp"
#include "dart/common/Composite.hpp"
#include "dart/common/SpecializedForAspect.hpp"
#include "dart/common/EmbeddedAspect.hpp"

#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/BoxShape.hpp"

using namespace dart::common;

struct EmbeddedStateData
{
  double d;
  int i;

  EmbeddedStateData() : d(0.0), i(0)
  {
    // Do nothing
  }

  bool operator==(const EmbeddedStateData& other) const
  {
    if(other.d != d)
      return false;

    if(other.i != i)
      return false;

    return true;
  }

  bool operator!=(const EmbeddedStateData& other) const
  {
    return !(*this == other);
  }
};

struct SecondEmbeddedStateData { };

struct EmbeddedPropertiesData
{
  bool b;
  float f;

  EmbeddedPropertiesData() : b(false), f(0.0)
  {
    // Do nothing
  }

  bool operator==(const EmbeddedPropertiesData& other) const
  {
    if(other.b != b)
      return false;

    if(other.f != f)
      return false;

    return true;
  }

  bool operator!=(const EmbeddedPropertiesData& other) const
  {
    return !(*this == other);
  }
};

struct SecondEmbeddedPropertiesData { };

class EmbeddedStateComposite :
    public EmbedState<EmbeddedStateComposite, EmbeddedStateData>
{
public:

  EmbeddedStateComposite(const EmbeddedStateData& state = EmbeddedStateData())
  {
    createAspect<Aspect>(static_cast<const Aspect::StateData&>(state));
  }

  void setAspectState(const AspectState& state) { mAspectState = state; }

};

class EmbeddedPropertiesComposite :
    public EmbedProperties<EmbeddedPropertiesComposite, EmbeddedPropertiesData>
{
public:

  EmbeddedPropertiesComposite(
      const EmbeddedPropertiesData& properties = EmbeddedPropertiesData())
  {
    createAspect<Aspect>(properties);
  }

  void setAspectProperties(const AspectProperties& properties)
  {
    mAspectProperties = properties;
  }

};

class EmbeddedStateAndPropertiesComposite :
    public EmbedStateAndProperties<
        EmbeddedStateAndPropertiesComposite,
        EmbeddedStateData, EmbeddedPropertiesData>
{
public:

  EmbeddedStateAndPropertiesComposite()
  {
    createAspect<Aspect>();
  }

  void setAspectState(const AspectState& state) { mAspectState = state; }

  void setAspectProperties(const AspectProperties& properties)
  {
    mAspectProperties = properties;
  }

};

class InheritAndEmbedStateComposite :
    public EmbedStateOnTopOf<
        InheritAndEmbedStateComposite,
        SecondEmbeddedStateData,
        EmbeddedStateComposite>
{
public:

  InheritAndEmbedStateComposite()
  {
    createAspect<Aspect>();
  }

  void setAspectState(const AspectState& state) { mAspectState = state; }
};

class InheritAndEmbedPropertiesComposite :
    public EmbedPropertiesOnTopOf<
        InheritAndEmbedPropertiesComposite,
        SecondEmbeddedPropertiesData,
        EmbeddedPropertiesComposite>
{
public:

  InheritAndEmbedPropertiesComposite()
  {
    createAspect<Aspect>();
  }

  void setAspectProperties(const AspectProperties& properties)
  {
    mAspectProperties = properties;
  }
};

class InheritAndEmbedStateAndPropertiesComposite :
    public EmbedStateAndPropertiesOnTopOf<
        InheritAndEmbedStateAndPropertiesComposite,
        SecondEmbeddedStateData,
        SecondEmbeddedPropertiesData,
        EmbeddedStateAndPropertiesComposite>
{
public:

  InheritAndEmbedStateAndPropertiesComposite()
  {
    createAspect<Aspect>();
  }

  void setAspectState(const AspectState& state) { mAspectState = state; }

  void setAspectProperties(const AspectProperties& properties)
  {
    mAspectProperties = properties;
  }
};

// Testing the creation of an Aspect using the AspectWithState template class
class StateAspectTest : public dart::common::AspectWithState<
    StateAspectTest, dart::common::Empty>
{
public:

  StateAspectTest(const StateData& state = StateData())
    : dart::common::AspectWithState<StateAspectTest, dart::common::Empty>(state)
  {

  }

};

class StateAndPropertiesAspectTest : public dart::common::AspectWithVersionedProperties<
    StateAndPropertiesAspectTest, dart::common::Empty>
{

};

class GenericAspect : public Aspect, public Subject
{
public:

  GenericAspect()
    : Aspect()
  {
    // Do nothing
  }

  std::unique_ptr<Aspect> cloneAspect() const override final
  {
    return dart::common::make_unique<GenericAspect>();
  }

};

class SpecializedAspect : public Aspect, public Subject
{
public:

  SpecializedAspect()
    : Aspect()
  {
    // Do nothing
  }

  std::unique_ptr<Aspect> cloneAspect() const override final
  {
    return dart::common::make_unique<SpecializedAspect>();
  }
};

template <typename T>
class StatefulAspect : public Aspect, public Subject
{
public:

  struct Data
  {
    T val;

    Data(const T& someVal = 0)
      : val(someVal)
    {
      // Do nothing
    }
  };

  using State = Aspect::MakeState<Data>;
  using Properties = Aspect::MakeProperties<Data>;

  StatefulAspect()
    : Aspect()
  {
    // Do nothing
  }

  StatefulAspect(const StatefulAspect& other)
    : Aspect(),
      mState(other.mState), mProperties(other.mProperties)
  {
    // Do nothing
  }

  StatefulAspect(const T& state)
    : Aspect(), mState(state)
  {
    // Do nothing
  }

  StatefulAspect(const T& state, const T& properties)
    : Aspect(),
      mState(state), mProperties(properties)
  {
    // Do nothing
  }

  std::unique_ptr<Aspect> cloneAspect() const override final
  {
    return dart::common::make_unique<StatefulAspect>(*this);
  }

  void setAspectState(const Aspect::State& otherState) override
  {
    mState.copy(otherState);
  }

  const Aspect::State* getAspectState() const override
  {
    return &mState;
  }

  void setAspectProperties(const Aspect::Properties& someProperties) override
  {
    mProperties.copy(someProperties);
  }

  const Aspect::Properties* getAspectProperties() const override
  {
    return &mProperties;
  }

  void randomize()
  {
    mState.val = dart::math::Random::uniform<T>(0, 100);
    mProperties.val = dart::math::Random::uniform<T>(0, 100);
  }

  State mState;

  Properties mProperties;

};

typedef StatefulAspect<double> DoubleAspect;
typedef StatefulAspect<float>  FloatAspect;
typedef StatefulAspect<char>   CharAspect;
typedef StatefulAspect<int>    IntAspect;

class CustomSpecializedComposite : public SpecializedForAspect<SpecializedAspect> { };

TEST(Aspect, Generic)
{
  Composite comp;

  EXPECT_TRUE( comp.get<GenericAspect>() == nullptr );
  EXPECT_FALSE( comp.has<GenericAspect>() );

  comp.set<GenericAspect>(nullptr);
  EXPECT_TRUE( comp.get<GenericAspect>() == nullptr );
  EXPECT_FALSE( comp.has<GenericAspect>() );

  sub_ptr<GenericAspect> aspect = comp.createAspect<GenericAspect>();
  GenericAspect* rawAspect = aspect;
  EXPECT_FALSE( nullptr == aspect );
  EXPECT_FALSE( nullptr == rawAspect );
  EXPECT_TRUE( comp.get<GenericAspect>() == aspect );
  EXPECT_TRUE( comp.get<GenericAspect>() == rawAspect);

  GenericAspect* newAspect = comp.createAspect<GenericAspect>();
  EXPECT_FALSE( nullptr == newAspect );
  EXPECT_FALSE( nullptr == rawAspect );
  EXPECT_FALSE( rawAspect == newAspect );
  EXPECT_TRUE( nullptr == aspect );
}

TEST(Aspect, Specialized)
{
  usedSpecializedAspectAccess = false;
  CustomSpecializedComposite comp;

  EXPECT_TRUE( comp.get<SpecializedAspect>() == nullptr );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_TRUE( comp.getSpecializedAspect() == nullptr );
  EXPECT_TRUE( comp.get<GenericAspect>() == nullptr );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  sub_ptr<SpecializedAspect> spec = comp.createAspect<SpecializedAspect>();
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
  SpecializedAspect* rawSpec = spec;

  sub_ptr<GenericAspect> generic = comp.createAspect<GenericAspect>();
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
  GenericAspect* rawGeneric = generic;

  EXPECT_TRUE( comp.get<SpecializedAspect>() == spec );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_TRUE( comp.getSpecializedAspect() == spec );

  EXPECT_TRUE( comp.get<GenericAspect>() == generic );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

//  SpecializedAspect* newSpec = comp.createSpecializedAspect();
  SpecializedAspect* newSpec = comp.createAspect<SpecializedAspect>();
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  GenericAspect* newGeneric = comp.createAspect<GenericAspect>();
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  EXPECT_TRUE( nullptr == spec );
  EXPECT_TRUE( nullptr == generic );

  EXPECT_FALSE( comp.get<SpecializedAspect>() == rawSpec );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_FALSE( comp.getSpecializedAspect() == rawSpec );

  EXPECT_FALSE( comp.get<GenericAspect>() == rawGeneric );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  EXPECT_TRUE( comp.get<SpecializedAspect>() == newSpec );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_TRUE( comp.getSpecializedAspect() == newSpec );

  EXPECT_TRUE( comp.get<GenericAspect>() == newGeneric );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
}

TEST(Aspect, Releasing)
{
  Composite sender;
  CustomSpecializedComposite receiver;

  // ---- Test generic releases ----
  {
    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );

    sub_ptr<GenericAspect> aspect = sender.createAspect<GenericAspect>();

    EXPECT_TRUE( sender.get<GenericAspect>() == aspect );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );

    receiver.set<GenericAspect>(sender.releaseAspect<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );

    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == aspect );

    sender.set<GenericAspect>(sender.releaseAspect<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );

    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == aspect );

    receiver.set<GenericAspect>(receiver.releaseAspect<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );

    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == aspect );

    sender.set<GenericAspect>(receiver.releaseAspect<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );

    EXPECT_TRUE( sender.get<GenericAspect>() == aspect );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );

    sender.releaseAspect<GenericAspect>();

    EXPECT_TRUE( nullptr == aspect );
    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );
  }

  // ---- Test specialized releases ----
  {
    EXPECT_TRUE( sender.get<SpecializedAspect>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );

    sub_ptr<SpecializedAspect> spec = sender.createAspect<SpecializedAspect>();

    EXPECT_TRUE( sender.get<SpecializedAspect>() == spec );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );

//    receiver.setSpecializedAspect(sender.release<SpecializedAspect>());
    receiver.set<SpecializedAspect>(sender.releaseAspect<SpecializedAspect>());

    EXPECT_FALSE( nullptr == spec );

    EXPECT_TRUE( sender.get<SpecializedAspect>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == spec );

//    sender.set<SpecializedAspect>(receiver.releaseSpecializedAspect());
    sender.set<SpecializedAspect>(receiver.releaseAspect<SpecializedAspect>());

    EXPECT_FALSE( nullptr == spec );

    EXPECT_TRUE( sender.get<SpecializedAspect>() == spec );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );

    sender.releaseAspect<SpecializedAspect>();

    EXPECT_TRUE( nullptr == spec );
    EXPECT_TRUE( sender.get<SpecializedAspect>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );
  }


  // ---- Test non-moving set method ----
  {
    // The set() methods being used in this block of code will make clones of
    // the aspects that are being passed in instead of transferring their
    // ownership like the previous blocks of code were.
    sub_ptr<GenericAspect> aspect = sender.createAspect<GenericAspect>();

    // This should create a copy of the GenericAspect without taking the aspect
    // away from 'sender'
    receiver.set<GenericAspect>(sender.get<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );
    EXPECT_FALSE( receiver.get<GenericAspect>() == aspect );
    EXPECT_FALSE( receiver.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( sender.get<GenericAspect>() == aspect );

    sub_ptr<GenericAspect> rec_aspect = receiver.get<GenericAspect>();
    EXPECT_FALSE( nullptr == rec_aspect );

    // This should replace the first GenericAspect that was created in 'sender'
    sender.set<GenericAspect>(receiver.get<GenericAspect>());

    EXPECT_TRUE( nullptr == aspect );
    EXPECT_FALSE( nullptr == rec_aspect );
    EXPECT_FALSE( sender.get<GenericAspect>() == receiver.get<GenericAspect>() );

    sub_ptr<GenericAspect> aspect2 = sender.get<GenericAspect>();
    EXPECT_FALSE( nullptr == aspect2 );

    sender.set<GenericAspect>(receiver.releaseAspect<GenericAspect>());
    EXPECT_TRUE( nullptr == aspect2 );
    EXPECT_FALSE( nullptr == rec_aspect );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( sender.get<GenericAspect>() == rec_aspect );
  }
}

template <class AspectT>
void makeStatesDifferent(AspectT* aspect, const AspectT* differentFrom)
{
  std::size_t counter = 0;
  while( aspect->mState.val == differentFrom->mState.val )
  {
    aspect->randomize();
    ++counter;
    if(counter > 10)
    {
      dtwarn << "[testAspect::makeStatesDifferent] Randomization failed to make "
             << "the states different after " << counter << " attempts!\n";
      break;
    }
  }
}

template <class AspectT>
void makePropertiesDifferent(AspectT* aspect, const AspectT* differentFrom)
{
  std::size_t counter = 0;
  while( aspect->mProperties.val == differentFrom->mProperties.val )
  {
    aspect->randomize();
    ++counter;
    if(counter > 10)
    {
      dtwarn << "[testAspect::makePropertiesDifferent] Randomization failed to "
             << "make the states different after " << counter << " attempts!\n";
      break;
    }
  }
}

TEST(Aspect, StateAndProperties)
{
  Composite comp1;
  comp1.createAspect<DoubleAspect>();
  comp1.createAspect<FloatAspect>();
  comp1.createAspect<CharAspect>();
  comp1.createAspect<IntAspect>();

  Composite comp2;
  comp2.createAspect<DoubleAspect>();
  comp2.createAspect<FloatAspect>();

  comp1.createAspect<StateAspectTest>();

  // ---- Test State Transfer ----

  comp2.setCompositeState(comp1.getCompositeState());

  EXPECT_EQ( comp1.get<DoubleAspect>()->mState.val,
             comp2.get<DoubleAspect>()->mState.val );

  EXPECT_EQ( comp1.get<FloatAspect>()->mState.val,
             comp2.get<FloatAspect>()->mState.val );

  makeStatesDifferent( comp2.get<DoubleAspect>(), comp1.get<DoubleAspect>() );
  makeStatesDifferent( comp2.get<FloatAspect>(), comp1.get<FloatAspect>() );

  EXPECT_NE( comp1.get<DoubleAspect>()->mState.val,
             comp2.get<DoubleAspect>()->mState.val );

  EXPECT_NE( comp1.get<FloatAspect>()->mState.val,
             comp2.get<FloatAspect>()->mState.val );

  comp1.setCompositeState( comp2.getCompositeState() );

  EXPECT_EQ( comp1.get<DoubleAspect>()->mState.val,
             comp2.get<DoubleAspect>()->mState.val );

  EXPECT_EQ( comp1.get<FloatAspect>()->mState.val,
             comp2.get<FloatAspect>()->mState.val );

  EXPECT_TRUE( nullptr == comp2.get<CharAspect>() );
  EXPECT_TRUE( nullptr == comp2.get<IntAspect>() );


  // ---- Test Property Transfer ----

  comp2.setCompositeProperties(comp1.getCompositeProperties());

  EXPECT_EQ( comp1.get<DoubleAspect>()->mProperties.val,
             comp2.get<DoubleAspect>()->mProperties.val );

  EXPECT_EQ( comp1.get<FloatAspect>()->mProperties.val,
             comp2.get<FloatAspect>()->mProperties.val );

  makePropertiesDifferent( comp2.get<DoubleAspect>(), comp1.get<DoubleAspect>() );
  makePropertiesDifferent( comp2.get<FloatAspect>(), comp1.get<FloatAspect>() );

  EXPECT_NE( comp1.get<DoubleAspect>()->mProperties.val,
             comp2.get<DoubleAspect>()->mProperties.val );

  EXPECT_NE( comp1.get<FloatAspect>()->mProperties.val,
             comp2.get<FloatAspect>()->mProperties.val );

  comp1.setCompositeProperties( comp2.getCompositeProperties() );

  EXPECT_EQ( comp1.get<DoubleAspect>()->mProperties.val,
             comp2.get<DoubleAspect>()->mProperties.val );

  EXPECT_EQ( comp1.get<FloatAspect>()->mProperties.val,
             comp2.get<FloatAspect>()->mProperties.val );

  EXPECT_TRUE( nullptr == comp2.get<CharAspect>() );
  EXPECT_TRUE( nullptr == comp2.get<IntAspect>() );


  // ---- Test Data Containers ----
  Composite::MakeState<DoubleAspect, IntAspect, FloatAspect> state(
        comp1.getCompositeState());

  EXPECT_EQ(comp1.get<DoubleAspect>()->mState.val,
            state.DoubleAspect::State::val);
  EXPECT_EQ(comp1.get<FloatAspect>()->mState.val,
            state.FloatAspect::State::val);
  EXPECT_EQ(comp1.get<IntAspect>()->mState.val,
            state.IntAspect::State::val);


  Composite::MakeProperties<DoubleAspect, CharAspect, FloatAspect> properties(
        comp2.getCompositeProperties());

  EXPECT_EQ(comp1.get<DoubleAspect>()->mProperties.val,
            properties.DoubleAspect::Properties::val);
  EXPECT_EQ(comp1.get<CharAspect>()->mProperties.val,
            properties.CharAspect::Properties::val);
  EXPECT_EQ(comp1.get<FloatAspect>()->mProperties.val,
            properties.FloatAspect::Properties::val);


  DoubleAspect::State doubleState(2.5);
  FloatAspect::State floatState(4.7);
  IntAspect::State intState(7);
  CharAspect::State charState('h');

  // The constructor arguments should match the type order
  Composite::MakeState<DoubleAspect, IntAspect, CharAspect, FloatAspect>(
        doubleState, intState, charState, floatState);

  // ---- Test copying and merging ----
  Composite::Properties c_properties_1(properties);
  EXPECT_FALSE(c_properties_1.has<IntAspect>());

  Composite::Properties c_properties_2;
  c_properties_2.create<IntAspect>();
  EXPECT_TRUE(c_properties_2.has<IntAspect>());

  c_properties_2.merge(c_properties_1);
  EXPECT_TRUE(c_properties_2.has<IntAspect>());

  c_properties_2.copy(c_properties_1);
  EXPECT_FALSE(c_properties_2.has<IntAspect>());
}

TEST(Aspect, Construction)
{
  Composite comp;

  comp.createAspect<DoubleAspect>();

  double s1 = dart::math::Random::uniform<double>(0, 100);
  comp.createAspect<DoubleAspect>(s1);
  EXPECT_EQ(comp.get<DoubleAspect>()->mState.val, s1);

  double s2 = dart::math::Random::uniform<double>(0, 100);
  double p = dart::math::Random::uniform<double>(0, 100);
  comp.createAspect<DoubleAspect>(s2, p);
  EXPECT_NE(comp.get<DoubleAspect>()->mState.val, s1);
  EXPECT_EQ(comp.get<DoubleAspect>()->mState.val, s2);
  EXPECT_EQ(comp.get<DoubleAspect>()->mProperties.val, p);
}

TEST(Aspect, Joints)
{
  usedSpecializedAspectAccess = false;

  dart::dynamics::SkeletonPtr skel = Skeleton::create();

  dart::dynamics::EulerJoint* euler =
      skel->createJointAndBodyNodePair<dart::dynamics::EulerJoint>().first;
  euler->getGenericJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  euler->getEulerJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::PlanarJoint* planar =
      skel->createJointAndBodyNodePair<dart::dynamics::PlanarJoint>().first;
  planar->getGenericJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  planar->getPlanarJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::PrismaticJoint* prismatic =
      skel->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>().first;
  prismatic->getGenericJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  prismatic->getPrismaticJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::RevoluteJoint* revolute =
      skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>().first;
  revolute->getGenericJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  revolute->getRevoluteJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::ScrewJoint* screw =
      skel->createJointAndBodyNodePair<dart::dynamics::ScrewJoint>().first;
  screw->getGenericJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  screw->getScrewJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::UniversalJoint* universal =
      skel->createJointAndBodyNodePair<dart::dynamics::UniversalJoint>().first;
  universal->getGenericJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  universal->getUniversalJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  // Regression test for issue #645
  universal->getGenericJointAspect(true);
}

TEST(Aspect, BodyNodes)
{
  SkeletonPtr skel = Skeleton::create();

  BodyNode* bn =
      skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>().second;

  bn->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones()));

  EXPECT_EQ(bn->getNumShapeNodes(), 1u);
  EXPECT_EQ(bn->getNumShapeNodesWith<dart::dynamics::VisualAspect>(), 1u);
  EXPECT_EQ(bn->getNumShapeNodesWith<dart::dynamics::CollisionAspect>(), 1u);
  EXPECT_EQ(bn->getNumShapeNodesWith<dart::dynamics::DynamicsAspect>(), 1u);
}

TEST(Aspect, Duplication)
{
  Composite comp1, comp2;

  comp1.createAspect<DoubleAspect>();
  comp1.createAspect<IntAspect>();
  comp1.createAspect<FloatAspect>();
  comp1.createAspect<CharAspect>();

  comp2.createAspect<DoubleAspect>();

  comp2.duplicateAspects(&comp1);

  EXPECT_FALSE(comp2.get<DoubleAspect>() == nullptr);
  EXPECT_FALSE(comp2.get<IntAspect>() == nullptr);
  EXPECT_FALSE(comp2.get<FloatAspect>() == nullptr);
  EXPECT_FALSE(comp2.get<CharAspect>() == nullptr);

  Composite::MakeState<DoubleAspect, IntAspect, FloatAspect> state;
  state.DoubleAspect::State::val = 1e-6;
  state.FloatAspect::State::val = 1.5;
  state.IntAspect::State::val = 456;

  comp1.setCompositeState(state);

  EXPECT_EQ(comp1.get<DoubleAspect>()->mState.val, 1e-6);
  EXPECT_EQ(comp1.get<FloatAspect>()->mState.val, 1.5);
  EXPECT_EQ(comp1.get<IntAspect>()->mState.val, 456);

  state = comp2.getCompositeState();

  EXPECT_EQ(state.DoubleAspect::State::val, 0);
  EXPECT_EQ(state.FloatAspect::State::val, 0);
  EXPECT_EQ(state.IntAspect::State::val, 0);

  state = comp1.getCompositeState();

  EXPECT_EQ(state.DoubleAspect::State::val, 1e-6);
  EXPECT_EQ(state.FloatAspect::State::val, 1.5);
  EXPECT_EQ(state.IntAspect::State::val, 456);
}

TEST(Aspect, Embedded)
{
  EmbeddedStateComposite s;
  EXPECT_TRUE(s.has<EmbeddedStateComposite::Aspect>());
  EXPECT_TRUE(s.get<EmbeddedStateComposite::Aspect>() != nullptr);

  EmbeddedPropertiesComposite p;
  EXPECT_TRUE(p.has<EmbeddedPropertiesComposite::Aspect>());
  EXPECT_TRUE(p.get<EmbeddedPropertiesComposite::Aspect>() != nullptr);

  EmbeddedStateAndPropertiesComposite sp;
  EXPECT_TRUE(sp.has<EmbeddedStateAndPropertiesComposite::Aspect>());
  EXPECT_TRUE(sp.get<EmbeddedStateAndPropertiesComposite::Aspect>() != nullptr);

  // --------- Test Embedded State -----------
  EmbeddedStateComposite::AspectState state = s.getAspectState();
  EmbeddedStateComposite::AspectState a_state =
      s.get<EmbeddedStateComposite::Aspect>()->getState();

  EXPECT_TRUE(state == a_state);

  state.d = 3.5;
  state.i = 750;
  s.setAspectState(state);

  state = s.getAspectState();
  a_state = s.get<EmbeddedStateComposite::Aspect>()->getState();
  EXPECT_EQ(3.5, state.d);
  EXPECT_EQ(750, state.i);
  EXPECT_TRUE(state == a_state);

  EXPECT_EQ(&s.get<EmbeddedStateComposite::Aspect>()->getState(),
             s.get<EmbeddedStateComposite::Aspect>()->getAspectState());
  EXPECT_EQ(&s.getAspectState(),
             s.get<EmbeddedStateComposite::Aspect>()->getAspectState());

  state.d = -4e-3;
  state.i = -18;
  s.get<EmbeddedStateComposite::Aspect>()->setAspectState(state);

  state = s.getAspectState();
  a_state = s.get<EmbeddedStateComposite::Aspect>()->getState();
  EXPECT_EQ(-4e-3, state.d);
  EXPECT_EQ(-18,   state.i);
  EXPECT_TRUE(state == a_state);


  // --------- Test Embedded Properties -----------
  EmbeddedPropertiesComposite::AspectProperties prop = p.getAspectProperties();
  EmbeddedPropertiesComposite::AspectProperties a_prop =
      p.get<EmbeddedPropertiesComposite::Aspect>()->getProperties();

  EXPECT_TRUE(prop == a_prop);

  prop.f = 7.5;
  prop.b = true;
  p.setAspectProperties(prop);

  prop = p.getAspectProperties();
  a_prop = p.get<EmbeddedPropertiesComposite::Aspect>()->getProperties();
  EXPECT_EQ(7.5,  prop.f);
  EXPECT_EQ(true, prop.b);
  EXPECT_TRUE(prop == a_prop);

  // Make sure the pointers are consistent
  EXPECT_EQ(&p.get<EmbeddedPropertiesComposite::Aspect>()->getProperties(),
             p.get<EmbeddedPropertiesComposite::Aspect>()->getAspectProperties());
  EXPECT_EQ(&p.getAspectProperties(),
             p.get<EmbeddedPropertiesComposite::Aspect>()->getAspectProperties());

  prop.f = -7e5;
  prop.b = false;
  p.get<EmbeddedPropertiesComposite::Aspect>()->setAspectProperties(prop);

  prop = p.getAspectProperties();
  a_prop = p.get<EmbeddedPropertiesComposite::Aspect>()->getProperties();
  EXPECT_EQ(-7e5,  prop.f);
  EXPECT_EQ(false, prop.b);
  EXPECT_TRUE(prop == a_prop);


  // --------- Test Embedded State and Properties Combination -----------
  // Make sure the pointers are consistent
  EXPECT_EQ(&sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState(),
             sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getAspectState());
  EXPECT_EQ(&sp.getAspectState(),
             sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getAspectState());

  EXPECT_EQ(&sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties(),
             sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getAspectProperties());
  EXPECT_EQ(&sp.getAspectProperties(),
             sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getAspectProperties());

  sp.setAspectState(s.getAspectState());
  sp.setAspectProperties(p.getAspectProperties());


  // --------- Test Inheritance -----------
  InheritAndEmbedStateComposite s_derived;
  EXPECT_NE(s_derived.get<EmbeddedStateComposite::Aspect>()->getState(),
            s.get<EmbeddedStateComposite::Aspect>()->getState());
  s_derived.setCompositeState(s.getCompositeState());
  EXPECT_EQ(s_derived.get<EmbeddedStateComposite::Aspect>()->getState(),
            s.get<EmbeddedStateComposite::Aspect>()->getState());

  InheritAndEmbedPropertiesComposite p_derived;
  EXPECT_NE(p_derived.get<EmbeddedPropertiesComposite::Aspect>()->getProperties(),
            p.get<EmbeddedPropertiesComposite::Aspect>()->getProperties());
  p_derived.setCompositeProperties(p.getCompositeProperties());
  EXPECT_EQ(p_derived.get<EmbeddedPropertiesComposite::Aspect>()->getProperties(),
            p.get<EmbeddedPropertiesComposite::Aspect>()->getProperties());

  InheritAndEmbedStateAndPropertiesComposite sp_derived;
  EXPECT_NE(sp_derived.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState(),
            sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState());
  EXPECT_NE(sp_derived.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties(),
            sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties());
  sp_derived.setCompositeState(sp.getCompositeState());
  EXPECT_EQ(sp_derived.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState(),
            sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState());
  EXPECT_NE(sp_derived.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties(),
            sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties());
  sp_derived.setCompositeProperties(sp.getCompositeProperties());
  EXPECT_EQ(sp_derived.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState(),
            sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getState());
  EXPECT_EQ(sp_derived.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties(),
            sp.get<EmbeddedStateAndPropertiesComposite::Aspect>()->getProperties());


  // --------- Test Construction -----------
  EmbeddedStateComposite s_constructed(state);
  EXPECT_EQ(s_constructed.get<EmbeddedStateComposite::Aspect>()->getState(), state);
}
