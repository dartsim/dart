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

// Use this preprocessor token to allow us to test that the specialized versions
// of Aspect functions are being used appropriately. This preprocessor token
// should NOT be used anywhere outside of this file (testAspect.cpp).
#define DART_UNITTEST_SPECIALIZED_ASPECT_ACCESS

#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Subject.h"
#include "dart/common/sub_ptr.h"
#include "dart/common/Composite.h"
#include "dart/common/SpecializedForAspect.h"

#include "dart/dynamics/EulerJoint.h"

using namespace dart::common;

// Testing the creation of an Aspect using the AspectWithState template class
class StateAspectTest : public dart::common::AspectWithState<
    StateAspectTest, dart::common::Empty>
{
public:

  StateAspectTest(Composite* mgr, const StateData& state = StateData())
    : dart::common::AspectWithState<StateAspectTest, dart::common::Empty>(mgr, state)
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

  GenericAspect(Composite* manager)
    : Aspect(manager)
  {
    // Do nothing
  }

  std::unique_ptr<Aspect> cloneAspect(Composite* newManager) const override final
  {
    return dart::common::make_unique<GenericAspect>(newManager);
  }

};

class SpecializedAspect : public Aspect, public Subject
{
public:

  SpecializedAspect(Composite* manager)
    : Aspect(manager)
  {
    // Do nothing
  }

  std::unique_ptr<Aspect> cloneAspect(Composite* newManager) const override final
  {
    return dart::common::make_unique<SpecializedAspect>(newManager);
  }
};

template <typename T>
class StatefulAspect : public Aspect, public Subject
{
public:

  class State : public Aspect::State
  {
  public:

    State() : val(static_cast<T>(dart::math::random(0, 100))) { }

    State(const State& other) : val(other.val) { }

    State(const T& otherVal) : val(otherVal) { }

    T val;

    std::unique_ptr<Aspect::State> clone() const override
    {
      return dart::common::make_unique<State>(*this);
    }

    void copy(const Aspect::State& anotherState) override
    {
      val = static_cast<const State&>(anotherState).val;
    }
  };

  class Properties : public Aspect::Properties
  {
  public:

    Properties() : val(static_cast<T>(dart::math::random(0, 100))) { }

    Properties(const Properties& other) : val(other.val) { }

    Properties(const T& otherVal) : val(otherVal) { }

    T val;

    std::unique_ptr<Aspect::Properties> clone() const override
    {
      return dart::common::make_unique<Properties>(*this);
    }

    void copy(const Aspect::Properties& otherProperties) override
    {
      val = static_cast<const Properties&>(otherProperties).val;
    }
  };

  StatefulAspect(Composite* mgr)
    : Aspect(mgr)
  {
    // Do nothing
  }

  StatefulAspect(Composite* mgr, const StatefulAspect& other)
    : Aspect(mgr),
      mState(other.mState), mProperties(other.mProperties)
  {
    // Do nothing
  }

  StatefulAspect(Composite* mgr, const T& state)
    : Aspect(mgr), mState(state)
  {
    // Do nothing
  }

  StatefulAspect(Composite* mgr, const T& state, const T& properties)
    : Aspect(mgr),
      mState(state), mProperties(properties)
  {
    // Do nothing
  }

  std::unique_ptr<Aspect> cloneAspect(Composite* newManager) const override final
  {
    return dart::common::make_unique<StatefulAspect>(newManager, *this);
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
    mState.val = static_cast<T>(dart::math::random(0, 100));
    mProperties.val = static_cast<T>(dart::math::random(0, 100));
  }

  State mState;

  Properties mProperties;

};

typedef StatefulAspect<double> DoubleAspect;
typedef StatefulAspect<float>  FloatAspect;
typedef StatefulAspect<char>   CharAspect;
typedef StatefulAspect<int>    IntAspect;

class CustomSpecializedManager : public SpecializedForAspect<SpecializedAspect> { };

TEST(Aspect, Generic)
{
  Composite mgr;

  EXPECT_TRUE( mgr.get<GenericAspect>() == nullptr );

  sub_ptr<GenericAspect> aspect = mgr.create<GenericAspect>();
  GenericAspect* rawAspect = aspect;
  EXPECT_FALSE( nullptr == aspect );
  EXPECT_TRUE( mgr.get<GenericAspect>() == aspect );

  GenericAspect* newAspect = mgr.create<GenericAspect>();
  EXPECT_FALSE( nullptr == newAspect );
  EXPECT_TRUE( nullptr == aspect );
  EXPECT_FALSE( rawAspect == newAspect );
}

TEST(Aspect, Specialized)
{
  usedSpecializedAspectAccess = false;
  CustomSpecializedManager mgr;

  EXPECT_TRUE( mgr.get<SpecializedAspect>() == nullptr );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_TRUE( mgr.getSpecializedAspect() == nullptr );
  EXPECT_TRUE( mgr.get<GenericAspect>() == nullptr );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  sub_ptr<SpecializedAspect> spec = mgr.create<SpecializedAspect>();
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
  SpecializedAspect* rawSpec = spec;

  sub_ptr<GenericAspect> generic = mgr.create<GenericAspect>();
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
  GenericAspect* rawGeneric = generic;

  EXPECT_TRUE( mgr.get<SpecializedAspect>() == spec );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_TRUE( mgr.getSpecializedAspect() == spec );

  EXPECT_TRUE( mgr.get<GenericAspect>() == generic );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

//  SpecializedAspect* newSpec = mgr.createSpecializedAspect();
  SpecializedAspect* newSpec = mgr.create<SpecializedAspect>();
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  GenericAspect* newGeneric = mgr.create<GenericAspect>();
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  EXPECT_TRUE( nullptr == spec );
  EXPECT_TRUE( nullptr == generic );

  EXPECT_FALSE( mgr.get<SpecializedAspect>() == rawSpec );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_FALSE( mgr.getSpecializedAspect() == rawSpec );

  EXPECT_FALSE( mgr.get<GenericAspect>() == rawGeneric );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;

  EXPECT_TRUE( mgr.get<SpecializedAspect>() == newSpec );
  EXPECT_TRUE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
//  EXPECT_TRUE( mgr.getSpecializedAspect() == newSpec );

  EXPECT_TRUE( mgr.get<GenericAspect>() == newGeneric );
  EXPECT_FALSE( usedSpecializedAspectAccess ); usedSpecializedAspectAccess = false;
}

TEST(Aspect, Releasing)
{
  Composite sender;
  CustomSpecializedManager receiver;

  // ---- Test generic releases ----
  {
    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );

    sub_ptr<GenericAspect> aspect = sender.create<GenericAspect>();

    EXPECT_TRUE( sender.get<GenericAspect>() == aspect );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );

    receiver.set<GenericAspect>(sender.release<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );

    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == aspect );

    sender.set<GenericAspect>(receiver.release<GenericAspect>());

    EXPECT_FALSE( nullptr == aspect );

    EXPECT_TRUE( sender.get<GenericAspect>() == aspect );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );

    sender.release<GenericAspect>();

    EXPECT_TRUE( nullptr == aspect );
    EXPECT_TRUE( sender.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );
  }

  // ---- Test specialized releases ----
  {
    EXPECT_TRUE( sender.get<SpecializedAspect>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );

    sub_ptr<SpecializedAspect> spec = sender.create<SpecializedAspect>();

    EXPECT_TRUE( sender.get<SpecializedAspect>() == spec );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );

//    receiver.setSpecializedAspect(sender.release<SpecializedAspect>());
    receiver.set<SpecializedAspect>(sender.release<SpecializedAspect>());

    EXPECT_FALSE( nullptr == spec );

    EXPECT_TRUE( sender.get<SpecializedAspect>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == spec );

//    sender.set<SpecializedAspect>(receiver.releaseSpecializedAspect());
    sender.set<SpecializedAspect>(receiver.release<SpecializedAspect>());

    EXPECT_FALSE( nullptr == spec );

    EXPECT_TRUE( sender.get<SpecializedAspect>() == spec );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );

    sender.release<SpecializedAspect>();

    EXPECT_TRUE( nullptr == spec );
    EXPECT_TRUE( sender.get<SpecializedAspect>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAspect() == nullptr );
  }


  // ---- Test non-moving set method ----
  {
    // The set() methods being used in this block of code will make clones of
    // the aspects that are being passed in instead of transferring their
    // ownership like the previous blocks of code were.
    sub_ptr<GenericAspect> aspect = sender.create<GenericAspect>();

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

    sender.set<GenericAspect>(receiver.release<GenericAspect>());
    EXPECT_TRUE( nullptr == aspect2 );
    EXPECT_FALSE( nullptr == rec_aspect );
    EXPECT_TRUE( receiver.get<GenericAspect>() == nullptr );
    EXPECT_TRUE( sender.get<GenericAspect>() == rec_aspect );
  }
}

template <class AspectT>
void makeStatesDifferent(AspectT* aspect, const AspectT* differentFrom)
{
  size_t counter = 0;
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
  size_t counter = 0;
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
  Composite mgr1;
  mgr1.create<DoubleAspect>();
  mgr1.create<FloatAspect>();
  mgr1.create<CharAspect>();
  mgr1.create<IntAspect>();

  Composite mgr2;
  mgr2.create<DoubleAspect>();
  mgr2.create<FloatAspect>();

  mgr1.create<StateAspectTest>();

  // ---- Test state transfer ----

  mgr2.setAspectStates(mgr1.getAspectStates());

  EXPECT_EQ( mgr1.get<DoubleAspect>()->mState.val,
             mgr2.get<DoubleAspect>()->mState.val );

  EXPECT_EQ( mgr1.get<FloatAspect>()->mState.val,
             mgr2.get<FloatAspect>()->mState.val );

  makeStatesDifferent( mgr2.get<DoubleAspect>(), mgr1.get<DoubleAspect>() );
  makeStatesDifferent( mgr2.get<FloatAspect>(), mgr1.get<FloatAspect>() );

  EXPECT_NE( mgr1.get<DoubleAspect>()->mState.val,
             mgr2.get<DoubleAspect>()->mState.val );

  EXPECT_NE( mgr1.get<FloatAspect>()->mState.val,
             mgr2.get<FloatAspect>()->mState.val );

  mgr1.setAspectStates( mgr2.getAspectStates() );

  EXPECT_EQ( mgr1.get<DoubleAspect>()->mState.val,
             mgr2.get<DoubleAspect>()->mState.val );

  EXPECT_EQ( mgr1.get<FloatAspect>()->mState.val,
             mgr2.get<FloatAspect>()->mState.val );

  EXPECT_TRUE( nullptr == mgr2.get<CharAspect>() );
  EXPECT_TRUE( nullptr == mgr2.get<IntAspect>() );


  // ---- Test property transfer ----

  mgr2.setAspectProperties(mgr1.getAspectProperties());

  EXPECT_EQ( mgr1.get<DoubleAspect>()->mProperties.val,
             mgr2.get<DoubleAspect>()->mProperties.val );

  EXPECT_EQ( mgr1.get<FloatAspect>()->mProperties.val,
             mgr2.get<FloatAspect>()->mProperties.val );

  makePropertiesDifferent( mgr2.get<DoubleAspect>(), mgr1.get<DoubleAspect>() );
  makePropertiesDifferent( mgr2.get<FloatAspect>(), mgr1.get<FloatAspect>() );

  EXPECT_NE( mgr1.get<DoubleAspect>()->mProperties.val,
             mgr2.get<DoubleAspect>()->mProperties.val );

  EXPECT_NE( mgr1.get<FloatAspect>()->mProperties.val,
             mgr2.get<FloatAspect>()->mProperties.val );

  mgr1.setAspectProperties( mgr2.getAspectProperties() );

  EXPECT_EQ( mgr1.get<DoubleAspect>()->mProperties.val,
             mgr2.get<DoubleAspect>()->mProperties.val );

  EXPECT_EQ( mgr1.get<FloatAspect>()->mProperties.val,
             mgr2.get<FloatAspect>()->mProperties.val );

  EXPECT_TRUE( nullptr == mgr2.get<CharAspect>() );
  EXPECT_TRUE( nullptr == mgr2.get<IntAspect>() );
}

TEST(Aspect, Construction)
{
  Composite mgr;

  mgr.create<DoubleAspect>();

  double s = dart::math::random(0, 100);
  mgr.create<DoubleAspect>(s);
  EXPECT_EQ(mgr.get<DoubleAspect>()->mState.val, s);

  double p = dart::math::random(0, 100);
  mgr.create<DoubleAspect>(dart::math::random(0, 100), p);
  EXPECT_EQ(mgr.get<DoubleAspect>()->mProperties.val, p);
}

TEST(Aspect, Joints)
{
  usedSpecializedAspectAccess = false;

  dart::dynamics::SkeletonPtr skel = Skeleton::create();

  dart::dynamics::EulerJoint* euler =
      skel->createJointAndBodyNodePair<dart::dynamics::EulerJoint>().first;
  euler->getMultiDofJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  euler->getEulerJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::PlanarJoint* planar =
      skel->createJointAndBodyNodePair<dart::dynamics::PlanarJoint>().first;
  planar->getMultiDofJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  planar->getPlanarJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::PrismaticJoint* prismatic =
      skel->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>().first;
  prismatic->getSingleDofJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  prismatic->getPrismaticJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::RevoluteJoint* revolute =
      skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>().first;
  revolute->getSingleDofJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  revolute->getRevoluteJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::ScrewJoint* screw =
      skel->createJointAndBodyNodePair<dart::dynamics::ScrewJoint>().first;
  screw->getSingleDofJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  screw->getScrewJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  dart::dynamics::UniversalJoint* universal =
      skel->createJointAndBodyNodePair<dart::dynamics::UniversalJoint>().first;
  universal->getMultiDofJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;
  universal->getUniversalJointAspect();
  EXPECT_TRUE(usedSpecializedAspectAccess); usedSpecializedAspectAccess = false;

  // Regression test for issue #645
  universal->getMultiDofJointAspect(true);
}

TEST(Aspect, Duplication)
{
  Composite mgr1, mgr2;

  mgr1.create<DoubleAspect>();
  mgr1.create<IntAspect>();
  mgr1.create<FloatAspect>();
  mgr1.create<CharAspect>();

  mgr2.create<DoubleAspect>();

  mgr2.duplicateAspects(&mgr1);

  EXPECT_FALSE(mgr2.get<DoubleAspect>() == nullptr);
  EXPECT_FALSE(mgr2.get<IntAspect>() == nullptr);
  EXPECT_FALSE(mgr2.get<FloatAspect>() == nullptr);
  EXPECT_FALSE(mgr2.get<CharAspect>() == nullptr);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
