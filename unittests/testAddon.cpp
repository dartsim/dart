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
// of Addon functions are being used appropriately. This preprocessor token
// should NOT be used anywhere outside of this file (testAddon.cpp).
#define DART_UNITTEST_SPECIALIZED_ADDON_ACCESS

#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Subject.h"
#include "dart/common/sub_ptr.h"
#include "dart/common/AddonManager.h"
#include "dart/common/SpecializedAddonManager.h"

#include "dart/dynamics/EulerJoint.h"

using namespace dart::common;

class GenericAddon : public Addon, public Subject
{
public:

  GenericAddon(AddonManager* manager)
    : Addon(manager)
  {
    // Do nothing
  }

  std::unique_ptr<Addon> cloneAddon(AddonManager* newManager) const override final
  {
    return std::unique_ptr<GenericAddon>(new GenericAddon(newManager));
  }

};

class SpecializedAddon : public Addon, public Subject
{
public:

  SpecializedAddon(AddonManager* manager)
    : Addon(manager)
  {
    // Do nothing
  }

  std::unique_ptr<Addon> cloneAddon(AddonManager* newManager) const override final
  {
    return std::unique_ptr<SpecializedAddon>(new SpecializedAddon(newManager));
  }
};

template <typename T>
class StatefulAddon : public Addon, public Subject
{
public:

  class State : public Addon::State
  {
  public:

    State() : val(static_cast<T>(dart::math::random(0, 100))) { }

    State(const State& other) : val(other.val) { }

    State(const T& otherVal) : val(otherVal) { }

    T val;

    std::unique_ptr<Addon::State> clone() const override
    {
      return std::unique_ptr<Addon::State>(new State(*this));
    }

    void copy(const Addon::State& anotherState) override
    {
      val = static_cast<const State&>(anotherState).val;
    }
  };

  class Properties : public Addon::Properties
  {
  public:

    Properties() : val(static_cast<T>(dart::math::random(0, 100))) { }

    Properties(const Properties& other) : val(other.val) { }

    Properties(const T& otherVal) : val(otherVal) { }

    T val;

    std::unique_ptr<Addon::Properties> clone() const override
    {
      return std::unique_ptr<Addon::Properties>(new Properties(*this));
    }

    void copy(const Addon::Properties& otherProperties) override
    {
      val = static_cast<const Properties&>(otherProperties).val;
    }
  };

  StatefulAddon(AddonManager* mgr)
    : Addon(mgr)
  {
    // Do nothing
  }

  StatefulAddon(AddonManager* mgr, const StatefulAddon& other)
    : Addon(mgr),
      mState(other.mState), mProperties(other.mProperties)
  {
    // Do nothing
  }

  StatefulAddon(AddonManager* mgr, const T& state)
    : Addon(mgr), mState(state)
  {
    // Do nothing
  }

  StatefulAddon(AddonManager* mgr, const T& state, const T& properties)
    : Addon(mgr),
      mState(state), mProperties(properties)
  {
    // Do nothing
  }

  std::unique_ptr<Addon> cloneAddon(AddonManager* newManager) const override final
  {
    return std::unique_ptr<Addon>(new StatefulAddon(newManager, *this));
  }

  void setAddonState(const Addon::State& otherState) override
  {
    mState.copy(otherState);
  }

  const Addon::State* getAddonState() const override
  {
    return &mState;
  }

  void setAddonProperties(const Addon::Properties& someProperties) override
  {
    mProperties.copy(someProperties);
  }

  const Addon::Properties* getAddonProperties() const override
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

typedef StatefulAddon<double> DoubleAddon;
typedef StatefulAddon<float>  FloatAddon;
typedef StatefulAddon<char>   CharAddon;
typedef StatefulAddon<int>    IntAddon;

class CustomSpecializedManager : public SpecializedAddonManager<SpecializedAddon> { };

TEST(Addon, Generic)
{
  AddonManager mgr;

  EXPECT_TRUE( mgr.get<GenericAddon>() == nullptr );

  sub_ptr<GenericAddon> addon = mgr.create<GenericAddon>();
  GenericAddon* rawAddon = addon;
  EXPECT_FALSE( nullptr == addon );
  EXPECT_TRUE( mgr.get<GenericAddon>() == addon );

  GenericAddon* newAddon = mgr.create<GenericAddon>();
  EXPECT_FALSE( nullptr == newAddon );
  EXPECT_TRUE( nullptr == addon );
  EXPECT_FALSE( rawAddon == newAddon );
}

TEST(Addon, Specialized)
{
  usedSpecializedAddonAccess = false;
  CustomSpecializedManager mgr;

  EXPECT_TRUE( mgr.get<SpecializedAddon>() == nullptr );
  EXPECT_TRUE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
//  EXPECT_TRUE( mgr.getSpecializedAddon() == nullptr );
  EXPECT_TRUE( mgr.get<GenericAddon>() == nullptr );
  EXPECT_FALSE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;

  sub_ptr<SpecializedAddon> spec = mgr.create<SpecializedAddon>();
  EXPECT_TRUE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
  SpecializedAddon* rawSpec = spec;

  sub_ptr<GenericAddon> generic = mgr.create<GenericAddon>();
  EXPECT_FALSE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
  GenericAddon* rawGeneric = generic;

  EXPECT_TRUE( mgr.get<SpecializedAddon>() == spec );
  EXPECT_TRUE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
//  EXPECT_TRUE( mgr.getSpecializedAddon() == spec );

  EXPECT_TRUE( mgr.get<GenericAddon>() == generic );
  EXPECT_FALSE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;

//  SpecializedAddon* newSpec = mgr.createSpecializedAddon();
  SpecializedAddon* newSpec = mgr.create<SpecializedAddon>();
  EXPECT_TRUE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;

  GenericAddon* newGeneric = mgr.create<GenericAddon>();
  EXPECT_FALSE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;

  EXPECT_TRUE( nullptr == spec );
  EXPECT_TRUE( nullptr == generic );

  EXPECT_FALSE( mgr.get<SpecializedAddon>() == rawSpec );
  EXPECT_TRUE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
//  EXPECT_FALSE( mgr.getSpecializedAddon() == rawSpec );

  EXPECT_FALSE( mgr.get<GenericAddon>() == rawGeneric );
  EXPECT_FALSE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;

  EXPECT_TRUE( mgr.get<SpecializedAddon>() == newSpec );
  EXPECT_TRUE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
//  EXPECT_TRUE( mgr.getSpecializedAddon() == newSpec );

  EXPECT_TRUE( mgr.get<GenericAddon>() == newGeneric );
  EXPECT_FALSE( usedSpecializedAddonAccess ); usedSpecializedAddonAccess = false;
}

TEST(Addon, Releasing)
{
  AddonManager sender;
  CustomSpecializedManager receiver;

  // ---- Test generic releases ----
  {
    EXPECT_TRUE( sender.get<GenericAddon>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAddon>() == nullptr );

    sub_ptr<GenericAddon> addon = sender.create<GenericAddon>();

    EXPECT_TRUE( sender.get<GenericAddon>() == addon );
    EXPECT_TRUE( receiver.get<GenericAddon>() == nullptr );

    receiver.set<GenericAddon>(sender.release<GenericAddon>());

    EXPECT_FALSE( nullptr == addon );

    EXPECT_TRUE( sender.get<GenericAddon>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAddon>() == addon );

    sender.set<GenericAddon>(receiver.release<GenericAddon>());

    EXPECT_FALSE( nullptr == addon );

    EXPECT_TRUE( sender.get<GenericAddon>() == addon );
    EXPECT_TRUE( receiver.get<GenericAddon>() == nullptr );

    sender.release<GenericAddon>();

    EXPECT_TRUE( nullptr == addon );
    EXPECT_TRUE( sender.get<GenericAddon>() == nullptr );
    EXPECT_TRUE( receiver.get<GenericAddon>() == nullptr );
  }

  // ---- Test specialized releases ----
  {
    EXPECT_TRUE( sender.get<SpecializedAddon>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );

    sub_ptr<SpecializedAddon> spec = sender.create<SpecializedAddon>();

    EXPECT_TRUE( sender.get<SpecializedAddon>() == spec );
//    EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );

//    receiver.setSpecializedAddon(sender.release<SpecializedAddon>());
    receiver.set<SpecializedAddon>(sender.release<SpecializedAddon>());

    EXPECT_FALSE( nullptr == spec );

    EXPECT_TRUE( sender.get<SpecializedAddon>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAddon() == spec );

//    sender.set<SpecializedAddon>(receiver.releaseSpecializedAddon());
    sender.set<SpecializedAddon>(receiver.release<SpecializedAddon>());

    EXPECT_FALSE( nullptr == spec );

    EXPECT_TRUE( sender.get<SpecializedAddon>() == spec );
//    EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );

    sender.release<SpecializedAddon>();

    EXPECT_TRUE( nullptr == spec );
    EXPECT_TRUE( sender.get<SpecializedAddon>() == nullptr );
//    EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );
  }


  // ---- Test non-moving set method ----
  {
    // The set() methods being used in this block of code will make clones of
    // the addons that are being passed in instead of transferring their
    // ownership like the previous blocks of code were.
    sub_ptr<GenericAddon> addon = sender.create<GenericAddon>();

    // This should create a copy of the GenericAddon without taking the addon
    // away from 'sender'
    receiver.set<GenericAddon>(sender.get<GenericAddon>());

    EXPECT_FALSE( nullptr == addon );
    EXPECT_FALSE( receiver.get<GenericAddon>() == addon );
    EXPECT_FALSE( receiver.get<GenericAddon>() == nullptr );
    EXPECT_TRUE( sender.get<GenericAddon>() == addon );

    sub_ptr<GenericAddon> rec_addon = receiver.get<GenericAddon>();
    EXPECT_FALSE( nullptr == rec_addon );

    // This should replace the first GenericAddon that was created in 'sender'
    sender.set<GenericAddon>(receiver.get<GenericAddon>());

    EXPECT_TRUE( nullptr == addon );
    EXPECT_FALSE( nullptr == rec_addon );
    EXPECT_FALSE( sender.get<GenericAddon>() == receiver.get<GenericAddon>() );

    sub_ptr<GenericAddon> addon2 = sender.get<GenericAddon>();
    EXPECT_FALSE( nullptr == addon2 );

    sender.set<GenericAddon>(receiver.release<GenericAddon>());
    EXPECT_TRUE( nullptr == addon2 );
    EXPECT_FALSE( nullptr == rec_addon );
    EXPECT_TRUE( receiver.get<GenericAddon>() == nullptr );
    EXPECT_TRUE( sender.get<GenericAddon>() == rec_addon );
  }
}

template <class AddonT>
void makeStatesDifferent(AddonT* addon, const AddonT* differentFrom)
{
  size_t counter = 0;
  while( addon->mState.val == differentFrom->mState.val )
  {
    addon->randomize();
    ++counter;
    if(counter > 10)
    {
      dtwarn << "[testAddon::makeStatesDifferent] Randomization failed to make "
             << "the states different after " << counter << " attempts!\n";
      break;
    }
  }
}

template <class AddonT>
void makePropertiesDifferent(AddonT* addon, const AddonT* differentFrom)
{
  size_t counter = 0;
  while( addon->mProperties.val == differentFrom->mProperties.val )
  {
    addon->randomize();
    ++counter;
    if(counter > 10)
    {
      dtwarn << "[testAddon::makePropertiesDifferent] Randomization failed to "
             << "make the states different after " << counter << " attempts!\n";
      break;
    }
  }
}

TEST(Addon, StateAndProperties)
{
  AddonManager mgr1;
  mgr1.create<DoubleAddon>();
  mgr1.create<FloatAddon>();
  mgr1.create<CharAddon>();
  mgr1.create<IntAddon>();

  AddonManager mgr2;
  mgr2.create<DoubleAddon>();
  mgr2.create<FloatAddon>();

  // ---- Test state transfer ----

  mgr2.setAddonStates(mgr1.getAddonStates());

  EXPECT_EQ( mgr1.get<DoubleAddon>()->mState.val,
             mgr2.get<DoubleAddon>()->mState.val );

  EXPECT_EQ( mgr1.get<FloatAddon>()->mState.val,
             mgr2.get<FloatAddon>()->mState.val );

  makeStatesDifferent( mgr2.get<DoubleAddon>(), mgr1.get<DoubleAddon>() );
  makeStatesDifferent( mgr2.get<FloatAddon>(), mgr1.get<FloatAddon>() );

  EXPECT_NE( mgr1.get<DoubleAddon>()->mState.val,
             mgr2.get<DoubleAddon>()->mState.val );

  EXPECT_NE( mgr1.get<FloatAddon>()->mState.val,
             mgr2.get<FloatAddon>()->mState.val );

  mgr1.setAddonStates( mgr2.getAddonStates() );

  EXPECT_EQ( mgr1.get<DoubleAddon>()->mState.val,
             mgr2.get<DoubleAddon>()->mState.val );

  EXPECT_EQ( mgr1.get<FloatAddon>()->mState.val,
             mgr2.get<FloatAddon>()->mState.val );

  EXPECT_TRUE( nullptr == mgr2.get<CharAddon>() );
  EXPECT_TRUE( nullptr == mgr2.get<IntAddon>() );


  // ---- Test property transfer ----

  mgr2.setAddonProperties(mgr1.getAddonProperties());

  EXPECT_EQ( mgr1.get<DoubleAddon>()->mProperties.val,
             mgr2.get<DoubleAddon>()->mProperties.val );

  EXPECT_EQ( mgr1.get<FloatAddon>()->mProperties.val,
             mgr2.get<FloatAddon>()->mProperties.val );

  makePropertiesDifferent( mgr2.get<DoubleAddon>(), mgr1.get<DoubleAddon>() );
  makePropertiesDifferent( mgr2.get<FloatAddon>(), mgr1.get<FloatAddon>() );

  EXPECT_NE( mgr1.get<DoubleAddon>()->mProperties.val,
             mgr2.get<DoubleAddon>()->mProperties.val );

  EXPECT_NE( mgr1.get<FloatAddon>()->mProperties.val,
             mgr2.get<FloatAddon>()->mProperties.val );

  mgr1.setAddonProperties( mgr2.getAddonProperties() );

  EXPECT_EQ( mgr1.get<DoubleAddon>()->mProperties.val,
             mgr2.get<DoubleAddon>()->mProperties.val );

  EXPECT_EQ( mgr1.get<FloatAddon>()->mProperties.val,
             mgr2.get<FloatAddon>()->mProperties.val );

  EXPECT_TRUE( nullptr == mgr2.get<CharAddon>() );
  EXPECT_TRUE( nullptr == mgr2.get<IntAddon>() );
}

TEST(Addon, Construction)
{
  AddonManager mgr;

  mgr.create<DoubleAddon>();

  double s = dart::math::random(0, 100);
  mgr.create<DoubleAddon>(s);
  EXPECT_EQ(mgr.get<DoubleAddon>()->mState.val, s);

  double p = dart::math::random(0, 100);
  mgr.create<DoubleAddon>(dart::math::random(0, 100), p);
  EXPECT_EQ(mgr.get<DoubleAddon>()->mProperties.val, p);
}

TEST(Addon, Joints)
{
  usedSpecializedAddonAccess = false;

  dart::dynamics::SkeletonPtr skel = Skeleton::create();

  dart::dynamics::EulerJoint* euler =
      skel->createJointAndBodyNodePair<dart::dynamics::EulerJoint>().first;
  euler->getMultiDofJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
  euler->getEulerJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;

  dart::dynamics::PlanarJoint* planar =
      skel->createJointAndBodyNodePair<dart::dynamics::PlanarJoint>().first;
  planar->getMultiDofJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
  planar->getPlanarJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;

  dart::dynamics::PrismaticJoint* prismatic =
      skel->createJointAndBodyNodePair<dart::dynamics::PrismaticJoint>().first;
  prismatic->getSingleDofJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
  prismatic->getPrismaticJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;

  dart::dynamics::RevoluteJoint* revolute =
      skel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>().first;
  revolute->getSingleDofJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
  revolute->getRevoluteJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;

  dart::dynamics::ScrewJoint* screw =
      skel->createJointAndBodyNodePair<dart::dynamics::ScrewJoint>().first;
  screw->getSingleDofJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
  screw->getScrewJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;

  dart::dynamics::UniversalJoint* universal =
      skel->createJointAndBodyNodePair<dart::dynamics::UniversalJoint>().first;
  universal->getMultiDofJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
  universal->getUniversalJointAddon();
  EXPECT_TRUE(usedSpecializedAddonAccess); usedSpecializedAddonAccess = false;
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
