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

#include <gtest/gtest.h>

#include "TestHelpers.h"

#include "dart/common/Subject.h"
#include "dart/common/sub_ptr.h"
#include "dart/common/AddonManager.h"

using namespace dart::common;

class GenericAddon : public Addon, public Subject
{
public:

  GenericAddon(AddonManager* manager)
    : Addon(manager, "GenericAddon")
  {
    // Do nothing
  }

  std::unique_ptr<Addon> clone(AddonManager* newManager) const override final
  {
    return std::unique_ptr<GenericAddon>(new GenericAddon(newManager));
  }

};

class SpecializedAddon : public Addon, public Subject
{
public:

  SpecializedAddon(AddonManager* manager)
    : Addon(manager, "SpecializedAddon")
  {
    // Do nothing
  }

  std::unique_ptr<Addon> clone(AddonManager* newManager) const override final
  {
    return std::unique_ptr<SpecializedAddon>(new SpecializedAddon(newManager));
  }
};

class StatefulAddon : public Addon, public Subject
{
public:

  class State : public Addon::State
  {
  public:

    State() : val(dart::math::random(0, 100)) { }

    State(const State& other) : val(other.val) { }

    double val;

    std::unique_ptr<Addon::State> clone() const override
    {
      return std::unique_ptr<Addon::State>(new State(*this));
    }

    void copy(const Addon::State& anotherState) override
    {
      val = static_cast<const State&>(anotherState).val;
    }
  };

};

class SpecializedManager : public AddonManager
{
public:

  SpecializedManager()
  {
    DART_INSTANTIATE_ADDON( SpecializedAddon, mSpecializedAddon );
  }

  DART_SPECIALIZED_ADDON( SpecializedAddon, mSpecializedAddon )

protected:

  AddonMap::iterator mSpecializedAddon;

};

TEST(Addon, Generic)
{
  AddonManager mgr;

  EXPECT_TRUE( mgr.get<GenericAddon>() == nullptr );

  sub_ptr<GenericAddon> addon = mgr.construct<GenericAddon>();
  GenericAddon* rawAddon = addon;
  EXPECT_FALSE( nullptr == addon );
  EXPECT_TRUE( mgr.get<GenericAddon>() == addon );

  GenericAddon* newAddon = mgr.construct<GenericAddon>();
  EXPECT_FALSE( nullptr == newAddon );
  EXPECT_TRUE( nullptr == addon );
  EXPECT_FALSE( rawAddon == newAddon );
}

TEST(Addon, Specialized)
{
  SpecializedManager mgr;

  EXPECT_TRUE( mgr.get<SpecializedAddon>() == nullptr );
  EXPECT_TRUE( mgr.getSpecializedAddon() == nullptr );

  sub_ptr<SpecializedAddon> spec = mgr.construct<SpecializedAddon>();
  SpecializedAddon* rawSpec = spec;

  EXPECT_TRUE( mgr.get<SpecializedAddon>() == spec );
  EXPECT_TRUE( mgr.getSpecializedAddon() == spec );

  SpecializedAddon* newSpec = mgr.constructSpecializedAddon();

  EXPECT_TRUE( nullptr == spec );

  EXPECT_FALSE( mgr.get<SpecializedAddon>() == rawSpec );
  EXPECT_FALSE( mgr.getSpecializedAddon() == rawSpec );

  EXPECT_TRUE( mgr.get<SpecializedAddon>() == newSpec );
  EXPECT_TRUE( mgr.getSpecializedAddon() == newSpec );
}

TEST(Addon, Releasing)
{
  AddonManager sender;
  SpecializedManager receiver;

  // ---- Test generic releases ----
  EXPECT_TRUE( sender.get<GenericAddon>() == nullptr );
  EXPECT_TRUE( receiver.get<GenericAddon>() == nullptr );

  sub_ptr<GenericAddon> addon = sender.construct<GenericAddon>();

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


  // ---- Test specialized releases ----
  EXPECT_TRUE( sender.get<SpecializedAddon>() == nullptr );
  EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );

  sub_ptr<SpecializedAddon> spec = sender.construct<SpecializedAddon>();

  EXPECT_TRUE( sender.get<SpecializedAddon>() == spec );
  EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );

  receiver.setSpecializedAddon(sender.release<SpecializedAddon>());

  EXPECT_FALSE( nullptr == spec );

  EXPECT_TRUE( sender.get<SpecializedAddon>() == nullptr );
  EXPECT_TRUE( receiver.getSpecializedAddon() == spec );

  sender.set<SpecializedAddon>(receiver.releaseSpecializedAddon());

  EXPECT_FALSE( nullptr == spec );

  EXPECT_TRUE( sender.get<SpecializedAddon>() == spec );
  EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );

  sender.release<SpecializedAddon>();

  EXPECT_TRUE( nullptr == spec );
  EXPECT_TRUE( sender.get<SpecializedAddon>() == nullptr );
  EXPECT_TRUE( receiver.getSpecializedAddon() == nullptr );
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
