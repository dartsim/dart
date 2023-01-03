/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/dynamics/ConstraintSolver.hpp"
#include "dart/dynamics/ContactSurface.hpp"
#include "dart/simulation/World.hpp"

#include <dart/test/io/TestHelpers.hpp>

#include <gtest/gtest.h>

using namespace dart;

//==============================================================================
std::shared_ptr<World> createWorld()
{
  return simulation::World::create();
}

//==============================================================================
TEST(ConstraintSolver, DefaultConstactSurfaceHandler)
{
  auto world = createWorld();
  auto solver = world->getConstraintSolver();
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
}

//==============================================================================
TEST(ConstraintSolver, CustomConstactSurfaceHandler)
{
  class CustomHandler : public dynamics::ContactSurfaceHandler
  {
  public:
    dynamics::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      auto params = ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
      params.mFirstFrictionalDirection = Eigen::Vector3d::UnitY();
      params.mContactSurfaceMotionVelocity = Eigen::Vector3d::UnitY();
      return params;
    }
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<CustomHandler>();
  solver->addContactSurfaceHandler(customHandler);

  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());

  // try to remove nonexisting handler
  EXPECT_FALSE(
      solver->removeContactSurfaceHandler(std::make_shared<CustomHandler>()));

  EXPECT_TRUE(solver->removeContactSurfaceHandler(defaultHandler));
  EXPECT_EQ(nullptr, customHandler->getParent());
  EXPECT_EQ(customHandler, solver->getLastContactSurfaceHandler());

  // removing last handler should not be done, but we test it anyways
  // a printed error message is expected
  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  EXPECT_EQ(nullptr, customHandler->getParent());
  EXPECT_EQ(nullptr, solver->getLastContactSurfaceHandler());

  solver->addContactSurfaceHandler(defaultHandler);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(defaultHandler, solver->getLastContactSurfaceHandler());

  solver->addContactSurfaceHandler(customHandler);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());

  auto customHandler2 = std::make_shared<CustomHandler>();
  auto customHandler3 = std::make_shared<CustomHandler>();
  solver->addContactSurfaceHandler(customHandler2);
  solver->addContactSurfaceHandler(customHandler3);
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler3, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(customHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler));
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler3, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  EXPECT_TRUE(solver->removeContactSurfaceHandler(customHandler3));
  ASSERT_NE(nullptr, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(customHandler2, solver->getLastContactSurfaceHandler());
  EXPECT_EQ(nullptr, defaultHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler->getParent());
  EXPECT_EQ(defaultHandler, customHandler2->getParent());
  EXPECT_EQ(customHandler2, customHandler3->getParent());

  // after we break the chain at handler 2, default handler is no longer
  // reachable
  customHandler2->setParent(nullptr);
  EXPECT_FALSE(solver->removeContactSurfaceHandler(defaultHandler));
}

//==============================================================================
TEST(ConstraintSolver, ConstactSurfaceHandlerIsCalled)
{
  class ValueHandler : public dynamics::ContactSurfaceHandler
  {
  public:
    ValueHandler(int value) : mValue(value)
    {
      // Do nothing
    }

    dynamics::ContactSurfaceParams createParams(
        const collision::Contact& contact,
        const size_t numContactsOnCollisionObject) const override
    {
      auto params = ContactSurfaceHandler::createParams(
          contact, numContactsOnCollisionObject);
      mCalled = true;
      params.mPrimaryFrictionCoeff = mValue;

      return params;
    }

    mutable bool mCalled{false};
    int mValue{0};
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<ValueHandler>(1);
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  customHandler->mCalled = false;
  auto params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_EQ(1, params.mPrimaryFrictionCoeff);

  auto customHandler2 = std::make_shared<ValueHandler>(2);
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);

  // Try once more adding the same handler instance; this should be ignored.
  // If it were added, the createParams() call could get into an infinite loop
  // calling the last handler as its parent, so rather check for it.
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);
}

//==============================================================================
TEST(ConstraintSolver, ConstactSurfaceHandlerIgnoreParent)
{
  class IgnoreParentHandler : public dynamics::ContactSurfaceHandler
  {
  public:
    IgnoreParentHandler(int value) : mValue(value)
    {
      // Do nothing
    }

    dynamics::ContactSurfaceParams createParams(
        const collision::Contact& /*contact*/,
        const size_t /*numContactsOnCollisionObject*/) const override
    {
      auto params = dynamics::ContactSurfaceParams{};
      mCalled = true;
      params.mPrimaryFrictionCoeff = mValue;

      return params;
    }

    mutable bool mCalled{false};
    int mValue{0};
  };

  auto world = createWorld();

  auto solver = world->getConstraintSolver();
  auto defaultHandler = solver->getLastContactSurfaceHandler();
  EXPECT_EQ(nullptr, defaultHandler->getParent());

  auto customHandler = std::make_shared<IgnoreParentHandler>(1);
  solver->addContactSurfaceHandler(customHandler);
  solver->removeContactSurfaceHandler(defaultHandler);

  customHandler->mCalled = false;
  auto params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_TRUE(customHandler->mCalled);
  EXPECT_EQ(1, params.mPrimaryFrictionCoeff);

  auto customHandler2 = std::make_shared<IgnoreParentHandler>(2);
  solver->addContactSurfaceHandler(customHandler2);

  customHandler->mCalled = customHandler2->mCalled = false;
  params = solver->getLastContactSurfaceHandler()->createParams({}, 0);
  EXPECT_FALSE(customHandler->mCalled);
  EXPECT_TRUE(customHandler2->mCalled);
  EXPECT_EQ(2, params.mPrimaryFrictionCoeff);
}
