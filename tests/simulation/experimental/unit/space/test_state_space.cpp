/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/simulation/experimental/common/constants.hpp"
#include "dart/simulation/experimental/space/state_space.hpp"

#include <gtest/gtest.h>

#include <span>
#include <vector>

#include <cmath>

using namespace dart::simulation::experimental;

TEST(StateSpace, BasicConstruction)
{
  StateSpace space;
  EXPECT_EQ(space.getDimension(), 0);
  EXPECT_EQ(space.getNumVariables(), 0);
  EXPECT_FALSE(space.isFinalized());
}

TEST(StateSpace, AddSingleVariable)
{
  StateSpace space;
  space.addVariable("pos", 3, -1.0, 1.0);

  EXPECT_EQ(space.getDimension(), 3);
  EXPECT_EQ(space.getNumVariables(), 1);
  EXPECT_TRUE(space.hasVariable("pos"));
  EXPECT_FALSE(space.hasVariable("nonexistent"));
}

TEST(StateSpace, AddMultipleVariables)
{
  StateSpace space;
  space.addVariable("pos", 3, -1.0, 1.0);
  space.addVariable("vel", 3, -10.0, 10.0);

  EXPECT_EQ(space.getDimension(), 6);
  EXPECT_EQ(space.getNumVariables(), 2);

  auto var1 = space.getVariable("pos");
  ASSERT_TRUE(var1.has_value());
  EXPECT_EQ(var1->name, "pos");
  EXPECT_EQ(var1->dimension, 3);
  EXPECT_EQ(var1->startIndex, 0);
  EXPECT_DOUBLE_EQ(var1->lowerBound, -1.0);
  EXPECT_DOUBLE_EQ(var1->upperBound, 1.0);

  auto var2 = space.getVariable("vel");
  ASSERT_TRUE(var2.has_value());
  EXPECT_EQ(var2->name, "vel");
  EXPECT_EQ(var2->dimension, 3);
  EXPECT_EQ(var2->startIndex, 3);
  EXPECT_DOUBLE_EQ(var2->lowerBound, -10.0);
  EXPECT_DOUBLE_EQ(var2->upperBound, 10.0);
}

TEST(StateSpace, MethodChaining)
{
  StateSpace space;
  space.addVariable("x", 1).addVariable("y", 1).addVariable("z", 1);

  EXPECT_EQ(space.getDimension(), 3);
  EXPECT_EQ(space.getNumVariables(), 3);
}

TEST(StateSpace, AddMultipleScalarsHelper)
{
  StateSpace space;
  const std::vector<std::string> names = {"x", "y", "z"};
  space.addVariables(std::span<const std::string>(names), -1.0, 1.0);

  EXPECT_EQ(space.getDimension(), 3);
  EXPECT_EQ(space.getNumVariables(), 3);

  auto varX = space.getVariable("x");
  ASSERT_TRUE(varX.has_value());
  EXPECT_EQ(varX->dimension, 1);
}

TEST(StateSpace, GetVariableNames)
{
  StateSpace space;
  space.addVariable("pos", 3);
  space.addVariable("vel", 3);
  space.addVariable("force", 3);

  auto names = space.getVariableNames();
  ASSERT_EQ(names.size(), 3);
  EXPECT_EQ(names[0], "pos");
  EXPECT_EQ(names[1], "vel");
  EXPECT_EQ(names[2], "force");
}

TEST(StateSpace, GetBounds)
{
  StateSpace space;
  space.addVariable(
      "pos",
      2,
      -dart::simulation::experimental::pi,
      dart::simulation::experimental::pi);
  space.addVariable("vel", 2, -10.0, 10.0);

  auto lowerBounds = space.getLowerBounds();
  auto upperBounds = space.getUpperBounds();

  ASSERT_EQ(lowerBounds.size(), 4);
  ASSERT_EQ(upperBounds.size(), 4);

  // First variable (pos)
  EXPECT_DOUBLE_EQ(lowerBounds[0], -dart::simulation::experimental::pi);
  EXPECT_DOUBLE_EQ(lowerBounds[1], -dart::simulation::experimental::pi);
  EXPECT_DOUBLE_EQ(upperBounds[0], dart::simulation::experimental::pi);
  EXPECT_DOUBLE_EQ(upperBounds[1], dart::simulation::experimental::pi);

  // Second variable (vel)
  EXPECT_DOUBLE_EQ(lowerBounds[2], -10.0);
  EXPECT_DOUBLE_EQ(lowerBounds[3], -10.0);
  EXPECT_DOUBLE_EQ(upperBounds[2], 10.0);
  EXPECT_DOUBLE_EQ(upperBounds[3], 10.0);
}

TEST(StateSpace, Finalization)
{
  StateSpace space;
  space.addVariable("pos", 3);

  EXPECT_FALSE(space.isFinalized());

  space.finalize();
  EXPECT_TRUE(space.isFinalized());

  // Should not throw if finalized again
  EXPECT_NO_THROW(space.finalize());

  // Should throw when trying to add after finalization
  EXPECT_THROW(space.addVariable("vel", 3), std::logic_error);
}

TEST(StateSpace, DuplicateVariableName)
{
  StateSpace space;
  space.addVariable("pos", 3);

  EXPECT_THROW(space.addVariable("pos", 3), std::invalid_argument);
}

TEST(StateSpace, ZeroDimensionVariable)
{
  StateSpace space;
  EXPECT_THROW(space.addVariable("bad", 0), std::invalid_argument);
}

TEST(StateSpace, InvalidBounds)
{
  StateSpace space;
  // Lower > upper
  EXPECT_THROW(space.addVariable("bad", 3, 10.0, -10.0), std::invalid_argument);
}

TEST(StateSpace, GetVariableIndex)
{
  StateSpace space;
  space.addVariable("pos", 3);
  space.addVariable("vel", 3);

  auto idx0 = space.getVariableIndex("pos");
  auto idx1 = space.getVariableIndex("vel");
  auto idx2 = space.getVariableIndex("nonexistent");

  ASSERT_TRUE(idx0.has_value());
  ASSERT_TRUE(idx1.has_value());
  EXPECT_FALSE(idx2.has_value());

  EXPECT_EQ(*idx0, 0);
  EXPECT_EQ(*idx1, 1);
}

TEST(StateSpace, UnboundedVariables)
{
  StateSpace space;
  space.addVariable("unbounded", 2);

  auto lowerBounds = space.getLowerBounds();
  auto upperBounds = space.getUpperBounds();

  EXPECT_TRUE(std::isinf(lowerBounds[0]));
  EXPECT_TRUE(lowerBounds[0] < 0); // Negative infinity
  EXPECT_TRUE(std::isinf(upperBounds[0]));
  EXPECT_TRUE(upperBounds[0] > 0); // Positive infinity
}
