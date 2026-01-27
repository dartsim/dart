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

#include <dart/constraint/constrained_group.hpp>
#include <dart/constraint/constraint_base.hpp>

#include <gtest/gtest.h>

namespace dart::constraint {
namespace {

class TestConstraint final : public ConstraintBase
{
public:
  explicit TestConstraint(std::size_t dim)
  {
    mDim = dim;
  }

  void update() override {}

  void getInformation(ConstraintInfo* /*info*/) override {}

  void applyUnitImpulse(std::size_t /*index*/) override {}

  void getVelocityChange(double* /*vel*/, bool /*withCfm*/) override {}

  void excite() override {}

  void unexcite() override {}

  void applyImpulse(double* /*lambda*/) override {}

  bool isActive() const override
  {
    return true;
  }

  dynamics::SkeletonPtr getRootSkeleton() const override
  {
    return nullptr;
  }
};

} // namespace

TEST(ConstrainedGroupTest, AddRemoveAndTotals)
{
  ConstrainedGroup group;

  auto c1 = std::make_shared<TestConstraint>(2);
  auto c2 = std::make_shared<TestConstraint>(3);

  group.addConstraint(c1);
  group.addConstraint(c2);

  EXPECT_EQ(group.getNumConstraints(), 2u);
  EXPECT_EQ(group.getTotalDimension(), 5u);

  EXPECT_EQ(group.getConstraint(0), c1);
  const ConstrainedGroup& constGroup = group;
  EXPECT_EQ(constGroup.getConstraint(1), c2);

  group.removeConstraint(c1);
  EXPECT_EQ(group.getNumConstraints(), 1u);
  EXPECT_EQ(group.getTotalDimension(), 3u);
  EXPECT_EQ(group.getConstraint(0), c2);

  group.removeAllConstraints();
  EXPECT_EQ(group.getNumConstraints(), 0u);
}

} // namespace dart::constraint
