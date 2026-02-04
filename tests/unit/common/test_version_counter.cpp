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

#include <dart/common/version_counter.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::common;

class TestVersionCounter : public VersionCounter
{
public:
  using VersionCounter::setVersionDependentObject;
};

TEST(VersionCounter, DefaultConstruction)
{
  VersionCounter counter;
  EXPECT_EQ(counter.getVersion(), 0u);
}

TEST(VersionCounter, IncrementVersion)
{
  VersionCounter counter;
  EXPECT_EQ(counter.getVersion(), 0u);

  std::size_t newVersion = counter.incrementVersion();
  EXPECT_EQ(newVersion, 1u);
  EXPECT_EQ(counter.getVersion(), 1u);
}

TEST(VersionCounter, MultipleIncrements)
{
  VersionCounter counter;

  for (std::size_t i = 1; i <= 10; ++i) {
    std::size_t version = counter.incrementVersion();
    EXPECT_EQ(version, i);
    EXPECT_EQ(counter.getVersion(), i);
  }
}

TEST(VersionCounter, IndependentCounters)
{
  VersionCounter counter1;
  VersionCounter counter2;

  counter1.incrementVersion();
  counter1.incrementVersion();

  counter2.incrementVersion();

  EXPECT_EQ(counter1.getVersion(), 2u);
  EXPECT_EQ(counter2.getVersion(), 1u);
}

TEST(VersionCounter, DependentObjectPropagation)
{
  TestVersionCounter parent;
  TestVersionCounter child;

  child.setVersionDependentObject(&parent);

  std::size_t initialParentVersion = parent.getVersion();
  child.incrementVersion();

  EXPECT_GT(parent.getVersion(), initialParentVersion);
}

TEST(VersionCounter, ChainedDependencies)
{
  TestVersionCounter grandparent;
  TestVersionCounter parent;
  TestVersionCounter child;

  parent.setVersionDependentObject(&grandparent);
  child.setVersionDependentObject(&parent);

  std::size_t initialGrandparentVersion = grandparent.getVersion();
  std::size_t initialParentVersion = parent.getVersion();

  child.incrementVersion();

  EXPECT_GT(parent.getVersion(), initialParentVersion);
  EXPECT_GT(grandparent.getVersion(), initialGrandparentVersion);
}
