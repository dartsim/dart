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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dartsim_engine/sim_engine.hpp>
#include <dartsim_ui/memory_actions.hpp>
#include <dartsim_ui/palette_actions.hpp>
#include <gtest/gtest.h>

#include <string>

using namespace dartsim;

TEST(DartsimMemoryActions, BuildsAllocatorAndFrameScratchStatus)
{
  SimEngine engine;
  const ui::MemoryStatus status = ui::buildMemoryStatus(engine);

  EXPECT_GT(status.frameScratchCapacityBytes, 0u);
  EXPECT_GE(status.frameScratchPeakUsedBytes, status.frameScratchUsedBytes);
  EXPECT_NE(status.frameScratchLabel.find("Frame scratch"), std::string::npos);
  EXPECT_NE(
      status.allocatorDebugLabel.find("Allocator debug counters"),
      std::string::npos);
  EXPECT_NE(status.ecsSummaryLabel.find("ECS:"), std::string::npos);

  ASSERT_EQ(status.allocators.size(), 2u);
  EXPECT_EQ(status.allocators[0].name, "free list");
  EXPECT_EQ(status.allocators[1].name, "pool");
}

TEST(DartsimMemoryActions, ReportsLargestEcsStorageRows)
{
  SimEngine engine;
  ASSERT_TRUE(
      ui::applyPaletteAction(
          engine, ui::PaletteActionKind::AddGroundAndBoxExample)
          .ok);

  const ui::MemoryStatus status = ui::buildMemoryStatus(engine, 3);

  EXPECT_GT(status.ecsEntityCount, 0u);
  EXPECT_GT(status.ecsStorageCount, 0u);
  EXPECT_LE(status.largestStorages.size(), 3u);
  ASSERT_FALSE(status.largestStorages.empty());

  for (std::size_t i = 1; i < status.largestStorages.size(); ++i) {
    EXPECT_GE(
        status.largestStorages[i - 1].capacity,
        status.largestStorages[i].capacity);
  }

  const ui::MemoryStatus hiddenStorages = ui::buildMemoryStatus(engine, 0);
  EXPECT_TRUE(hiddenStorages.largestStorages.empty());
  EXPECT_EQ(hiddenStorages.ecsStorageCount, status.ecsStorageCount);
}
