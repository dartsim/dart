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

// This test exercises the OpenUSD scene loader end-to-end through the DART 7
// model-loading front door. It is only compiled when DART is built with
// DART_BUILD_IO_USD=ON (which requires OpenUSD/pxr); the registration in
// tests/integration/CMakeLists.txt is gated on that toggle, so this file
// compiles out cleanly in the default build where OpenUSD is absent.

#include <dart/simulation/io/skeleton_loader.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <gtest/gtest.h>

namespace sx = dart::simulation;

//==============================================================================
TEST(UsdLoader, AddsSimpleChainLinkToWorld)
{
  sx::World world;

  const sx::Multibody multibody
      = sx::io::addSkeleton(world, "dart://sample/usd/simple_chain.usda");

  // The minimal USD sample maps to a one-link multibody anchored on the stage
  // default prim ("simple_chain").
  EXPECT_EQ(multibody.getName(), "simple_chain");
  EXPECT_GT(multibody.getLinkCount(), 0u);
  EXPECT_EQ(world.getMultibodyCount(), 1u);
}
