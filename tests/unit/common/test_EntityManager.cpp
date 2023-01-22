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

#include "dart/common/common.hpp"

#include <gtest/gtest.h>

using namespace dart;
using namespace common;

namespace {

struct position
{
  float x;
  float y;
};

struct velocity
{
  float dx;
  float dy;
};

} // namespace

//==============================================================================
TEST(EntityManagerTest, Basics)
{
  AlignedAllocatorRaw alloc = AlignedAllocatorRaw();

  auto registry = EntityManager(alloc);
  (void)registry;

  auto view = registry.view<const position, velocity>();

  // use a callback
  view.each([](const auto& pos, auto& vel) {
    (void)pos;
    (void)vel;
  });

  // use an extended callback
  view.each([](const auto entity, const auto& pos, auto& vel) {
    (void)entity;
    (void)pos;
    (void)vel;
  });

  // use a range-for
  for (auto [entity, pos, vel] : view.each()) {
    (void)entity;
    (void)pos;
    (void)vel;
  }

  // use forward iterators and get only the components of interest
  for (auto entity : view) {
    auto& vel = view.get<velocity>(entity);
    (void)vel;
  }
}
