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

#include <dart/constraint/ContactPatchCache.hpp>

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/Contact.hpp>

#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include <cstddef>
#include <vector>

using namespace dart;

namespace {

class DummyCollisionObject : public collision::CollisionObject
{
public:
  DummyCollisionObject() : CollisionObject(nullptr, nullptr) {}

private:
  void updateEngineData() override {}
};

class RawCollisionResult : public collision::CollisionResult
{
public:
  void reserve(std::size_t count)
  {
    mContacts.reserve(count);
  }

  void addContactRaw(const collision::Contact& contact)
  {
    mContacts.push_back(contact);
  }

  void clearRaw()
  {
    mContacts.clear();
  }
};

collision::Contact makeContact(
    collision::CollisionObject* objA,
    collision::CollisionObject* objB,
    const Eigen::Vector3d& point,
    double depth)
{
  collision::Contact contact;
  contact.collisionObject1 = objA;
  contact.collisionObject2 = objB;
  contact.point = point;
  contact.normal = Eigen::Vector3d::UnitZ();
  contact.penetrationDepth = depth;
  return contact;
}

void buildContacts(
    std::vector<DummyCollisionObject>& objects,
    std::size_t pairCount,
    std::size_t contactsPerPair,
    RawCollisionResult& result)
{
  result.clearRaw();
  result.reserve(pairCount * contactsPerPair);

  for (std::size_t pairIndex = 0u; pairIndex < pairCount; ++pairIndex) {
    auto* objA = &objects[2u * pairIndex];
    auto* objB = &objects[2u * pairIndex + 1u];

    for (std::size_t contactIndex = 0u; contactIndex < contactsPerPair;
         ++contactIndex) {
      const double depth = 0.01 * static_cast<double>(contactIndex + 1u);
      const double x = static_cast<double>(pairIndex);
      const double y = static_cast<double>(contactIndex) * 0.01;
      result.addContactRaw(
          makeContact(objA, objB, Eigen::Vector3d(x, y, 0.0), depth));
    }
  }
}

} // namespace

static void BM_ContactPatchCacheUpdate(benchmark::State& state)
{
  const auto pairCount = static_cast<std::size_t>(state.range(0));
  const auto contactsPerPair = static_cast<std::size_t>(state.range(1));

  std::vector<DummyCollisionObject> objects(pairCount * 2u);
  RawCollisionResult raw;
  buildContacts(objects, pairCount, contactsPerPair, raw);

  constraint::ContactPatchCache cache;
  constraint::ContactPatchCacheOptions options;
  options.enabled = true;
  options.maxPointsPerPair = 4u;

  std::vector<collision::Contact> output;
  output.reserve(pairCount * options.maxPointsPerPair);

  cache.update(raw, options, output);
  benchmark::ClobberMemory();

  for (auto _ : state) {
    cache.update(raw, options, output);
    benchmark::DoNotOptimize(output.data());
  }

  state.SetItemsProcessed(
      state.iterations() * pairCount * contactsPerPair);
}

BENCHMARK(BM_ContactPatchCacheUpdate)
    ->Args({1, 4})
    ->Args({1, 16})
    ->Args({10, 4})
    ->Args({10, 16})
    ->Args({100, 4})
    ->Args({100, 16})
    ->Unit(benchmark::kMicrosecond);
