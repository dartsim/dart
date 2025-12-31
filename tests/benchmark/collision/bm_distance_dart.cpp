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

#include <dart/collision/dart/All.hpp>
#include <dart/dynamics/All.hpp>

#include <benchmark/benchmark.h>

#include <vector>

using namespace dart;

namespace {

struct DistanceScene
{
  std::shared_ptr<collision::CollisionDetector> detector;
  std::unique_ptr<collision::CollisionGroup> group;
  std::vector<std::shared_ptr<dynamics::SimpleFrame>> frames;
};

DistanceScene createScene(std::size_t count)
{
  DistanceScene scene;
  scene.detector = collision::DARTCollisionDetector::create();
  scene.group = scene.detector->createCollisionGroup();
  scene.frames.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    auto frame = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
    frame->setShape(std::make_shared<dynamics::SphereShape>(0.5));
    frame->setTranslation(Eigen::Vector3d(static_cast<double>(i) * 2.0, 0.0, 0.0));
    scene.group->addShapeFrame(frame.get());
    scene.frames.push_back(std::move(frame));
  }

  return scene;
}

} // namespace

static void BM_DistanceDart(benchmark::State& state)
{
  auto scene = createScene(static_cast<std::size_t>(state.range(0)));
  collision::DistanceOption option(false, 0.0, nullptr);
  collision::DistanceResult result;

  for (auto _ : state) {
    scene.group->distance(option, &result);
    benchmark::DoNotOptimize(result.minDistance);
  }
}

BENCHMARK(BM_DistanceDart)
    ->Arg(32)
    ->Arg(128)
    ->Arg(512)
    ->Unit(benchmark::kMicrosecond);
