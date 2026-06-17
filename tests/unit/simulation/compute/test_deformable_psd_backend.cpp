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

#include <dart/simulation/compute/deformable_psd_backend.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/newton_barrier/psd_backend.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <string_view>
#include <thread>

#include <cstddef>

namespace {

using namespace dart::simulation::compute;
namespace nb = dart::simulation::detail::newton_barrier;

// A fake accelerator backed by file-static state, registered through the
// backend-neutral hook with plain function pointers. No device technology is
// involved, so the core acceleration control is exercised on any host.
std::atomic<bool> g_available{false};
std::atomic<int> g_fakeProjectCalls{0};
std::atomic<int> g_releaseCalls{0};

bool fakeAvailable()
{
  return g_available.load(std::memory_order_relaxed);
}

void fakeRelease()
{
  g_releaseCalls.fetch_add(1, std::memory_order_relaxed);
}

void markerProjector(double* blocks, std::size_t, std::size_t blockCount)
{
  // A distinct non-CPU projector so install/restore is observable.
  g_fakeProjectCalls.fetch_add(1, std::memory_order_relaxed);
  if (blocks != nullptr && blockCount > 0) {
    blocks[0] = 42.0;
  }
}

class PsdProbeStage final : public dart::simulation::compute::WorldStepStage
{
public:
  explicit PsdProbeStage(bool expectAccelerated)
    : m_expectAccelerated(expectAccelerated)
  {
    // Empty.
  }

  [[nodiscard]] std::string_view getName() const noexcept override
  {
    return "psd_probe";
  }

  void execute(
      dart::simulation::World&,
      dart::simulation::compute::ComputeExecutor&) override
  {
    std::array<double, 4> block{-1.0, 0.0, 0.0, 2.0};
    projectSymmetricBlocksToPsd(block.data(), 2, 1);
    m_observed = block[0];

    const bool accelerated = m_observed == 42.0;
    if (accelerated != m_expectAccelerated) {
      m_allObservedExpected.store(false, std::memory_order_relaxed);
    }
  }

  [[nodiscard]] double observed() const noexcept
  {
    return m_observed;
  }

  [[nodiscard]] bool allObservedExpected() const noexcept
  {
    return m_allObservedExpected.load(std::memory_order_relaxed);
  }

private:
  bool m_expectAccelerated;
  double m_observed = 0.0;
  std::atomic<bool> m_allObservedExpected{true};
};

void registerFakeAccelerator(bool available)
{
  g_available.store(available, std::memory_order_relaxed);
  setDeformablePsdAccelerator({&fakeAvailable, &markerProjector, &fakeRelease});
}

class DeformablePsdAcceleratorTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // Clear any registered accelerator and return to the CPU backend so tests
    // do not leak the process-wide acceleration state into one another.
    setDeformablePsdAccelerated(false);
    setDeformablePsdAccelerator(DeformablePsdAccelerator{});
    setDeformablePsdBlockProjector(nullptr);
    g_available.store(false, std::memory_order_relaxed);
    g_fakeProjectCalls.store(0, std::memory_order_relaxed);
    g_releaseCalls.store(0, std::memory_order_relaxed);
  }
};

TEST_F(DeformablePsdAcceleratorTest, NoAcceleratorRegisteredIsSafeNoOp)
{
  setDeformablePsdAccelerator(DeformablePsdAccelerator{});
  EXPECT_FALSE(isDeformablePsdAccelerationAvailable());
  EXPECT_EQ(deformablePsdAcceleratorProjector(), nullptr);
  EXPECT_FALSE(setDeformablePsdAccelerated(true));
  EXPECT_FALSE(isDeformablePsdAccelerated());
  EXPECT_EQ(deformablePsdBlockProjector(), &projectSymmetricBlocksToPsdCpu);
}

TEST_F(DeformablePsdAcceleratorTest, RegisteredButUnavailableStaysOnCpu)
{
  registerFakeAccelerator(false);
  EXPECT_FALSE(isDeformablePsdAccelerationAvailable());
  EXPECT_EQ(deformablePsdAcceleratorProjector(), nullptr);
  EXPECT_FALSE(setDeformablePsdAccelerated(true));
  EXPECT_FALSE(isDeformablePsdAccelerated());
  EXPECT_EQ(g_fakeProjectCalls.load(std::memory_order_relaxed), 0);
}

TEST_F(DeformablePsdAcceleratorTest, EnableAndDisableToggleTheAccelerator)
{
  registerFakeAccelerator(true);
  EXPECT_TRUE(isDeformablePsdAccelerationAvailable());
  EXPECT_EQ(deformablePsdAcceleratorProjector(), &markerProjector);

  EXPECT_TRUE(setDeformablePsdAccelerated(true));
  EXPECT_TRUE(isDeformablePsdAccelerated());
  std::array<double, 4> block{-1.0, 0.0, 0.0, 2.0};
  projectSymmetricBlocksToPsd(block.data(), 2, 1);
  EXPECT_EQ(block[0], 42.0);

  EXPECT_FALSE(setDeformablePsdAccelerated(false));
  EXPECT_FALSE(isDeformablePsdAccelerated());
  EXPECT_EQ(g_releaseCalls.load(std::memory_order_relaxed), 1);
}

TEST_F(DeformablePsdAcceleratorTest, DisableWithoutHookRestoresCpuProjector)
{
  // An accelerator with no disable hook still restores the CPU projector when
  // acceleration is turned off.
  setDeformablePsdBlockProjector(&markerProjector);
  g_available.store(true, std::memory_order_relaxed);
  setDeformablePsdAccelerator({&fakeAvailable, &markerProjector, nullptr});
  EXPECT_FALSE(setDeformablePsdAccelerated(false));
  EXPECT_EQ(deformablePsdBlockProjector(), &projectSymmetricBlocksToPsdCpu);
}

TEST_F(DeformablePsdAcceleratorTest, ScopedProjectorOverridesOnlyCurrentThread)
{
  setDeformablePsdBlockProjector(nullptr);

  {
    ScopedDeformablePsdBlockProjector scope(&markerProjector);

    std::array<double, 4> scopedBlock{-1.0, 0.0, 0.0, 2.0};
    projectSymmetricBlocksToPsd(scopedBlock.data(), 2, 1);
    EXPECT_EQ(scopedBlock[0], 42.0);
    EXPECT_TRUE(isDeformablePsdAccelerated());

    std::array<double, 4> threadBlock{-1.0, 0.0, 0.0, 2.0};
    bool workerSawAcceleration = true;
    std::thread worker([&threadBlock, &workerSawAcceleration] {
      projectSymmetricBlocksToPsd(threadBlock.data(), 2, 1);
      workerSawAcceleration = isDeformablePsdAccelerated();
    });
    worker.join();
    EXPECT_FALSE(workerSawAcceleration);
    EXPECT_NEAR(threadBlock[0], 0.0, 1e-9);
  }

  EXPECT_FALSE(isDeformablePsdAccelerated());
}

TEST_F(DeformablePsdAcceleratorTest, NewtonBarrierWrapperRoutesCoreBackend)
{
  EXPECT_EQ(nb::psdBlockProjector(), &projectSymmetricBlocksToPsdCpu);
  EXPECT_EQ(
      nb::setPsdBlockProjector(&markerProjector),
      &projectSymmetricBlocksToPsdCpu);
  EXPECT_EQ(deformablePsdBlockProjector(), &markerProjector);
  EXPECT_EQ(nb::setPsdBlockProjector(nullptr), &markerProjector);
  EXPECT_EQ(nb::psdBlockProjector(), &projectSymmetricBlocksToPsdCpu);

  std::array<double, 4> block{-1.0, 0.0, 0.0, 2.0};
  nb::projectSymmetricBlocksToPsd(block.data(), 2, 1);
  EXPECT_NEAR(block[0], 0.0, 1e-9);
  EXPECT_NEAR(block[1], 0.0, 1e-9);
  EXPECT_NEAR(block[2], 0.0, 1e-9);
  EXPECT_NEAR(block[3], 2.0, 1e-9);
}

TEST_F(DeformablePsdAcceleratorTest, WorldsResolveDifferentAcceleratorPolicies)
{
  registerFakeAccelerator(true);

  dart::simulation::World cpuWorld;
  PsdProbeStage cpuProbe(/*expectAccelerated=*/false);
  dart::simulation::compute::SequentialExecutor cpuExecutor;
  cpuWorld.step(cpuExecutor, cpuProbe);
  EXPECT_NEAR(cpuProbe.observed(), 0.0, 1e-9);

  dart::simulation::WorldOptions acceleratedOptions;
  acceleratedOptions.computeAcceleratorPolicy
      = dart::simulation::ComputeAcceleratorPolicy::PreferAccelerated;
  dart::simulation::World acceleratedWorld(acceleratedOptions);
  PsdProbeStage acceleratedProbe(/*expectAccelerated=*/true);
  dart::simulation::compute::SequentialExecutor acceleratedExecutor;
  acceleratedWorld.step(acceleratedExecutor, acceleratedProbe);
  EXPECT_EQ(acceleratedProbe.observed(), 42.0);
}

TEST_F(
    DeformablePsdAcceleratorTest,
    ConcurrentWorldStepsKeepScopedAcceleratorPolicy)
{
  registerFakeAccelerator(true);

  dart::simulation::World cpuWorld;
  dart::simulation::WorldOptions acceleratedOptions;
  acceleratedOptions.computeAcceleratorPolicy
      = dart::simulation::ComputeAcceleratorPolicy::PreferAccelerated;
  dart::simulation::World acceleratedWorld(acceleratedOptions);

  PsdProbeStage cpuProbe(/*expectAccelerated=*/false);
  PsdProbeStage acceleratedProbe(/*expectAccelerated=*/true);
  dart::simulation::compute::SequentialExecutor cpuExecutor;
  dart::simulation::compute::SequentialExecutor acceleratedExecutor;

  std::thread cpuThread([&] { cpuWorld.step(100, cpuExecutor, cpuProbe); });
  std::thread acceleratedThread([&] {
    acceleratedWorld.step(100, acceleratedExecutor, acceleratedProbe);
  });
  cpuThread.join();
  acceleratedThread.join();

  EXPECT_TRUE(cpuProbe.allObservedExpected());
  EXPECT_TRUE(acceleratedProbe.allObservedExpected());
  EXPECT_EQ(g_fakeProjectCalls.load(std::memory_order_relaxed), 100);
}

TEST(DeformablePsdBackend, CpuProjectionClampsNegativeEigenvalues)
{
  // [[-1, 0], [0, 2]] projects to [[0, 0], [0, 2]] (the negative eigenvalue is
  // clamped to zero), matching the per-element PSD projection of the solver.
  std::array<double, 4> block{-1.0, 0.0, 0.0, 2.0};
  projectSymmetricBlocksToPsd(block.data(), 2, 1);
  EXPECT_NEAR(block[0], 0.0, 1e-9);
  EXPECT_NEAR(block[1], 0.0, 1e-9);
  EXPECT_NEAR(block[2], 0.0, 1e-9);
  EXPECT_NEAR(block[3], 2.0, 1e-9);
}

} // namespace
