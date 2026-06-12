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
#include <dart/simulation/detail/newton_barrier/psd_backend.hpp>

#include <gtest/gtest.h>

#include <array>

#include <cstddef>

namespace {

using namespace dart::simulation::compute;
namespace nb = dart::simulation::detail::newton_barrier;

// A fake accelerator backed by file-static state, registered through the
// backend-neutral hook with plain function pointers. No device technology is
// involved, so the core acceleration control is exercised on any host.
bool g_available = false;
bool g_installed = false;

bool fakeAvailable()
{
  return g_available;
}

bool fakeEnable()
{
  g_installed = true;
  return true;
}

void fakeDisable()
{
  g_installed = false;
}

void noopProjector(double*, std::size_t, std::size_t)
{
  // A distinct non-CPU projector so install/restore is observable.
}

class DeformablePsdAcceleratorTest : public ::testing::Test
{
protected:
  void TearDown() override
  {
    // Clear any registered accelerator and return to the CPU backend so tests
    // do not leak the process-wide acceleration state into one another.
    setDeformablePsdAccelerator(DeformablePsdAccelerator{});
    setDeformablePsdAccelerated(false);
    g_available = false;
    g_installed = false;
  }
};

TEST_F(DeformablePsdAcceleratorTest, NoAcceleratorRegisteredIsSafeNoOp)
{
  setDeformablePsdAccelerator(DeformablePsdAccelerator{});
  EXPECT_FALSE(isDeformablePsdAccelerationAvailable());
  EXPECT_FALSE(setDeformablePsdAccelerated(true));
  EXPECT_FALSE(isDeformablePsdAccelerated());
  EXPECT_EQ(deformablePsdBlockProjector(), &projectSymmetricBlocksToPsdCpu);
}

TEST_F(DeformablePsdAcceleratorTest, RegisteredButUnavailableStaysOnCpu)
{
  g_available = false;
  setDeformablePsdAccelerator({&fakeAvailable, &fakeEnable, &fakeDisable});
  EXPECT_FALSE(isDeformablePsdAccelerationAvailable());
  EXPECT_FALSE(setDeformablePsdAccelerated(true));
  EXPECT_FALSE(isDeformablePsdAccelerated());
  EXPECT_FALSE(g_installed); // enable is never called when unavailable
}

TEST_F(DeformablePsdAcceleratorTest, EnableAndDisableToggleTheAccelerator)
{
  g_available = true;
  setDeformablePsdAccelerator({&fakeAvailable, &fakeEnable, &fakeDisable});
  EXPECT_TRUE(isDeformablePsdAccelerationAvailable());

  EXPECT_TRUE(setDeformablePsdAccelerated(true));
  EXPECT_TRUE(isDeformablePsdAccelerated());
  EXPECT_TRUE(g_installed);

  EXPECT_FALSE(setDeformablePsdAccelerated(false));
  EXPECT_FALSE(isDeformablePsdAccelerated());
  EXPECT_FALSE(g_installed);
}

TEST_F(DeformablePsdAcceleratorTest, DisableWithoutHookRestoresCpuProjector)
{
  // An accelerator with no disable hook still restores the CPU projector when
  // acceleration is turned off.
  setDeformablePsdBlockProjector(&noopProjector);
  g_available = true;
  setDeformablePsdAccelerator({&fakeAvailable, &fakeEnable, nullptr});
  EXPECT_FALSE(setDeformablePsdAccelerated(false));
  EXPECT_EQ(deformablePsdBlockProjector(), &projectSymmetricBlocksToPsdCpu);
}

TEST_F(DeformablePsdAcceleratorTest, NewtonBarrierWrapperRoutesCoreBackend)
{
  EXPECT_EQ(nb::psdBlockProjector(), &projectSymmetricBlocksToPsdCpu);
  EXPECT_EQ(
      nb::setPsdBlockProjector(&noopProjector),
      &projectSymmetricBlocksToPsdCpu);
  EXPECT_EQ(deformablePsdBlockProjector(), &noopProjector);
  EXPECT_EQ(nb::setPsdBlockProjector(nullptr), &noopProjector);
  EXPECT_EQ(nb::psdBlockProjector(), &projectSymmetricBlocksToPsdCpu);

  std::array<double, 4> block{-1.0, 0.0, 0.0, 2.0};
  nb::projectSymmetricBlocksToPsd(block.data(), 2, 1);
  EXPECT_NEAR(block[0], 0.0, 1e-9);
  EXPECT_NEAR(block[1], 0.0, 1e-9);
  EXPECT_NEAR(block[2], 0.0, 1e-9);
  EXPECT_NEAR(block[3], 2.0, 1e-9);
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
