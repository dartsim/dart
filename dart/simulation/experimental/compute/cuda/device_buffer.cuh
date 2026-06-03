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

#pragma once

#include <cuda_runtime_api.h>
#include <dart/simulation/experimental/compute/cuda/cuda_runtime.cuh>

#include <string_view>
#include <vector>

#include <cstddef>

// One owning RAII device buffer for the experimental CUDA modules. It subsumes
// the three near-identical buffer classes the solvers used to each define:
//   * rigid_body_state_batch_cuda's DeviceDoubleBuffer (alloc; free copy
//   helpers)
//   * vbd_block_descent_cuda's DeviceArray<T>           (alloc + copy-in ctor)
//   * deformable_psd_projection_cuda's ResidentDeviceBuffer (grow-and-reuse)
// All three wrap the same cudaMalloc / cudaMemcpy / cudaFree lifecycle. See
// docs/design/shared_cuda_device_substrate.md (PLAN-031).
//
// Build-tree-only (.cuh): never installed and never seen by the boundary
// scanner. The default scalar is double so the migrated rigid and PSD paths
// stay bit-for-bit unchanged; VBD instantiates it for float on the opt-in
// mixed-precision rollout exactly as its DeviceArray<float> did.

namespace dart::simulation::experimental::compute::cuda {

template <typename T = double>
class DeviceBuffer
{
public:
  DeviceBuffer() = default;

  /// Allocate storage for @p count elements without copying any host data
  /// (the rigid-body path's "allocate then copy in" usage).
  explicit DeviceBuffer(std::size_t count)
  {
    ensure(count);
  }

  /// Allocate storage for @p host.size() elements and copy them to the device
  /// (the VBD path's copy-in constructor).
  explicit DeviceBuffer(const std::vector<T>& host)
  {
    ensure(host.size());
    copyToDevice(host, "copy to device");
  }

  ~DeviceBuffer()
  {
    release();
  }

  DeviceBuffer(const DeviceBuffer&) = delete;
  DeviceBuffer& operator=(const DeviceBuffer&) = delete;

  [[nodiscard]] T* data() noexcept
  {
    return m_data;
  }

  [[nodiscard]] const T* data() const noexcept
  {
    return m_data;
  }

  [[nodiscard]] std::size_t size() const noexcept
  {
    return m_count;
  }

  [[nodiscard]] std::size_t byteSize() const noexcept
  {
    return m_count * sizeof(T);
  }

  /// Return device storage for at least @p count elements, reusing the existing
  /// allocation whenever it is already large enough and only reallocating (and
  /// freeing the old storage) when a larger request arrives. This is the
  /// resident grow-and-reuse path the PSD projection drives across calls so the
  /// per-call cudaMalloc/cudaFree round trip is removed from the solver hot
  /// path. @ref allocationCount only increments on an actual (re)allocation.
  T* ensure(std::size_t count)
  {
    if (count > m_capacity) {
      release();
      if (count > 0) {
        throwIfCudaError(
            cudaMalloc(reinterpret_cast<void**>(&m_data), count * sizeof(T)),
            "allocation");
      }
      m_capacity = count;
      ++m_allocationCount;
    }
    m_count = count;
    return m_data;
  }

  /// Free the device allocation. The cumulative @ref allocationCount is kept so
  /// callers can assert that reuse, not per-call churn, drove the allocations.
  void release() noexcept
  {
    if (m_data != nullptr) {
      (void)cudaFree(m_data);
      m_data = nullptr;
    }
    m_capacity = 0;
    m_count = 0;
  }

  /// Cumulative number of device (re)allocations this buffer has performed.
  [[nodiscard]] std::size_t allocationCount() const noexcept
  {
    return m_allocationCount;
  }

  /// Copy @p host to the device (host-to-device), a no-op for an empty source.
  void copyToDevice(const std::vector<T>& host, std::string_view operation)
  {
    if (host.empty()) {
      return;
    }
    throwIfCudaError(
        cudaMemcpy(
            m_data,
            host.data(),
            host.size() * sizeof(T),
            cudaMemcpyHostToDevice),
        operation);
  }

  /// Copy the device buffer back into @p host (device-to-host), a no-op for an
  /// empty target.
  void copyFromDevice(std::vector<T>& host, std::string_view operation) const
  {
    if (host.empty()) {
      return;
    }
    throwIfCudaError(
        cudaMemcpy(
            host.data(),
            m_data,
            host.size() * sizeof(T),
            cudaMemcpyDeviceToHost),
        operation);
  }

private:
  T* m_data = nullptr;
  std::size_t m_capacity = 0;
  std::size_t m_count = 0;
  std::size_t m_allocationCount = 0;
};

} // namespace dart::simulation::experimental::compute::cuda
