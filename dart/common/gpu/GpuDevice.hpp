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

#pragma once

#include <dart/common/Fwd.hpp>

#if DART_ENABLED_GPU

  #include <dart/common/gpu/Fwd.hpp>
  #include <dart/common/gpu/GpuUtils.hpp>

namespace dart::common {

/// A class that represents an OpenCL device.
class DART_COMMON_API GpuDevice
{
public:
  /// Default constructor that creates an invalid GpuDevice.
  GpuDevice();

  /// Check if the GpuDevice is valid and can be used.
  ///
  /// @return True if the device is valid, false otherwise.
  [[nodiscard]] bool isValid() const;

  /// Creates a GPU kernel
  [[nodiscard]] GpuKernel createKernel();

  /// Creates a GPU kernel from kernel source code string
  [[nodiscard]] GpuKernel createKernel(const std::string& kernel_string);

  /// Get the OpenCL device object.
  ///
  /// @return The OpenCL device object.
  [[nodiscard]] cl::Device getOpenCLDevice() const;

  /// Get the OpenCL context associated with this device.
  ///
  /// @return The OpenCL context object.
  [[nodiscard]] cl::Context getOpenCLContext() const;

private:
  /// The OpenCL device object.
  cl::Device m_device;

  /// The OpenCL context object.
  cl::Context m_context;
};

} // namespace dart::common

#endif //
