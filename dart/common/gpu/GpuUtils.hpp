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

  #include <memory>
  #include <string>
  #include <unordered_map>
  #include <vector>

#endif

namespace dart::common {

/// Returns whether GPU is available on the system
///
/// @param[in] use_cache Whether to use the cached value
/// @return True if GPU is available on the system
[[nodiscard]] DART_COMMON_API bool IsGpuAvailable(bool use_cache = true);

#if DART_ENABLED_GPU

/// Returns all available OpenCL platforms
///
/// @return A vector of all available OpenCL platforms
[[nodiscard]] DART_COMMON_API std::vector<cl::Platform> GetPlatforms();

/// Returns all available OpenCL devices
///
/// @param[in] device_type The type of device to return
/// @return A vector of all available OpenCL devices
[[nodiscard]] DART_COMMON_API std::vector<cl::Device> GetDevices(
    cl_device_type device_type = CL_DEVICE_TYPE_ALL);

/// Returns all available OpenCL devices
///
/// @param[in] device_type The type of device to return
/// @return A vector of all available OpenCL devices
[[nodiscard]] DART_COMMON_API std::vector<cl::Device> GetDevices(
    const cl::Platform& platform,
    cl_device_type device_type = CL_DEVICE_TYPE_ALL);

/// Returns all available OpenCL devices
///
/// @param[in] platforms The platforms to search for devices
/// @param[in] device_type The type of device to return
/// @return A vector of all available OpenCL devices
[[nodiscard]] DART_COMMON_API std::vector<cl::Device> GetDevices(
    const std::vector<cl::Platform>& platforms,
    cl_device_type device_type = CL_DEVICE_TYPE_ALL);

/// Returns a string representation of the given OpenCL device type
///
/// @param[in] device_type The device type to convert to a string
/// @return A string representation of the given OpenCL device type
[[nodiscard]] DART_COMMON_API std::string DeviceTypeToString(
    cl_device_type device_type);

#endif

} // namespace dart::common
