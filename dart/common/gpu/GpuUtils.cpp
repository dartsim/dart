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

#include "dart/common/gpu/GpuUtils.hpp"

#if DART_ENABLED_GPU

  #include <fstream>

#endif

namespace dart::common {

//==============================================================================
bool IsGpuAvailable(bool use_cache)
{
#if DART_ENABLED_GPU
  static bool gpu_available
      = !GetDevices(GetPlatforms(), CL_DEVICE_TYPE_GPU).empty();
  return use_cache ? gpu_available
                   : !GetDevices(GetPlatforms(), CL_DEVICE_TYPE_GPU).empty();
#else
  DART_UNUSED(use_cache);
  return false;
#endif
}

#if DART_ENABLED_GPU

//==============================================================================
std::vector<cl::Platform> GetPlatforms()
{
  std::vector<cl::Platform> platforms;
  cl::Platform::get(&platforms);
  return platforms;
}

//==============================================================================
std::vector<cl::Device> GetDevices(cl_device_type device_type)
{
  return GetDevices(GetPlatforms(), device_type);
}

//==============================================================================
std::vector<cl::Device> GetDevices(
    const cl::Platform& platform, cl_device_type device_type)
{
  std::vector<cl::Device> devices;
  platform.getDevices(device_type, &devices);
  return devices;
}

//==============================================================================
std::vector<cl::Device> GetDevices(
    const std::vector<cl::Platform>& platforms, cl_device_type device_type)
{
  std::vector<cl::Device> devices;

  for (const auto& platform : platforms) {
    const auto platform_devices = GetDevices(platform, device_type);
    devices.insert(
        devices.end(), platform_devices.begin(), platform_devices.end());
  }

  return devices;
}

//==============================================================================
std::string DeviceTypeToString(cl_device_type device_type)
{
  switch (device_type) {
    case CL_DEVICE_TYPE_CPU:
      return "CPU";
    case CL_DEVICE_TYPE_GPU:
      return "GPU";
    case CL_DEVICE_TYPE_ACCELERATOR:
      return "Accelerator";
    case CL_DEVICE_TYPE_CUSTOM:
      return "Custom";
    case CL_DEVICE_TYPE_DEFAULT:
      return "Default";
    default:
      return "Unknown device type";
  }
}

//==============================================================================
cl::Device GetFirstDevice(cl_device_type device_type)
{
  std::vector<cl::Platform> platforms;
  const cl_int error = cl::Platform::get(&platforms);
  if (error != CL_SUCCESS) {
    throw std::runtime_error("Failed to get OpenCL platforms");
  }
  if (platforms.empty()) {
    throw std::runtime_error("No OpenCL platforms found");
  }

  std::vector<cl::Device> devices;
  platforms[0].getDevices(device_type, &devices);
  if (devices.empty()) {
    throw std::runtime_error("No OpenCL devices found");
  }

  return devices[0];
}

//==============================================================================
cl::Context CreateContextForDevice(const cl::Device& device)
{
  cl_int error;
  cl::Context context
      = cl::Context({device}, nullptr, nullptr, nullptr, &error);
  if (error != CL_SUCCESS) {
    throw std::runtime_error("Failed to create OpenCL context");
  }
  return context;
}

//==============================================================================
cl::Program CreateProgramFromFile(
    const cl::Context& context,
    const std::string& fileName,
    const std::string& buildOptions = "")
{
  std::ifstream file(fileName);
  if (!file.good()) {
    throw std::runtime_error("Failed to open file " + fileName);
  }

  std::string source(
      (std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  cl::Program::Sources sources({source});

  cl_int error;
  cl::Program program(context, sources, &error);
  if (error != CL_SUCCESS) {
    throw std::runtime_error("Failed to create OpenCL program");
  }
  program.build(
      {context.getInfo<CL_CONTEXT_DEVICES>()[0]}, buildOptions.c_str());
  return program;
}

//==============================================================================
template <typename T>
void CopyDataToDevice(
    cl::CommandQueue& queue, cl::Buffer& buffer, const T* data, size_t size)
{
  cl_int error
      = queue.enqueueWriteBuffer(buffer, CL_TRUE, 0, size * sizeof(T), data);
  if (error != CL_SUCCESS) {
    throw std::runtime_error("Failed to copy data to device buffer");
  }
}

//==============================================================================
void BuildProgram(cl::Program& program, const cl::Device& device)
{
  cl_int error = program.build({device});
  if (error != CL_SUCCESS) {
    throw std::runtime_error(
        "Failed to build OpenCL program: "
        + program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(device));
  }
}

//==============================================================================
void EnqueueKernel(
    cl::CommandQueue& queue,
    cl::Kernel& kernel,
    const cl::NDRange& global,
    const cl::NDRange& local)
{
  cl_int error
      = queue.enqueueNDRangeKernel(kernel, cl::NullRange, global, local);
  if (error != CL_SUCCESS) {
    throw std::runtime_error("Failed to enqueue OpenCL kernel");
  }
}

#endif

} // namespace dart::common
