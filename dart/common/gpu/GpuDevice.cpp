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

#include "dart/common/gpu/GpuDevice.hpp"

#include "dart/common/gpu/GpuKernel.hpp"

#if DART_ENABLED_GPU

namespace dart::common {

//==============================================================================
GpuDevice::GpuDevice()
{
  const auto& devices = GetDevices(CL_DEVICE_TYPE_GPU);
  if (devices.empty()) {
    return;
  }
  m_device = devices[0];

  m_context = cl::Context(m_device);
}

//==============================================================================
bool dart::common::GpuDevice::isValid() const
{
  return m_device != cl::Device() && m_context != cl::Context();
}

//==============================================================================
GpuKernel GpuDevice::createKernel()
{
  return GpuKernel(this);
}

//==============================================================================
GpuKernel GpuDevice::createKernel(const std::string& kernel_string)
{
  return GpuKernel(this, kernel_string);
}

//==============================================================================
cl::Device GpuDevice::getOpenCLDevice() const
{
  return m_device;
}

//==============================================================================
cl::Context GpuDevice::getOpenCLContext() const
{
  return m_context;
}

} // namespace dart::common

#endif // if DART_ENABLED_GPU
