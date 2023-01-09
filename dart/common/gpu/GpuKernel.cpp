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

#include "dart/common/gpu/GpuKernel.hpp"

#if DART_ENABLED_GPU

namespace dart::common {

//==============================================================================
GpuKernel::GpuKernel(GpuDevice* runtime)
  : m_runtime(runtime),
// Create command queue per kernel to be thread-safe
  #ifdef CL_VERSION_3_0
    m_queue(
        m_runtime->getOpenCLContext(),
        m_runtime->getOpenCLDevice(),
        CL_QUEUE_PROFILING_ENABLE)
  #else
    m_queue(clCreateCommandQueue(
        m_runtime->getOpenCLContext()(),
        m_runtime->getOpenCLDevice()(),
        CL_QUEUE_PROFILING_ENABLE,
        nullptr))
  #endif
{
  // Empty
}

//==============================================================================
GpuKernel::GpuKernel(GpuDevice* runtime, const std::string& kernel_string)
  : GpuKernel(runtime)
{
  buildKernel(kernel_string);
}

//==============================================================================
bool GpuKernel::has(const std::string& kernel_name) const
{
  return m_kernels.find(kernel_name) != m_kernels.end();
}

//==============================================================================
void GpuKernel::buildKernel(const std::string& kernel_source)
{
  // Create a program
  m_program = cl::Program(m_runtime->getOpenCLContext(), kernel_source);

  // Build the program for the devices
  m_program.build(m_runtime->getOpenCLContext().getInfo<CL_CONTEXT_DEVICES>());

  std::vector<cl::Kernel> kernels;
  m_program.createKernels(&kernels);
  for (const auto& i : kernels) {
    m_kernels[i.getInfo<CL_KERNEL_FUNCTION_NAME>()] = i;
  }
}

} // namespace dart::common

#endif
