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

  #include <dart/common/TemplateUtils.hpp>
  #include <dart/common/gpu/Fwd.hpp>

namespace dart::common {

/// An object-oriented interface to execute GPU kernels.
class DART_COMMON_API GpuKernel
{
private:
  friend class GpuDevice;
  /// Constructor that initializes a GpuKernel object with a GpuDevice instance.
  ///
  /// @param[in] runtime The GpuDevice instance to use for kernel execution.
  explicit GpuKernel(GpuDevice* runtime);

  /// Constructor that initializes a GpuKernel object with a kernel string.
  ///
  /// @param[in] runtime The GpuDevice instance to use for kernel execution.
  /// @param[in] kernel_string The source string for the OpenCL kernel.
  explicit GpuKernel(GpuDevice* runtime, const std::string& kernel_string);

public:
  /// Checks if the kernel with the given name has been built and is available
  /// for execution.
  ///
  /// @param[in] kernel_name The name of the kernel to check.
  ///
  /// @returns `true` if the kernel is available, `false` otherwise.
  [[nodiscard]] bool has(const std::string& kernel_name) const;

  /// Executes the kernel with the given name.
  ///
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel_name The name of the kernel to execute.
  /// @param[in] args The arguments to pass to the kernel.
  ///
  /// @returns `true` if the kernel is executed successfully, `false` otherwise.
  template <typename... Args>
  bool run(const std::string& kernel_name, Args&&... args);

private:
  /// Builds the kernel from the kernel string.
  ///
  /// @param[in] kernel_source The source string for the OpenCL kernel.
  void buildKernel(const std::string& kernel_source);

  /// Runs the kernel with the given arguments.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to execute.
  /// @param[in] args The arguments to pass to the kernel.
  template <size_t ArgIndex, typename... Args>
  void runKernel(cl::Kernel& kernel, Args&&... args);

  /// Sets the kernel argument with an arithmetic type.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam Arg The type of the argument being set.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] arg The argument value.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename Arg,
      typename... Args,
      typename = std::enable_if_t<std::is_arithmetic_v<Arg>>>
  void setKernelArgs(cl::Kernel& kernel, Arg&& arg, Args&&... args);

  /// Sets the kernel argument with a C-style array.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam ArrayElementType The type of the array elements.
  /// @tparam ArrayN The number of elements in the array.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] array The array to set as the argument.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename ArrayElementType,
      size_t ArrayN,
      typename... Args>
  void setKernelArgs(
      cl::Kernel& kernel, ArrayElementType (&array)[ArrayN], Args&&... args);

  // TODO: Add support for a const C-style array.

  /// Sets the kernel argument with a std::array.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam ArrayElementType The type of the array elements.
  /// @tparam ArrayN The number of elements in the array.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] std_array The std::array to set as the argument.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename ArrayElementType,
      size_t ArrayN,
      typename... Args>
  void setKernelArgs(
      cl::Kernel& kernel,
      std::array<ArrayElementType, ArrayN>& std_array,
      Args&&... args);

  /// Sets the kernel argument with a const std::array.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam ArrayElementType The type of the array elements.
  /// @tparam ArrayN The number of elements in the array.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] std_array The const std::array to set as the argument.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename ArrayElementType,
      size_t ArrayN,
      typename... Args>
  void setKernelArgs(
      cl::Kernel& kernel,
      const std::array<ArrayElementType, ArrayN>& std_array,
      Args&&... args);

  /// Sets the kernel argument with a contiguous container.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam Container The type of the container.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] container The contiguous container to set as the argument.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename Container,
      typename... Args,
      typename = std::enable_if_t<is_contiguous_container_v<Container>>>
  void setKernelArgs(cl::Kernel& kernel, Container& container, Args&&... args);

  /// Sets the kernel argument with a const and contiguous container.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam Container The type of the container.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] container The const and contiguous container to set as the
  /// argument.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename Container,
      typename... Args,
      typename = std::enable_if_t<is_contiguous_container_v<Container>>>
  void setKernelArgs(
      cl::Kernel& kernel, const Container& container, Args&&... args);

  /// Sets the kernel argument with iterators of contiguous container.
  ///
  /// @tparam ArgIndex The index of the current argument being set.
  /// @tparam ArrayElementType The type of the array elements.
  /// @tparam ArrayN The number of elements in the array.
  /// @tparam Args Variadic template arguments for the kernel arguments.
  ///
  /// @param[in] kernel The kernel to set the argument for.
  /// @param[in] begin The beginning iterator of the container.
  /// @param[in] end The ending iterator of the container.
  /// @param[in] args The remaining arguments to set.
  template <
      size_t ArgIndex,
      typename Iterator,
      typename... Args,
      typename = std::enable_if_t<is_contiguous_iterator_v<Iterator>>>
  void setKernelArgs(
      cl::Kernel& kernel, Iterator begin, Iterator end, Args&&... args);

  /// Helper function to set the kernel arguments with iterators of contiguous
  /// container.
  template <size_t ArgIndex, typename Iterator, typename... Args>
  void setKernelArgsHelper(
      cl::Kernel& kernel,
      Iterator begin,
      Iterator end,
      std::false_type,
      Args&&... args);

  /// Helper function to set the kernel arguments with iterators of contiguous
  /// container.
  template <size_t ArgIndex, typename Iterator, typename... Args>
  void setKernelArgsHelper(
      cl::Kernel& kernel,
      Iterator begin,
      Iterator end,
      std::true_type,
      Args&&... args);

  template <size_t ArgIndex>
  void setKernelArgs(cl::Kernel& kernel);

  GpuDevice* m_runtime;
  cl::CommandQueue m_queue;
  cl::Program m_program;
  cl::Event m_event;
  std::unordered_map<std::string, cl::Kernel> m_kernels;
  std::vector<std::pair<cl::Buffer, size_t>> m_buffers;
  cl::NDRange m_global = cl::NDRange(0, 0, 0);
  cl::NDRange m_local = cl::NullRange;
  cl::NDRange m_offset = cl::NullRange;
};

} // namespace dart::common

//==============================================================================
// Implementation
//==============================================================================

  #include <dart/common/gpu/GpuDevice.hpp>

namespace dart::common {

//==============================================================================
template <typename... Args>
bool GpuKernel::run(const std::string& kernel_name, Args&&... args)
{
  auto kernel_iter = m_kernels.find(kernel_name);
  if (kernel_iter == m_kernels.end()) {
    return false;
  }

  auto& kernel = kernel_iter->second;
  runKernel<0>(kernel, std::forward<Args>(args)...);
  return true;
}

//==============================================================================
template <size_t ArgIndex, typename... Args>
void GpuKernel::runKernel(cl::Kernel& kernel, Args&&... args)
{
  setKernelArgs<ArgIndex>(kernel, std::forward<Args>(args)...);

  std::vector<cl::Event> events;
  events.reserve(m_buffers.size() + 1);

  // Enqueue the kernel for execution
  m_queue.enqueueNDRangeKernel(
      kernel, m_offset, m_global, m_local, nullptr, &m_event);
  events.push_back(m_event);

  // Map the buffers for reading
  for (const auto& i : m_buffers) {
    cl::Event event;
    void* mapped_ptr = m_queue.enqueueMapBuffer(
        i.first, CL_TRUE, CL_MAP_READ, 0, i.second, nullptr, &event);
    events.push_back(event);
    m_queue.enqueueUnmapMemObject(i.first, mapped_ptr);
  }

  // Wait for all operations to complete
  m_queue.finish();
}

//==============================================================================
template <size_t ArgIndex, typename Arg, typename... Args, typename Enable>
void GpuKernel::setKernelArgs(cl::Kernel& kernel, Arg&& arg, Args&&... args)
{
  kernel.setArg(ArgIndex, std::forward<Arg>(arg));
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

template <
    size_t ArgIndex,
    typename ArrayElementType,
    size_t ArrayN,
    typename... Args>
void GpuKernel::setKernelArgs(
    cl::Kernel& kernel, ArrayElementType (&array)[ArrayN], Args&&... args)
{
  constexpr size_t array_size = ArrayN * sizeof(ArrayElementType);
  if (ArrayN > m_global[0]) {
    m_global = cl::NDRange(ArrayN);
  }
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(), CL_MEM_USE_HOST_PTR, array_size, array);
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <
    size_t ArgIndex,
    typename ArrayElementType,
    size_t ArrayN,
    typename... Args>
void GpuKernel::setKernelArgs(
    cl::Kernel& kernel,
    std::array<ArrayElementType, ArrayN>& std_array,
    Args&&... args)
{
  constexpr size_t array_size = ArrayN * sizeof(ArrayElementType);
  if (ArrayN > m_global[0]) {
    m_global = cl::NDRange(ArrayN);
  }
  // TODO: Avoid creating buffer for every kernel call
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(),
      CL_MEM_USE_HOST_PTR,
      array_size,
      std_array.data());
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <
    size_t ArgIndex,
    typename ArrayElementType,
    size_t ArrayN,
    typename... Args>
void GpuKernel::setKernelArgs(
    cl::Kernel& kernel,
    const std::array<ArrayElementType, ArrayN>& std_array,
    Args&&... args)
{
  constexpr size_t array_size = ArrayN * sizeof(ArrayElementType);
  if (ArrayN > m_global[0]) {
    m_global = cl::NDRange(ArrayN);
  }
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(),
      CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
      array_size,
      const_cast<ArrayElementType*>(std_array.data()));
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <
    size_t ArgIndex,
    typename Container,
    typename... Args,
    typename Enable>
void GpuKernel::setKernelArgs(
    cl::Kernel& kernel, Container& container, Args&&... args)
{
  using value_type = typename Container::value_type;
  const size_t array_size = container.size() * sizeof(value_type);
  if (container.size() > m_global[0]) {
    m_global = cl::NDRange(container.size());
  }
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(),
      CL_MEM_USE_HOST_PTR,
      array_size,
      container.data());
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <
    size_t ArgIndex,
    typename Container,
    typename... Args,
    typename Enable>
void GpuKernel::setKernelArgs(
    cl::Kernel& kernel, const Container& container, Args&&... args)
{
  using value_type = typename Container::value_type;
  const size_t array_size = container.size() * sizeof(value_type);
  if (container.size() > m_global[0]) {
    m_global = cl::NDRange(container.size());
  }
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(),
      CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
      array_size,
      const_cast<value_type*>(container.data()));
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <size_t ArgIndex, typename Iterator, typename... Args, typename Enable>
void GpuKernel::setKernelArgs(
    cl::Kernel& kernel, Iterator begin, Iterator end, Args&&... args)
{
  using value_type =
      typename std::remove_reference<decltype(*std::declval<Iterator>())>::type;
  setKernelArgsHelper<ArgIndex, Iterator, Args...>(
      kernel,
      begin,
      end,
      std::is_const<value_type>{},
      std::forward<Args>(args)...);
}

//==============================================================================
template <size_t ArgIndex, typename Iterator, typename... Args>
void GpuKernel::setKernelArgsHelper(
    cl::Kernel& kernel,
    Iterator begin,
    Iterator end,
    std::false_type,
    Args&&... args)
{
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  const size_t array_size = std::distance(begin, end) * sizeof(value_type);
  if (array_size > m_global[0]) {
    m_global = cl::NDRange(array_size);
  }
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(),
      CL_MEM_USE_HOST_PTR,
      array_size,
      &(*begin));
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <size_t ArgIndex, typename Iterator, typename... Args>
void GpuKernel::setKernelArgsHelper(
    cl::Kernel& kernel,
    Iterator begin,
    Iterator end,
    std::true_type,
    Args&&... args)
{
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  const size_t array_size = std::distance(begin, end) * sizeof(value_type);
  if (array_size > m_global[0]) {
    m_global = cl::NDRange(array_size);
  }
  const auto buffer = cl::Buffer(
      m_runtime->getOpenCLContext(),
      CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY,
      array_size,
      const_cast<void*>(static_cast<const void*>(&(*begin))));
  m_buffers.emplace_back(std::make_pair(buffer, array_size));
  kernel.setArg(ArgIndex, buffer);
  setKernelArgs<ArgIndex + 1>(kernel, std::forward<Args>(args)...);
}

//==============================================================================
template <size_t ArgIndex>
void GpuKernel::setKernelArgs(cl::Kernel& kernel)
{
  // Base case for recursion
  DART_UNUSED(kernel);
}

} // namespace dart::common

#endif
