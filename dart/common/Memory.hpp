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

#include <dart/config.hpp>

#include <dart/common/Fwd.hpp>

#include <map>
#include <memory>
#include <vector>

namespace dart::common {

/// Returns a shared pointer to an object of type _Tp that is aligned to the
/// specified alignment.
///
/// @param alignment The alignment value that the base address needs to be
/// aligned to.
/// @param args The arguments to forward to the constructor of _Tp.
/// @return A shared pointer to an object of type _Tp that is aligned to the
/// specified alignment.
/// @note If the alignment is 0, this function will return a shared pointer to
/// an object of type _Tp that is not aligned.
template <typename _Tp, typename... _Args>
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... args);

/// Returns the padding needed to align the given base address to the specified
/// alignment.
///
/// @param base_address The base address that needs to be aligned.
/// @param alignment The alignment value that the base address needs to be
/// aligned to.
/// @return The padding needed to align the base address to the specified
/// alignment.
/// @note If the alignment is 0, this function will return 0.
[[nodiscard]] constexpr std::size_t GetPadding(
    const std::size_t base_address, const std::size_t alignment);

/// Returns the padding needed to align the given base address to the specified
/// alignment.
///
/// @param base_address The base address that needs to be aligned.
/// @param alignment The alignment value that the base address needs to be
/// aligned to.
/// @return The padding needed to align the base address to the specified
/// alignment.
/// @note If the alignment is 0, this function will return 0.
template <size_t Alignment>
[[nodiscard]] constexpr std::size_t GetPadding(const std::size_t base_address);

/// Returns the padding needed to align the given base address to the specified
/// alignment, including the size of the header.
///
/// @param base_address The base address that needs to be aligned.
/// @param alignment The alignment value that the base address needs to be
/// aligned to.
/// @return The padding needed to align the base address to the specified
/// alignment, including the size of the header.
/// @note If the alignment is 0, this function will return 0.
template <typename HeaderType>
[[nodiscard]] constexpr std::size_t GetPaddingIncludingHeader(
    const std::size_t base_address, const std::size_t alignment);

template <size_t Alignment, typename HeaderType>
[[nodiscard]] constexpr std::size_t GetPaddingIncludingHeader(
    const std::size_t base_address);

/// Allocates memory on a specified alignment boundary.
///
/// @param[in] alignment: The alignment value, which must be a power of 2.
/// @param[in] size: Size of the requested memory allocation.
/// @returns A pointer to the memory block that was allocated, or nullptr if the
/// allocation failed. The pointer is aligned to the specified alignment.
[[nodiscard]] DART_COMMON_API void* AlignedAlloc(
    std::size_t alignment, std::size_t size);

/// Releases a block of memory that was allocated with AlignedAlloc().
///
/// @param[in] ptr: A pointer to the memory block that was returned by
/// AlignedAlloc().
DART_COMMON_API void AlignedFree(void* ptr);

/// Checks if the given pointer is aligned to the specified alignment.
///
/// @param[in] ptr: The pointer to check.
/// @param[in] alignment: The alignment value that the pointer should be aligned
/// to.
/// @return True if the given pointer is aligned to the specified alignment,
/// false otherwise. Note: If the alignment is 0, this function will return
/// true.
[[nodiscard]] DART_COMMON_API bool IsAligned(
    void* ptr, std::size_t alignment) noexcept;

} // namespace dart::common

#define DART_RAW_PTR_CREATOR_NAME create
#define DART_SHARED_PTR_CREATOR_NAME createShared
#define DART_UNIQUE_PTR_CREATOR_NAME createUnique

// Define static creator function that returns a smart pointer to an object
#define _DART_DEFINE_OBJECT_CREATOR(class_name, func_name, ptr_type, creator)  \
  template <typename... Args>                                                  \
  static ptr_type<class_name> func_name(Args&&... args)                        \
  {                                                                            \
    return creator<class_name>(std::forward<Args>(args)...);                   \
  }

// Define static creator function that returns a smart pointer to an object with
// protected constructor
#define _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                        \
    class_name, func_name, ptr_type, creator)                                  \
private:                                                                       \
  struct private_structure                                                     \
  {                                                                            \
    explicit private_structure() {}                                            \
  };                                                                           \
                                                                               \
public:                                                                        \
  template <typename... Args>                                                  \
  class_name(const private_structure&, Args&&... args)                         \
    : class_name(std::forward<Args>(args)...)                                  \
  {                                                                            \
  }                                                                            \
  template <typename... Args>                                                  \
  static ptr_type<class_name> func_name(Args&&... args)                        \
  {                                                                            \
    return creator<class_name>(                                                \
        private_structure{}, std::forward<Args>(args)...);                     \
  }

// Define static creator function that returns a raw pointer to the object.
// This static functions will be defined: create()
#define DART_DEFINE_RAW_OBJECT_CREATOR(class_name)                             \
  template <typename... Args>                                                  \
  static class_name* DART_RAW_PTR_CREATOR_NAME(Args&&... args)                 \
  {                                                                            \
    return new class_name(std::forward<Args>(args)...);                        \
  }

// Define static creator function that returns std::shared_ptr to the object.
// This static functions will be defined: createShared()
#define DART_DEFINE_SHARED_OBJECT_CREATOR(class_name)                          \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR(                                                 \
      class_name,                                                              \
      DART_SHARED_PTR_CREATOR_NAME,                                            \
      std::shared_ptr,                                                         \
      std::make_shared)

// Define static creator function that returns std::shared_ptr to the object
// where the constructor is protected.
// This static functions will be defined: createShared()
#define DART_DEFINE_SHARED_OBJECT_CREATOR_FOR_PROTECTED_CTOR(class_name)       \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                              \
      class_name,                                                              \
      DART_SHARED_PTR_CREATOR_NAME,                                            \
      std::shared_ptr,                                                         \
      std::make_shared)

// Define static creator function that returns std::shared_ptr to the object
// requires aligned memory allocation.
// This static functions will be defined: createShared()
#define DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(class_name)                  \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR(                                                 \
      class_name,                                                              \
      DART_SHARED_PTR_CREATOR_NAME,                                            \
      std::shared_ptr,                                                         \
      ::dart::common::make_aligned_shared)

// Define static creator function that returns std::shared_ptr to the object
// requires aligned memory allocation where the constructor is protected.
// This static functions will be defined: createShared()
#define DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR_FOR_PROTECTED_CTOR(          \
    class_name)                                                                \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                              \
      class_name,                                                              \
      DART_SHARED_PTR_CREATOR_NAME,                                            \
      std::shared_ptr,                                                         \
      ::dart::common::make_aligned_shared)

// Define static creator function that returns std::unique_ptr to the object
#define DART_DEFINE_UNIQUE_OBJECT_CREATOR(class_name)                          \
  duke /*! Create unique instance of this class */                             \
  _DART_DEFINE_OBJECT_CREATOR(                                                 \
      class_name,                                                              \
      DART_UNIQUE_PTR_CREATOR_NAME,                                            \
      std::unique_ptr,                                                         \
      ::std::make_unique)

// Define static creator function that returns std::unique_ptr to the object
// where the constructor is protected
#define DART_DEFINE_UNIQUE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(class_name)       \
  /*! Create unique instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                              \
      class_name,                                                              \
      DART_UNIQUE_PTR_CREATOR_NAME,                                            \
      std::unique_ptr,                                                         \
      ::std::make_unique)

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to the object
#define DART_DEFINE_OBJECT_CREATORS(class_name)                                \
  DART_DEFINE_SHARED_OBJECT_CREATOR(class_name)                                \
  DART_DEFINE_UNIQUE_OBJECT_CREATOR(class_name)

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to the object where the constructor is
// protected
#define DART_DEFINE_OBJECT_CREATORS_FOR_PROTECTED_CTOR(X)                      \
  DART_DEFINE_SHARED_OBJECT_CREATOR_FOR_PROTECTED_CTOR(X)                      \
  DART_DEFINE_UNIQUE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(X)

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to the object
#if DART_ENABLE_SIMD
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS(class_name)                      \
    DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(class_name)                      \
    DART_DEFINE_UNIQUE_OBJECT_CREATOR(class_name)
#else
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS(class_name)                      \
    DART_DEFINE_OBJECT_CREATORS(class_name)
#endif

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to the object where the constructor is
// protected
#if DART_ENABLE_SIMD
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS_FOR_PROTECTED_CTOR(class_name)   \
    DART_DEFINE_CREATE_ALIGNED_PTR_SHARED_FOR_PROTECTED_CTOR(class_name)       \
    DART_DEFINE_UNIQUE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(class_name)
#else
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS_FOR_PROTECTED_CTOR(class_name)   \
    DART_DEFINE_OBJECT_CREATORS_FOR_PROTECTED_CTOR(class_name)
#endif

#include <dart/common/detail/Memory-impl.hpp>
