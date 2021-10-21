/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <vector>

#include "dart/common/export.hpp"

namespace dart {
namespace common {

template <typename _Tp, typename... _Args>
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args);

/// Returns the distance from the base address to the next aligned address of
/// the base address, so that base_address + padding = aligned_address.
constexpr std::size_t get_padding(
    const std::size_t base_address, const std::size_t alignment);

/// A portable function to allocate memory on a specified alignment boundary.
///
/// \param[in] alignment: The alignment value, which must be an integer power
/// of 2.
/// \param[in] size: Size of the requested memory allocation.
/// \returns A pointer to the memory block that was allocated or nullptr if the
/// operation failed. The pointer is a multiple of alignment.
DART_COMMON_API void* aligned_alloc(std::size_t alignment, std::size_t size);

/// A portable function to releases a block of memory that was allocated with
/// aligned_alloc()
///
/// \param[in] ptr: A pointer to the memory block that was returned to the
/// aligned_alloc()
DART_COMMON_API void aligned_free(void* ptr);

DART_COMMON_API bool is_aligned(void* ptr, std::size_t alignment) noexcept;

namespace literals {

constexpr std::size_t operator"" _KiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024);
}

constexpr std::size_t operator"" _KB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000);
}

constexpr std::size_t operator"" _MiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024 * 1024);
}

constexpr std::size_t operator"" _MB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000 * 1000);
}

constexpr std::size_t operator"" _GiB(unsigned long long value) noexcept
{
  return std::size_t(value * 1024 * 1024 * 1024);
}

constexpr std::size_t operator"" _GB(unsigned long long value) noexcept
{
  return std::size_t(value * 1000 * 1000 * 1000);
}

} // namespace literals

} // namespace common
} // namespace dart

// -- Standard shared/weak pointers --
// Define a typedef for const and non-const version of shared_ptr and weak_ptr
// for the class X
#define DART_COMMON_DECLARE_SHARED_WEAK(X)                                     \
  class X;                                                                     \
  using X##Ptr = std::shared_ptr<X>;                                           \
  using Const##X##Ptr = std::shared_ptr<const X>;                              \
  using Weak##X##Ptr = std::weak_ptr<X>;                                       \
  using WeakConst##X##Ptr = std::weak_ptr<const X>;

#define DART_COMMON_DECLARE_SHARED_WEAK_STRUCT(X)                              \
  struct X;                                                                    \
  using X##Ptr = std::shared_ptr<X>;                                           \
  using Const##X##Ptr = std::shared_ptr<const X>;                              \
  using Weak##X##Ptr = std::weak_ptr<X>;                                       \
  using WeakConst##X##Ptr = std::weak_ptr<const X>;

// -- Standard shared/weak/unique pointers --
// Type aliases for const and non-const version of shared_ptr, weak_ptr, and
// unique_ptr for the class X
#define DART_COMMON_DECLARE_SMART_POINTERS(X)                                  \
  DART_COMMON_DECLARE_SHARED_WEAK(X)                                           \
  using Unique##X##Ptr = std::unique_ptr<X>;                                   \
  using UniqueConst##X##Ptr = std::unique_ptr<const X>;

#define DART_COMMON_DECLARE_SMART_POINTERS_STRUCT(X)                           \
  DART_COMMON_DECLARE_SHARED_WEAK_STRUCT(X)                                    \
  using Unique##X##Ptr = std::unique_ptr<X>;                                   \
  using UniqueConst##X##Ptr = std::unique_ptr<const X>;

#define DART_DEFINE_STRUCT_POINTERS_T1(x)                                      \
  template <typename T>                                                        \
  struct x;                                                                    \
  template <typename T>                                                        \
  using x##Ptr = ::std::shared_ptr<x<T>>;                                      \
  template <typename T>                                                        \
  using Const##x##Ptr = ::std::shared_ptr<const x<T>>;                         \
  template <typename T>                                                        \
  using x##WeakPtr = ::std::weak_ptr<x<T>>;                                    \
  template <typename T>                                                        \
  using Const##x##WeakPtr = ::std::weak_ptr<const x<T>>;                       \
  template <typename T>                                                        \
  using x##UniquePtr = ::std::unique_ptr<x<T>>;                                \
  template <typename T>                                                        \
  using Const##x##UniquePtr = ::std::unique_ptr<const x<T>>;                   \
  using x##f = x<float>;                                                       \
  using x##d = x<double>;                                                      \
  void _ANONYMOUS_FUNCTION_3()

#define DART_DEFINE_CLASS_POINTERS_T1(x)                                       \
  template <typename T>                                                        \
  class x;                                                                     \
  template <typename T>                                                        \
  using x##Ptr = ::std::shared_ptr<x<T>>;                                      \
  template <typename T>                                                        \
  using Const##x##Ptr = ::std::shared_ptr<const x<T>>;                         \
  template <typename T>                                                        \
  using x##WeakPtr = ::std::weak_ptr<x<T>>;                                    \
  template <typename T>                                                        \
  using Const##x##WeakPtr = ::std::weak_ptr<const x<T>>;                       \
  template <typename T>                                                        \
  using x##UniquePtr = ::std::unique_ptr<x<T>>;                                \
  template <typename T>                                                        \
  using Const##x##UniquePtr = ::std::unique_ptr<const x<T>>;                   \
  using x##f = x<float>;                                                       \
  using x##d = x<double>;                                                      \
  void _ANONYMOUS_FUNCTION_3()

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
      _DART_DEFINE_OBJECT_CREATOR(                                             \
          class_name,                                                          \
          DART_UNIQUE_PTR_CREATOR_NAME,                                        \
          std::unique_ptr,                                                     \
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

#include "dart/common/detail/memory_impl.hpp"
