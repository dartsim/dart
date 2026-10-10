
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

#ifndef DARTPY_C_API_HPP_
#define DARTPY_C_API_HPP_

// Python.h must precede standard headers; dart/config.hpp includes none.
#include <dart/config.hpp>

#include <Python.h>

#include <memory>
#include <typeinfo>

#include <cstdint>

// The table below passes std::type_info and std::shared_ptr across modules, so
// an extension that reads it must use dartpy's DART version and C++ standard
// library ABI. kAbiTag records both; build it from the same headers on each
// side.
#define DARTPY_C_API_STRINGIFY_IMPL(x) #x
#define DARTPY_C_API_STRINGIFY(x) DARTPY_C_API_STRINGIFY_IMPL(x)
#if defined(_LIBCPP_VERSION)
  #define DARTPY_C_API_STDLIB                                                  \
    "libc++" DARTPY_C_API_STRINGIFY(_LIBCPP_ABI_VERSION)
#elif defined(__GLIBCXX__)
  #define DARTPY_C_API_STDLIB                                                  \
    "libstdc++" DARTPY_C_API_STRINGIFY(_GLIBCXX_USE_CXX11_ABI)
#elif defined(_MSC_VER)
  #define DARTPY_C_API_STDLIB                                                  \
    "msvc" DARTPY_C_API_STRINGIFY(_ITERATOR_DEBUG_LEVEL)
#else
  #define DARTPY_C_API_STDLIB "unknown"
#endif
#if defined(_GLIBCXX_DEBUG) || (defined(_MSC_VER) && defined(_DEBUG))
  #define DARTPY_C_API_DEBUG "-debug"
#else
  #define DARTPY_C_API_DEBUG ""
#endif

namespace dartpy {
namespace c_api {

/// Version of CApi; it changes whenever the table changes incompatibly.
inline constexpr std::uint32_t kVersion = 1;

/// Name of the capsule that dartpy exports as `dartpy._C_API`.
inline constexpr char kCapsuleName[] = "dartpy._C_API";

/// DART version and C++ standard library ABI of the including module.
inline constexpr char kAbiTag[]
    = "dart-" DART_VERSION "-" DARTPY_C_API_STDLIB DARTPY_C_API_DEBUG;

/// How a wrapped borrowed pointer relates to its Python parent.
enum Policy : int
{
  /// Borrow the object. Graph objects still keep their skeleton alive.
  kReference = 0,
  /// Borrow the object and keep the parent alive while the result lives.
  kReferenceInternal = 1,
};

/// Function table that lets other extension modules exchange DART objects with
/// dartpy. Each function returns null (or a negative value) with a Python
/// exception set on failure; a TypeError means the object has the wrong type.
struct CApi
{
  std::uint32_t version;
  std::uint32_t size;
  const char* abiTag;

  /// Returns the native object behind a dartpy wrapper, converted to `type`.
  void* (*unwrap)(PyObject* object, const std::type_info& type);

  /// Stores shared ownership of the native object behind a dartpy wrapper in
  /// `owner`. `complete` is the object's most-derived address. Returns 0.
  int (*share)(PyObject* object, void* complete, std::shared_ptr<void>* owner);

  /// Returns a new reference to the dartpy wrapper of a borrowed pointer whose
  /// static type is `type`, reusing an existing wrapper when there is one.
  PyObject* (*wrap)(
      const std::type_info& type,
      const std::type_info& dynamicType,
      void* complete,
      void* pointer,
      int policy,
      PyObject* parent);

  /// Like wrap(), but a new wrapper also shares `owner`.
  PyObject* (*wrapShared)(
      const std::type_info& type,
      const std::type_info& dynamicType,
      void* complete,
      void* pointer,
      const std::shared_ptr<void>* owner);

  /// Copies a dartpy value such as `dartpy.math.Isometry3` into `out`, which
  /// points to a `type`. With `convert`, also accepts what the dartpy type
  /// implicitly converts from. Returns 0, or 1 without an exception when the
  /// object is not convertible.
  int (*loadValue)(
      PyObject* object, const std::type_info& type, void* out, int convert);

  /// Returns a new dartpy object holding a copy of `*value`.
  PyObject* (*castValue)(const std::type_info& type, const void* value);
};

} // namespace c_api
} // namespace dartpy

#endif // DARTPY_C_API_HPP_
