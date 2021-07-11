/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COMMON_SMARTPOINTER_HPP_
#define DART_COMMON_SMARTPOINTER_HPP_

#include <memory>

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
  void _ANONYMOUS_FUNCTION_3()

#endif // DART_COMMON_SMARTPOINTER_HPP_
