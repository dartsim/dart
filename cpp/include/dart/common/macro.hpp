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

#include <cassert>

#include "dart/common/logging.hpp"

// DART_NUM_ARGS(<arg1> [, <arg2> [, ...]])
#define DETAIL_DART_NUM_ARGS(z, a, b, c, d, e, f, cnt, ...) cnt
#define DART_NUM_ARGS(...)                                                     \
  DETAIL_DART_NUM_ARGS(, ##__VA_ARGS__, 6, 5, 4, 3, 2, 1, 0)

// DART_CONCAT(a, b)
#define DETAIL_DART_CONCAT(a, b) a##b
#define DART_CONCAT(a, b) DETAIL_DART_CONCAT(a, b)

// DART_UNUSED(<variable1> [, <variable2> [, ...]])
#define DETAIL_DART_UNUSED_0()
#define DETAIL_DART_UNUSED_1(a) (void)(a)
#define DETAIL_DART_UNUSED_2(a, b) (void)(a), DETAIL_DART_UNUSED_1(b)
#define DETAIL_DART_UNUSED_3(a, b, c) (void)(a), DETAIL_DART_UNUSED_2(b, c)
#define DETAIL_DART_UNUSED_4(a, b, c, d)                                       \
  (void)(a), DETAIL_DART_UNUSED_3(b, c, d)
#define DETAIL_DART_UNUSED_5(a, b, c, d, e)                                    \
  (void)(a), DETAIL_DART_UNUSED_4(b, c, d, e)
#define DETAIL_DART_UNUSED_6(a, b, c, d, e, f)                                 \
  (void)(a), DETAIL_DART_UNUSED_5(b, c, d, e, f)
#define DART_UNUSED(...)                                                       \
  DART_CONCAT(DETAIL_DART_UNUSED_, DART_NUM_ARGS(__VA_ARGS__))(__VA_ARGS__)

// DART_ASSERT(<expression> [, <message>])
#define DETAIL_DART_ASSERT_1(condition) assert(condition)
#define DETAIL_DART_ASSERT_2(condition, message) assert((condition) && #message)
#define DART_ASSERT(...)                                                       \
  DART_CONCAT(DETAIL_DART_ASSERT_, DART_NUM_ARGS(__VA_ARGS__))                 \
  (__VA_ARGS__)

#if defined(_MSC_VER)

  #if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT

    #if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template type class_name<float>;                                \
        extern template type class_name<double>;                               \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(type, module_name, class_name)              \
        template type DART_##module_name##_API class_name<float>;              \
        template type DART_##module_name##_API class_name<double>;

    #else

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template class class_name<float>;                               \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(class_name)                                 \
        template type DART_##module_name##_API class_name<float>;

    #endif

  #elif DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE

    #if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template type class_name<float>;                                \
        extern template type class_name<double>;                               \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(type, module_name, class_name)              \
        template type DART_##module_name##_API class_name<float>;              \
        template type DART_##module_name##_API class_name<double>;

    #else

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template type class_name<double>;                               \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(type, module_name, class_name)              \
        template type DART_##module_name##_API class_name<double>;

    #endif

  #else

    #define DART_TEMPLATE_HEADER(type, module_name, class_name)                \
      using class_name##f = class_name<float>;                                 \
      using class_name##d = class_name<double>;
    #define DART_TEMPLATE_SOURCE(type, module_name, class_name)

  #endif

#else

  #if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT

    #if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template type DART_##module_name##_API class_name<float>;       \
        extern template type DART_##module_name##_API class_name<double>;      \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(type, module_name, class_name)              \
        template type class_name<float>;                                       \
        template type class_name<double>;

    #else

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template class DART_##module_name##_API class_name<float>;      \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(class_name) template type class_name<float>;

    #endif

  #elif DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE

    #if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template type DART_##module_name##_API class_name<float>;       \
        extern template type DART_##module_name##_API class_name<double>;      \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(type, module_name, class_name)              \
        template type class_name<float>;                                       \
        template type class_name<double>;

    #else

      #define DART_TEMPLATE_HEADER(type, module_name, class_name)              \
        extern template type DART_##module_name##_API class_name<double>;      \
        using class_name##f = class_name<float>;                               \
        using class_name##d = class_name<double>;                              \
        using class_name##ld = class_name<long double>;
      #define DART_TEMPLATE_SOURCE(type, module_name, class_name)              \
        template type class_name<double>;

    #endif

  #else

    #define DART_TEMPLATE_HEADER(type, module_name, class_name)                \
      using class_name##f = class_name<float>;                                 \
      using class_name##d = class_name<double>;
    #define DART_TEMPLATE_SOURCE(type, module_name, class_name)

  #endif

#endif

#define DART_TEMPLATE_CLASS_HEADER(module_name, class_name)                    \
  DART_TEMPLATE_HEADER(class, module_name, class_name)
#define DART_TEMPLATE_STRUCT_HEADER(module_name, class_name)                   \
  DART_TEMPLATE_HEADER(struct, module_name, class_name)
#define DART_TEMPLATE_CLASS_SOURCE(module_name, class_name)                    \
  DART_TEMPLATE_SOURCE(class, module_name, class_name)
#define DART_TEMPLATE_STRUCT_SOURCE(module_name, class_name)                   \
  DART_TEMPLATE_SOURCE(struct, module_name, class_name)

#define DART_TEMPLATE_CLASS_SCALAR(class_name)                                 \
  using class_name##f = class_name<float>;                                     \
  using class_name##d = class_name<double>;                                    \
  using class_name##ld = class_name<long double>;

#define DART_NOT_IMPLEMENTED                                                   \
  DART_FATAL("Not implemented: {}:{}", __FILE__, __LINE__);                    \
  void(0)

#define DART_SCALAR_SHOULD_BE_FLOATING_POINT(class_name, scalar_type)          \
  static_assert(                                                               \
      ::std::is_floating_point_v<scalar_type>,                                 \
      "#class_name only can instantiated with floating point types");          \
  void _ANONYMOUS_FUNCTION_11()

// This macro is used to mark all the class that inherit
// virtually from another to avoid problems on MSVC
// See https://github.com/dartsim/dart/issues/1522
#if defined(_MSC_VER)
  #define DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN __pragma(vtordisp(push, 2))
  #define DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END __pragma(vtordisp(pop))
#else
  #define DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
  #define DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END
#endif
