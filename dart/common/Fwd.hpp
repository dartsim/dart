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

#include <dart/common/Export.hpp>
#include <dart/common/Macros.hpp>

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

namespace dart::common {

} // namespace dart::common
