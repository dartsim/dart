/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#ifndef DART_COMMON_FILESYSTEM_HPP_
#define DART_COMMON_FILESYSTEM_HPP_

#include "dart/common/Platform.hpp"

#if !defined(DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL)

  // Check for feature test macro for <filesystem>
  #if defined(__cpp_lib_filesystem)
    #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 0

  // Check for feature test macro for <experimental/filesystem>
  #elif defined(__cpp_lib_experimental_filesystem)
    #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 1

  // We can't check if headers exist...
  // Let's assume experimental to be safe
  #elif !defined(__has_include)
    #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 1

  // Check if the header "<filesystem>" exists
  #elif __has_include(<filesystem>)

    // If we're compiling on Visual Studio and are not compiling with C++17, we
    // need to use experimental
    #ifdef _MSC_VER

      // Check and include header that defines "_HAS_CXX17"
      #if __has_include(<yvals_core.h>)
        #include <yvals_core.h>

        // Check for enabled C++17 support
        #if defined(_HAS_CXX17) && _HAS_CXX17
          // We're using C++17, so let's use the normal version
          #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 0
        #endif
      #endif

      // If the marco isn't defined yet, that means any of the other VS specific
      // checks failed, so we need to use experimental
      #ifndef DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL
        #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 1
      #endif

    // Not on Visual Studio. Let's use the normal version
    #else // #ifdef _MSC_VER
      #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 0
    #endif

  // Check if the header "<filesystem>" exists
  #elif __has_include(<experimental/filesystem>)
    #define DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL 1

  // Fail if neither header is available with a nice error message
  #else
    #error Could not find system header "<filesystem>" or "<experimental/filesystem>"
  #endif

  // We priously determined that we need the exprimental version
  #if DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL
    // Include it
    #include <experimental/filesystem>

namespace dart::common {

namespace filesystem = ::std::experimental::filesystem;
using error_code = ::std::error_code;

} // namespace dart::common

  // We have a decent compiler and can use the normal version
  #else
    // Include it
    #include <filesystem>

namespace dart::common {
namespace filesystem = ::std::filesystem;
using error_code = ::std::error_code;
} // namespace dart::common

  #endif

#endif // #ifndef DART_INCLUDE_STD_FILESYSTEM_EXPERIMENTAL

#endif // #ifndef DART_COMMON_FILESYSTEM_HPP_
