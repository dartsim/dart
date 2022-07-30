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

#pragma once

#include <memory>
#include <string>

#include "dart/common/export.hpp"
#include "dart/common/platform.hpp"

#if DART_OS_LINUX

  #define DYNLIB_HANDLE void*

#elif DART_OS_MACOS

  #define DYNLIB_HANDLE void*

#elif DART_OS_WINDOWS

//#ifdef NOMINMAX
//  #include <windows.h>
//#else
//  #define NOMINMAX
//  #include <windows.h>
//  #undef NOMINMAX
//#endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
using hInstance = HINSTANCE__*;
  #define DYNLIB_HANDLE hInstance

#endif

#if DART_OS_LINUX
static constexpr const char* DART_SHARED_LIB_EXTENSION = "so";
#elif DART_OS_MACOS
static constexpr const char* DART_SHARED_LIB_EXTENSION = "dylib";
#elif DART_OS_WINDOWS
static constexpr const char* DART_SHARED_LIB_EXTENSION = "dll";
#else
  #error Unhandled platform
#endif

#if DART_OS_LINUX
static constexpr const char* DART_SHARED_LIB_PREFIX = "lib";
#elif DART_OS_MACOS
static constexpr const char* DART_SHARED_LIB_PREFIX = "lib";
#elif DART_OS_WINDOWS
static constexpr const char* DART_SHARED_LIB_PREFIX = "";
#else
  #error Unhandled platform
#endif

namespace dart::common {

class DART_COMMON_API SharedLibrary final
{
public:
  using HandlePtr = DYNLIB_HANDLE;
  using ConstHandlePtr = const DYNLIB_HANDLE;

  /**
   * Constructs from shared library path.
   * @param path The path to the shared library.
   */
  explicit SharedLibrary(const std::string& path = "");

  /**
   * Destructor
   */
  ~SharedLibrary();

  /**
   * Loads a shared library of given path.
   */
  bool load(const std::string& path);

  /**
   * Returns true if the shared library is loaded.
   */
  bool is_loaded() const;

  /**
   * Returns function of @c symbol_name
   *
   * @param[in] symbol_name The demangled symbol name.
   */
  void* symbol(const std::string& symbol_name) const;

  /**
   * Returns the path to the shared library
   */
  const std::string& path() const;

  /**
   * Returns the shared library handle.
   */
  HandlePtr mutable_handle_ptr();

  /**
   * Returns the shared library handle.
   */
  ConstHandlePtr handle_ptr() const;

private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

} // namespace dart::common
