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

#include "dart/common/shared_library.hpp"

#include "dart/common/detail/shared_library_manager.hpp"
#include "dart/common/logging.hpp"

#if DART_OS_LINUX || DART_OS_MACOS

  #include <dlfcn.h>
  #define DYNLIB_LOAD(a) dlopen(a, RTLD_LAZY | RTLD_GLOBAL)
  #define DYNLIB_GETSYM(a, b) dlsym(a, b)
  #define DYNLIB_UNLOAD(a) dlclose(a)

#elif DART_OS_WINDOWS

  #define WIN32_LEAN_AND_MEAN
  // We can not use LOAD_WITH_ALTERED_SEARCH_PATH with relative paths
  #define DYNLIB_LOAD(a) LoadLibraryEx(a, nullptr, 0)
  #define DYNLIB_GETSYM(a, b) GetProcAddress(a, b)
  #define DYNLIB_UNLOAD(a) !FreeLibrary(a)

#endif

namespace dart {
namespace common {

//==============================================================================
std::shared_ptr<SharedLibrary> SharedLibrary::create(std::string_view path)
{
  return detail::SharedLibraryManager::getSingleton().load(path);
}

//==============================================================================
SharedLibrary::SharedLibrary(
    ProtectedConstructionTag, const std::string& canonicalPath)
  : mCanonicalPath(canonicalPath), mPath(canonicalPath), mInstance(nullptr)
{
  mInstance = static_cast<DYNLIB_HANDLE>(DYNLIB_LOAD(canonicalPath.c_str()));

  DART_ERROR_IF(
      !mInstance,
      "Failed to load dynamic library '{}': {}",
      canonicalPath,
      getLastError());
}

//==============================================================================
SharedLibrary::~SharedLibrary()
{
  if (!isValid())
    return;

  DART_ERROR_IF(
      DYNLIB_UNLOAD(mInstance),
      "Failed to unload library '{}': {}",
      mPath,
      getLastError());
}

//==============================================================================
const common::filesystem::path& SharedLibrary::getCanonicalPath() const
{
  return mCanonicalPath;
}

//==============================================================================
const std::string& SharedLibrary::path() const
{
  return mPath;
}

//==============================================================================
bool SharedLibrary::isValid() const
{
  return (mInstance != nullptr);
}

//==============================================================================
void* SharedLibrary::getSymbol(std::string_view symbolName) const
{
  if (!isValid())
    return nullptr;

  const std::string symbolNameString(symbolName);
  auto symbol = DYNLIB_GETSYM(mInstance, symbolNameString.c_str());

  if (!symbol) {
    DART_WARN("Failed to load a symbol '{}'.", symbolNameString);
    return nullptr;
  }

  return static_cast<void*>(symbol);
}

//==============================================================================
std::string SharedLibrary::getLastError() const
{
#if DART_OS_LINUX || DART_OS_MACOS
  return std::string(dlerror());
#elif DART_OS_WINDOWS
  LPTSTR lpMsgBuf;
  FormatMessage(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM
          | FORMAT_MESSAGE_IGNORE_INSERTS,
      nullptr,
      GetLastError(),
      MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      (LPTSTR)&lpMsgBuf,
      0,
      nullptr);
  std::string ret = lpMsgBuf;
  // Free the buffer.
  LocalFree(lpMsgBuf);
  return ret;
#else
  return std::string("");
#endif
}

} // namespace common
} // namespace dart
