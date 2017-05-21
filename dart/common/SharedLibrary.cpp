/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/common/SharedLibrary.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/detail/SharedLibraryManager.hpp"

#if DART_OS_LINUX || DART_OS_MACOS

#include <dlfcn.h>
#define DYNLIB_HANDLE void*
#define DYNLIB_LOAD(a) dlopen(a, RTLD_LAZY | RTLD_GLOBAL)
#define DYNLIB_GETSYM(a, b) dlsym(a, b)
#define DYNLIB_UNLOAD(a) dlclose(a)

#elif DART_OS_WINDOWS

#define WIN32_LEAN_AND_MEAN
#if !defined(NOMINMAX) && defined(_MSC_VER)
#define NOMINMAX // required to stop windows.h messing up std::min
#endif
#include <windows.h>
#define DYNLIB_HANDLE hInstance
// We can not use LOAD_WITH_ALTERED_SEARCH_PATH with relative paths
#define DYNLIB_LOAD(a) LoadLibraryEx(a, nullptr, 0)
#define DYNLIB_GETSYM(a, b) GetProcAddress(a, b)
#define DYNLIB_UNLOAD(a) !FreeLibrary(a)
struct HINSTANCE__;
typedef struct HINSTANCE__* hInstance;

#endif

namespace dart {
namespace common {

//==============================================================================
std::shared_ptr<SharedLibrary> SharedLibrary::create(const std::string& path)
{
  return detail::SharedLibraryManager::getSingleton().load(path);
}

//==============================================================================
SharedLibrary::SharedLibrary(ProtectedContructionTag, const std::string& path)
  : mFileName(path), mInstance(nullptr)
{
  auto nameWithExtension = mFileName;
#if DART_OS_LINUX
  // dlopen() does not add .so to the filename, like windows does for .dll
  if (nameWithExtension.find(".so") == std::string::npos)
    nameWithExtension += ".so";
#elif DART_OS_MACOS
  // dlopen() does not add .dylib to the filename, like windows does for .dll
  if (nameWithExtension.substr(fileName.length() - 6, 6) != ".dylib")
    nameWithExtension += ".dylib";
#elif DART_OS_WINDOWS
  // Although LoadLibraryEx will add .dll itself when you only specify the
  // library name, if you include a relative path then it does not. So, add it
  // to be sure.
  if (nameWithExtension.substr(nameWithExtension.length() - 4, 4) != ".dll")
    nameWithExtension += ".dll";
#endif

  mInstance
      = static_cast<DYNLIB_HANDLE>(DYNLIB_LOAD(nameWithExtension.c_str()));

  if (!mInstance)
  {
    dterr << "[SharedLibrary::load] Failed to load dynamic library '"
          << nameWithExtension << "': " << getLastError() << "\n";
  }
}

//==============================================================================
SharedLibrary::~SharedLibrary()
{
  if (!isValid())
    return;

  if (DYNLIB_UNLOAD(mInstance))
  {
    dterr << "[SharedLibrary::~SharedLibrary] Failed to unload library '"
          << mFileName << "': " << getLastError() << "\n";
  }
}

//==============================================================================
const std::string& SharedLibrary::getPath() const
{
  return mFileName;
}

//==============================================================================
bool SharedLibrary::isValid() const
{
  return (mInstance != nullptr);
}

//==============================================================================
void* SharedLibrary::getSymbol(const std::string& symbolName) const
{
  if (!isValid())
    return nullptr;

  auto symbol = DYNLIB_GETSYM(mInstance, symbolName.c_str());

  if (!symbol)
  {
    dtwarn << "[SharedLibrary::getSymbol] Failed to load a symbol '"
           << symbolName << "'.\n";
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
        FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        GetLastError(),
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR)&lpMsgBuf,
        0,
        nullptr
        );
  std::string ret = lpMsgBuf;
  // Free the buffer.
  LocalFree( lpMsgBuf );
  return ret;
#else
  return std::string("");
#endif
}

} // namespace common
} // namespace dart
