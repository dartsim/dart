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

#include "dart/common/shared_library.hpp"

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

namespace dart::common {

//==============================================================================
struct SharedLibrary::Implementation
{
  std::string path;
  HandlePtr handle{nullptr};

  Implementation()
  {
    // Do nothing
  }
};

//==============================================================================
std::string get_last_error()
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

//==============================================================================
SharedLibrary::SharedLibrary(const std::string& path)
  : m_impl(std::make_unique<Implementation>())
{
  // Load shared library
  load(path);
}

//==============================================================================
SharedLibrary::~SharedLibrary()
{
  if (!is_loaded()) {
    return;
  }

  const bool unloaded = DYNLIB_UNLOAD(m_impl->handle);

  if (!unloaded) {
    DART_ERROR("Failed to unload shared library: {}", get_last_error());
  }
}

//==============================================================================
bool SharedLibrary::load(const std::string& path)
{
  // TODO(JS): We use the handle to identify the shared library. dlopen(~)
  // returns the same handle for the exactly same path. This means passing
  // "./foo.so",
  // "././foo.so", or "../foosymlink.so" where "foosymlink.so" is a symlink to
  // "foo.so" returns different handles. To avoid this, we should always convert
  // the path to the canonical path before passing to dlopen(~) to get the same
  // handle for the same shared library. Reference:
  // https://stackoverflow.com/a/12148491/3122234
  m_impl->path = path;
  m_impl->handle = DYNLIB_LOAD(path.c_str());

  if (m_impl->handle == nullptr) {
    DART_ERROR(
        "Failed to load the shared library '{}': {}", path, get_last_error());
    return false;
  }

  return true;
}

//==============================================================================
bool SharedLibrary::is_loaded() const
{
  return m_impl->handle != nullptr;
}

//==============================================================================
void* SharedLibrary::symbol(const std::string& symbol_name) const
{
  if (!is_loaded()) {
    return nullptr;
  }

  void* symbol = DYNLIB_GETSYM(m_impl->handle, symbol_name.c_str());

  if (symbol == nullptr) {
    DART_ERROR(
        "Failed to get symbol '{}' from shared library '{}: {}",
        symbol_name,
        m_impl->path,
        get_last_error());
    return nullptr;
  }

  return symbol;
}

//==============================================================================
const std::string& SharedLibrary::path() const
{
  return m_impl->path;
}

//==============================================================================
SharedLibrary::HandlePtr SharedLibrary::mutable_handle_ptr()
{
  return m_impl->handle;
}

//==============================================================================
SharedLibrary::ConstHandlePtr SharedLibrary::handle_ptr() const
{
  return m_impl->handle;
}

} // namespace dart::common
