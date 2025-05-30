/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_COMMON_SHAREDLIBRARY_HPP_
#define DART_COMMON_SHAREDLIBRARY_HPP_

#include <dart/common/Deprecated.hpp>
#include <dart/common/Filesystem.hpp>
#include <dart/common/Platform.hpp>

#include <memory>
#include <string>

#if DART_OS_LINUX

  #define DYNLIB_HANDLE void*

#elif DART_OS_MACOS

  #define DYNLIB_HANDLE void*

#elif DART_OS_WINDOWS

  #include <dart/common/IncludeWindows.hpp>
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

namespace dart {
namespace common {

namespace detail {
class SharedLibraryManager;
} // namespace detail

/// SharedLibrary is a RAII object wrapping a shared library.
class SharedLibrary
{
protected:
  enum ProtectedConstructionTag
  {
    ProtectedConstruction
  };

public:
  /// Creates a SharedLibrary from a path to the shared library.
  ///
  /// \note SharedLibrary should be always created from this create function.
  /// \param[in] path The path to the shared library. The path can be a relative
  /// path or an absolute path. If the path doens't exist this function returns
  /// nullptr. If the path exist, the path will be stored as the canonical path
  /// where a canonical path is an absolute path that has no elements which are
  /// symbolic links, and no dot or dot dot elements such as
  /// "/path/../to/yourfile".
  /// \return Pointer to the created SharedLibrary upon success in loading.
  /// Otherwise, returns nullptr.
  /// \deprecated Deprecated in 6.10. Please use create(const std::string&)
  /// instead.
  DART_DEPRECATED(6.10)
  static std::shared_ptr<SharedLibrary> create(
      const common::filesystem::path& path);

  /// Creates a SharedLibrary from a path to the shared library.
  ///
  /// \note SharedLibrary should be always created from this create function.
  /// \param[in] path The path to the shared library. The path can be a relative
  /// path or an absolute path. If the path doens't exist this function returns
  /// nullptr. If the path exist, the path will be stored as the canonical path
  /// where a canonical path is an absolute path that has no elements which are
  /// symbolic links, and no dot or dot dot elements such as
  /// "/path/../to/yourfile".
  /// \return Pointer to the created SharedLibrary upon success in loading.
  /// Otherwise, returns nullptr.
  static std::shared_ptr<SharedLibrary> create(const std::string& path);

  /// Constructs from a path to the shared library.
  ///
  /// This constructor is only called by detail::SharedLibraryManager.
  /// ProtectedConstructionTag is necessary to enforce creating SharedLibrary
  /// using std::make_shared.
  ///
  /// \note Please use create() to contruct SharedLibrary instead of this
  /// constructor.
  /// \param[in] path The canonical path to the shared library.
  /// \return Pointer to the created SharedLibrary upon success in loading.
  /// Otherwise, returns nullptr.
  /// \deprecated Deprecated in 6.10. Please use
  /// SharedLibrary(ProtectedConstructionTag, const std::string&) instead.
  DART_DEPRECATED(6.10)
  explicit SharedLibrary(
      ProtectedConstructionTag, const common::filesystem::path& path);

  /// Constructs from a path to the shared library.
  ///
  /// This constructor is only called by detail::SharedLibraryManager.
  /// ProtectedConstructionTag is necessary to enforce creating SharedLibrary
  /// using std::make_shared.
  ///
  /// \note Please use create() to contruct SharedLibrary instead of this
  /// constructor.
  /// \param[in] path The canonical path to the shared library.
  /// \return Pointer to the created SharedLibrary upon success in loading.
  /// Otherwise, returns nullptr.
  explicit SharedLibrary(ProtectedConstructionTag, const std::string& path);

  /// Destructor.
  virtual ~SharedLibrary();

  /// Returns the path to the shared library file.
  const common::filesystem::path& getCanonicalPath() const;

  /// Returns the path to the shared library file.
  const std::string& path() const;

  /// Returns true if the shared library loading was successful.
  bool isValid() const;

  /// Returns a symbol from the shared library if it exists. Return nullptr
  /// otherwise.
  ///
  /// You have to reinterpret_cast the return value to the appropriate type to
  /// make use of the void* returned by this function.
  void* getSymbol(const std::string& symbolName) const;

protected:
  friend class detail::SharedLibraryManager;

  /// Canonical path to the shared library where a canonical path is an absolute
  /// path that has no elements which are symbolic links, and no dot or dot dot
  /// elements such as "/path/../to/yourfile".
  /// \deprecated Use mCanonicalPath2 instead.
  common::filesystem::path mCanonicalPath;
  // TODO(JS): Remove in DART 7.

  /// Canonical path to the shared library where a canonical path is an absolute
  /// path that has no elements which are symbolic links, and no dot or dot dot
  /// elements such as "/path/../to/yourfile".
  std::string mPath;

  /// Handle to the loaded library.
  DYNLIB_HANDLE mInstance;

private:
  /// Returns the last loading error.
  std::string getLastError() const;
};

} // namespace common
} // namespace dart

#endif // DART_COMMON_SHAREDLIBRARY_HPP_
