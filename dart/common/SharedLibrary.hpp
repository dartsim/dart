/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_SHAREDLIBRARY_HPP_
#define DART_COMMON_SHAREDLIBRARY_HPP_

#include <memory>
#include <string>
#include <boost/filesystem.hpp>
#include "dart/common/Platform.hpp"

#if DART_OS_LINUX

#define DYNLIB_HANDLE void*

#elif DART_OS_MACOS

#define DYNLIB_HANDLE void*

#elif DART_OS_WINDOWS

#ifdef NOMINMAX
#include <windows.h>
#else
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif
using hInstance = HINSTANCE__*;
#define DYNLIB_HANDLE hInstance

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
  static std::shared_ptr<SharedLibrary> create(
      const boost::filesystem::path& path);

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
  explicit SharedLibrary(
      ProtectedConstructionTag, const boost::filesystem::path& path);

  /// Destructor.
  virtual ~SharedLibrary();

  /// Returns the path to the shared library file.
  const boost::filesystem::path& getCanonicalPath() const;

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
  boost::filesystem::path mCanonicalPath;

  /// Handle to the loaded library.
  DYNLIB_HANDLE mInstance;

private:
  /// Returns the last loading error.
  std::string getLastError() const;
};

} // namespace common
} // namespace dart

#endif // DART_COMMON_SHAREDLIBRARY_HPP_
