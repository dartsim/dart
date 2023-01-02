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

#ifndef DART_COMMON_DETAIL_SHAREDLIBRARYMANAGER_HPP_
#define DART_COMMON_DETAIL_SHAREDLIBRARYMANAGER_HPP_

#include "dart/common/Deprecated.hpp"
#include "dart/common/Filesystem.hpp"
#include "dart/common/Singleton.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace dart {
namespace common {

class SharedLibrary;

namespace detail {

class SharedLibraryManager final : public Singleton<SharedLibraryManager>
{
public:
  /// Loads the shared library with the specified path.
  ///
  /// \param[in] path The path to the shared library. If the path doesn't
  /// include the extension, this function will use the best guess depending on
  /// the OS (e.g., '.so' for Linux, '.dylib' for macOS, and '.dll' for
  /// Windows).
  /// \return Pointer to the shared library upon success. Otherwise, returns
  /// nullptr.
  /// \deprecated Deprecated in 6.10. Please use load(const std::string&)
  /// instead.
  DART_DEPRECATED(6.10)
  std::shared_ptr<SharedLibrary> load(const common::filesystem::path& path);

  /// Loads the shared library with the specified path.
  ///
  /// \param[in] path The path to the shared library. If the path doesn't
  /// include the extension, this function will use the best guess depending on
  /// the OS (e.g., '.so' for Linux, '.dylib' for macOS, and '.dll' for
  /// Windows).
  /// \return Pointer to the shared library upon success. Otherwise, returns
  /// nullptr.
  std::shared_ptr<SharedLibrary> load(const std::string& path);

protected:
  friend class Singleton<SharedLibraryManager>;

protected:
  struct FileSystemHash
  {
    size_t operator()(const ::dart::common::filesystem::path& p) const
    {
      return ::dart::common::filesystem::hash_value(p);
    }
  };

  /// Map from library path to the library instances.
  std::unordered_map<
      common::filesystem::path,
      std::weak_ptr<SharedLibrary>,
      FileSystemHash>
      mLibraries;
  // TODO(JS): Remove this in DART 7.

  /// Map from library path to the library instances.
  std::unordered_map<std::string, std::weak_ptr<SharedLibrary>>
      mSharedLibraries;
};

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_SHAREDLIBRARYMANAGER_HPP_
