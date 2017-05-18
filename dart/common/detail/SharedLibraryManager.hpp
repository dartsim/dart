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

#ifndef DART_COMMON_DETAIL_SHAREDLIBRARYMANAGER_HPP_
#define DART_COMMON_DETAIL_SHAREDLIBRARYMANAGER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include "dart/common/Singleton.hpp"

namespace dart {
namespace common {

class SharedLibrary;

namespace detail {

class SharedLibraryManager final : public Singleton<SharedLibraryManager>
{
public:
  /// Loads a shared library of fileName.
  ///
  /// \param[in] The filename of the shared library. If the filename doesn't
  /// include the extension, this function will use the best guess depending on
  /// the OS (e.g., '.so' for Linux, '.dylib' for macOS, and '.dll' for
  /// Windows).
  /// \return Pointer to the shared library upon success. Otherwise, returns
  /// nullptr.
  std::shared_ptr<SharedLibrary> load(const std::string& fileName);

protected:
  friend class Singleton<SharedLibraryManager>;

protected:
  std::unordered_map<std::string, std::weak_ptr<SharedLibrary>> mLibraries;
};

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_SHAREDLIBRARYMANAGER_HPP_
