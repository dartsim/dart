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

#ifndef DART_COMMON_SHAREDLIBRARY_HPP_
#define DART_COMMON_SHAREDLIBRARY_HPP_

#include <string>
#include "dart/common/Platform.hpp"

#if DART_OS_LINUX
#define DYNLIB_HANDLE void*
#elif DART_OS_MACOS
#define DYNLIB_HANDLE void*
#elif DART_OS_WINDOWS
#define DYNLIB_HANDLE hInstance
#endif

namespace dart {
namespace common {

/// SharedLibrary is a RAII object wrapping a shared library.
class SharedLibrary
{
public:
  /// Constructor
  ///
  /// \param[in] The filename of the shared library. If the filename doesn't
  /// include the extension, this function will use the best guess depending on
  /// the OS (e.g., '.so' for Linux, '.dylib' for macOS, and '.dll' for
  /// Windows).
  explicit SharedLibrary(const std::string& fileName);

  /// Destructor
  virtual ~SharedLibrary();

  /// Returns the file name of the shared library
  const std::string& getFileName() const;

  /// Returns true if the shared library loading was successful.
  bool isGood() const;

  /// Returns a symbol from the shared library
  void* getSymbol(const std::string& symbolName) const;

protected:
  void load();

  /// Returns the last loading error
  std::string dynlibError() const;

protected:
  /// Filename of the shared library.
  std::string mFileName;

  /// Handle to the loaded library.
  DYNLIB_HANDLE mInstance;
};

} // namespace common
} // namespace dart

#endif // DART_COMMON_SHAREDLIBRARY_HPP_
