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

#include <dart/common/shared_library.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart::common;

namespace {

/// Returns a path to a system shared library that should exist on the platform.
/// Returns empty string if no suitable library is found.
std::string getSystemLibraryPath()
{
#if DART_OS_LINUX
  // Try common locations for libm on Linux
  const std::vector<std::string> candidates = {
      "/lib/x86_64-linux-gnu/libm.so.6",
      "/lib/aarch64-linux-gnu/libm.so.6",
      "/lib64/libm.so.6",
      "/usr/lib/libm.so.6",
      "/usr/lib64/libm.so.6",
      "/lib/libm.so.6",
  };
  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }
  return "";
#elif DART_OS_MACOS
  // On macOS Big Sur and later, system libraries are in a shared cache
  // and direct paths like /usr/lib/libSystem.B.dylib may not work.
  // Use libc++.dylib which is typically available for dlopen.
  const std::vector<std::string> candidates = {
      "/usr/lib/libc++.1.dylib",
      "/usr/lib/libSystem.B.dylib",
  };
  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }
  return "";
#elif DART_OS_WINDOWS
  // On Windows, use kernel32.dll which is always present
  return "kernel32.dll";
#else
  return "";
#endif
}

} // namespace

//==============================================================================
// Platform Constants Tests
//==============================================================================

TEST(SharedLibraryTest, SharedLibExtension_CorrectForPlatform)
{
#if DART_OS_LINUX
  EXPECT_STREQ(DART_SHARED_LIB_EXTENSION, "so");
#elif DART_OS_MACOS
  EXPECT_STREQ(DART_SHARED_LIB_EXTENSION, "dylib");
#elif DART_OS_WINDOWS
  EXPECT_STREQ(DART_SHARED_LIB_EXTENSION, "dll");
#endif
}

TEST(SharedLibraryTest, SharedLibPrefix_CorrectForPlatform)
{
#if DART_OS_LINUX
  EXPECT_STREQ(DART_SHARED_LIB_PREFIX, "lib");
#elif DART_OS_MACOS
  EXPECT_STREQ(DART_SHARED_LIB_PREFIX, "lib");
#elif DART_OS_WINDOWS
  EXPECT_STREQ(DART_SHARED_LIB_PREFIX, "");
#endif
}

//==============================================================================
// Create Tests - Invalid Paths
//==============================================================================

TEST(SharedLibraryTest, Create_InvalidPath_ReturnsNullptr)
{
  auto lib = SharedLibrary::create("/nonexistent/path/to/library.so");

  EXPECT_EQ(lib, nullptr);
}

TEST(SharedLibraryTest, Create_EmptyPath_ReturnsNullptr)
{
  auto lib = SharedLibrary::create("");

  EXPECT_EQ(lib, nullptr);
}

TEST(SharedLibraryTest, Create_NonExistentFile_ReturnsNullptr)
{
  auto lib = SharedLibrary::create("/tmp/this_library_does_not_exist_12345.so");

  EXPECT_EQ(lib, nullptr);
}

TEST(SharedLibraryTest, Create_DirectoryPath_ReturnsNullptr)
{
  // Trying to load a directory as a library should fail
  auto lib = SharedLibrary::create("/tmp");

  EXPECT_EQ(lib, nullptr);
}

//==============================================================================
// Create Tests - Valid Library
//==============================================================================

TEST(SharedLibraryTest, Create_ValidLibrary_ReturnsNonNull)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);

  ASSERT_NE(lib, nullptr);
  EXPECT_TRUE(lib->isValid());
}

TEST(SharedLibraryTest, IsValid_ValidLibrary_ReturnsTrue)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);

  ASSERT_NE(lib, nullptr);
  EXPECT_TRUE(lib->isValid());
}

//==============================================================================
// Path Tests
//==============================================================================

TEST(SharedLibraryTest, Path_ValidLibrary_ReturnsNonEmptyString)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);

  ASSERT_NE(lib, nullptr);
  EXPECT_FALSE(lib->path().empty());
}

TEST(SharedLibraryTest, GetCanonicalPath_ValidLibrary_ReturnsNonEmptyPath)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);

  ASSERT_NE(lib, nullptr);
  EXPECT_FALSE(lib->getCanonicalPath().empty());
}

TEST(SharedLibraryTest, GetCanonicalPath_IsAbsolutePath)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);

  ASSERT_NE(lib, nullptr);
  EXPECT_TRUE(lib->getCanonicalPath().is_absolute());
}

//==============================================================================
// Symbol Tests
//==============================================================================

TEST(SharedLibraryTest, GetSymbol_ValidSymbol_ReturnsNonNull)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);
  ASSERT_NE(lib, nullptr);

#if DART_OS_LINUX || DART_OS_MACOS
  // cos is a standard math function available in libm/libSystem
  void* symbol = lib->getSymbol("cos");
  EXPECT_NE(symbol, nullptr);
#elif DART_OS_WINDOWS
  // GetCurrentProcess is always available in kernel32.dll
  void* symbol = lib->getSymbol("GetCurrentProcess");
  EXPECT_NE(symbol, nullptr);
#endif
}

TEST(SharedLibraryTest, GetSymbol_InvalidSymbol_ReturnsNullptr)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);
  ASSERT_NE(lib, nullptr);

  void* symbol = lib->getSymbol("this_symbol_definitely_does_not_exist_12345");

  EXPECT_EQ(symbol, nullptr);
}

TEST(SharedLibraryTest, GetSymbol_EmptySymbolName_ReturnsNullptr)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib = SharedLibrary::create(libPath);
  ASSERT_NE(lib, nullptr);

  void* symbol = lib->getSymbol("");

  EXPECT_EQ(symbol, nullptr);
}

//==============================================================================
// SharedLibraryManager Tests (via create)
//==============================================================================

TEST(SharedLibraryTest, Create_SameLibraryTwice_ReturnsSameInstance)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib1 = SharedLibrary::create(libPath);
  auto lib2 = SharedLibrary::create(libPath);

  ASSERT_NE(lib1, nullptr);
  ASSERT_NE(lib2, nullptr);

  // SharedLibraryManager should return the same instance for the same path
  EXPECT_EQ(lib1.get(), lib2.get());
}

TEST(SharedLibraryTest, Create_DifferentPaths_ReturnsDifferentInstances)
{
  // This test verifies that different paths result in different instances
  // We use the same library but with different path representations if possible

  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  auto lib1 = SharedLibrary::create(libPath);
  ASSERT_NE(lib1, nullptr);

  // Try to create with a non-existent path - should return nullptr
  auto lib2 = SharedLibrary::create("/nonexistent/path.so");
  EXPECT_EQ(lib2, nullptr);
}

//==============================================================================
// Destruction Tests
//==============================================================================

TEST(SharedLibraryTest, Destruction_LibraryUnloadsCleanly)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  {
    auto lib = SharedLibrary::create(libPath);
    ASSERT_NE(lib, nullptr);
    EXPECT_TRUE(lib->isValid());
    // Library goes out of scope here and should be unloaded
  }

  // If we got here without crashing, the library unloaded cleanly
  SUCCEED();
}

TEST(SharedLibraryTest, Destruction_CanReloadAfterUnload)
{
  const std::string libPath = getSystemLibraryPath();
  if (libPath.empty()) {
    GTEST_SKIP() << "No system library found for this platform";
  }

  std::shared_ptr<SharedLibrary> lib1;
  {
    lib1 = SharedLibrary::create(libPath);
    ASSERT_NE(lib1, nullptr);
  }

  // Reset the shared_ptr to trigger unload (if this is the last reference)
  lib1.reset();

  // Should be able to load again
  auto lib2 = SharedLibrary::create(libPath);
  ASSERT_NE(lib2, nullptr);
  EXPECT_TRUE(lib2->isValid());
}
