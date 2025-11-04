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

#include "dart7/common/diagnostics.hpp"

#include <spdlog/fmt/fmt.h>

#include <typeinfo>

// Platform-specific includes for library path detection
#if defined(_WIN32) || defined(_WIN64)
  #define WIN32_LEAN_AND_MEAN
  #include <windows.h>
#elif defined(__linux__) || defined(__APPLE__) || defined(__unix__)
  #include <dlfcn.h>
#endif

namespace dart7::common {

std::string getCompilerInfo()
{
#if defined(__clang__)
  return fmt::format(
      "Clang {}.{}.{}", __clang_major__, __clang_minor__, __clang_patchlevel__);
#elif defined(_MSC_VER)
  return fmt::format("MSVC {}", _MSC_VER);
#elif defined(__GNUC__)
  return fmt::format(
      "GCC {}.{}.{}", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#else
  return "Unknown";
#endif
}

std::string getCxxABI()
{
#ifdef __GLIBCXX__
  return fmt::format("libstdc++ {}", __GLIBCXX__);
#elif defined(_LIBCPP_VERSION)
  return fmt::format("libc++ {}", _LIBCPP_VERSION);
#elif defined(_MSC_VER)
  return "MSVC STL";
#else
  return "unknown";
#endif
}

long getCxxStandard()
{
  return __cplusplus;
}

std::string getLibraryPath(void* func)
{
#if defined(_WIN32) || defined(_WIN64)
  // Windows implementation
  HMODULE hModule = nullptr;
  if (GetModuleHandleExA(
          GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS
              | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
          reinterpret_cast<LPCSTR>(func),
          &hModule)) {
    char path[MAX_PATH];
    if (GetModuleFileNameA(hModule, path, MAX_PATH)) {
      return std::string(path);
    }
  }
  return "unknown";
#elif defined(__linux__) || defined(__APPLE__) || defined(__unix__)
  // Unix/Linux/macOS implementation
  Dl_info info;
  if (dladdr(func, &info) && info.dli_fname) {
    return info.dli_fname;
  }
  return "unknown";
#else
  return "unsupported platform";
#endif
}

void printRuntimeInfo()
{
  fmt::print(
      stderr,
      "\n"
      "╔════════════════════════════════════════════════════════════════╗\n"
      "║ DART7 Runtime Information                                      ║\n"
      "╠════════════════════════════════════════════════════════════════╣\n"
      "║ Compiler:     {:<49}║\n"
      "║ C++ ABI:      {:<49}║\n"
      "║ C++ Standard: C++{:<44}║\n"
      "║ std::string:  {:<49}║\n"
      "╚════════════════════════════════════════════════════════════════╝\n",
      getCompilerInfo(),
      getCxxABI(),
      getCxxStandard(),
      typeid(std::string).name());
}

void printErrorHelp(const std::string& section, const std::string& error)
{
  fmt::print(
      stderr,
      "\n"
      "╔════════════════════════════════════════════════════════════════╗\n"
      "║ ERROR: Initialization failed                                  ║\n"
      "╠════════════════════════════════════════════════════════════════╣\n"
      "║ Failed at:   {:<49}║\n"
      "║ Exception:   {:<49}║\n"
      "║                                                                ║\n"
      "║ Quick fixes to try:                                            ║\n"
      "║ 1. Clean rebuild:                                              ║\n"
      "║    pixi run clean && pixi run build-dartpy7                    ║\n"
      "║                                                                ║\n"
      "║ 2. Check library dependencies:                                 ║\n"
      "║    ldd <path-to-library>                                       ║\n"
      "║                                                                ║\n"
      "║ 3. Verify environment:                                         ║\n"
      "║    pixi list | grep -E '(dart7|nanobind|spdlog)'               ║\n"
      "╚════════════════════════════════════════════════════════════════╝\n",
      section.substr(0, 49),
      error.substr(0, 49));
}

} // namespace dart7::common
