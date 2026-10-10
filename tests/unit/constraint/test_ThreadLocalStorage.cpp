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

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <string_view>

#if defined(__linux__)
  #include <link.h>
#endif

namespace {

#if defined(__linux__)
struct TlsSizes
{
  std::optional<std::size_t> executable;
  std::optional<std::size_t> dart;
};

std::size_t getTlsSize(const dl_phdr_info& info)
{
  for (ElfW(Half) i = 0; i < info.dlpi_phnum; ++i) {
    if (info.dlpi_phdr[i].p_type == PT_TLS)
      return info.dlpi_phdr[i].p_memsz;
  }
  return 0u;
}

int collectTlsSizes(dl_phdr_info* info, std::size_t, void* data)
{
  auto& sizes = *static_cast<TlsSizes*>(data);
  std::string_view name = info->dlpi_name;
  if (const auto slash = name.rfind('/'); slash != std::string_view::npos)
    name.remove_prefix(slash + 1u);

  // The main program comes first. A shared libdart is loaded by its soname,
  // which extends the linker file name with the version.
  constexpr std::string_view dartFileName = DART_LIBRARY_FILE_NAME;
  if (!sizes.executable)
    sizes.executable = getTlsSize(*info);
  else if (name.substr(0u, dartFileName.size()) == dartFileName)
    sizes.dart = getTlsSize(*info);
  return 0;
}
#endif

} // namespace

//==============================================================================
TEST(ThreadLocalStorage, DartKeepsPerThreadStorageSmall)
{
#if defined(__linux__)
  // Every thread of a process that links DART carries DART's thread-local
  // storage, so large per-thread caches belong on the heap.
  const dart::constraint::BoxedLcpConstraintSolver solver;
  TlsSizes sizes;
  dl_iterate_phdr(&collectTlsSizes, &sizes);
  ASSERT_TRUE(sizes.executable.has_value());

  // Static builds link DART into the test executable.
  EXPECT_LT(sizes.dart.value_or(*sizes.executable), 64u * 1024u);
#else
  GTEST_SKIP() << "Reads ELF program headers";
#endif
}
