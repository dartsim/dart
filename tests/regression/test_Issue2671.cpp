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

#include <dart/external/imgui/imgui.h>

#include <gtest/gtest.h>

#if defined(__linux__)
  #include <dlfcn.h>
#endif

//==============================================================================
TEST(Issue2671, LegacyImguiPublicApiRemainsAvailable)
{
  ASSERT_NE(nullptr, ImGui::GetVersion());
}

#if defined(__linux__) && defined(DART_EXTERNAL_IMGUI_HAS_DART_PATCHES)
//==============================================================================
TEST(Issue2671, InternalFormattingHelpersAreNotExported)
{
  Dl_info info;
  ASSERT_NE(
      0, dladdr(reinterpret_cast<const void*>(&ImGui::GetVersion), &info));
  ASSERT_NE(nullptr, info.dli_fname);

  void* handle = dlopen(info.dli_fname, RTLD_LAZY | RTLD_NOLOAD);
  if (handle == nullptr) {
    handle = dlopen(info.dli_fname, RTLD_LAZY);
  }

  ASSERT_NE(nullptr, handle) << dlerror();

  dlerror();
  EXPECT_EQ(nullptr, dlsym(handle, "_Z14ImFormatStringPcmPKcz"));
}
#endif
