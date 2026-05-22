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

#include <imgui.h>

#include <gtest/gtest.h>

namespace {

class ScopedImGuiContextRestore
{
public:
  ScopedImGuiContextRestore() : mPreviousContext(ImGui::GetCurrentContext())
  {
    // Do nothing
  }

  ~ScopedImGuiContextRestore()
  {
    ImGui::SetCurrentContext(mPreviousContext);
  }

private:
  ImGuiContext* mPreviousContext;
};

} // namespace

TEST(Issue2668, ColorPicker3ReturnsFalseWithoutCurrentContext)
{
  ScopedImGuiContextRestore restore;
  ImGui::SetCurrentContext(nullptr);

  float color[3] = {1.0f, 0.0f, 0.0f};

  EXPECT_FALSE(ImGui::ColorPicker3("Fuzz Color Picker", color, 0));
  EXPECT_FLOAT_EQ(color[0], 1.0f);
  EXPECT_FLOAT_EQ(color[1], 0.0f);
  EXPECT_FLOAT_EQ(color[2], 0.0f);
}

TEST(Issue2668, ColorPicker3ReturnsFalseWithoutCurrentWindow)
{
  ScopedImGuiContextRestore restore;
  ImGuiContext* context = ImGui::CreateContext();
  ImGui::SetCurrentContext(context);

  float color[3] = {1.0f, 0.0f, 0.0f};

  EXPECT_FALSE(ImGui::ColorPicker3("Fuzz Color Picker", color, 0));
  EXPECT_FLOAT_EQ(color[0], 1.0f);
  EXPECT_FLOAT_EQ(color[1], 0.0f);
  EXPECT_FLOAT_EQ(color[2], 0.0f);

  ImGui::DestroyContext(context);
}
