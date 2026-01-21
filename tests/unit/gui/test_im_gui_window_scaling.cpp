/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/include_im_gui.hpp>

#include <gtest/gtest.h>

namespace {

struct TestScalingWidget
{
  static std::pair<ImVec2, ImVec2> computePlacement(
      float fontSize,
      ImVec2 basePos = ImVec2(10.0f, 20.0f),
      ImVec2 baseSize = ImVec2(360.0f, 400.0f),
      float defaultFontSize = 13.0f)
  {
    const float fontScale = fontSize / defaultFontSize;
    return {
        ImVec2(basePos.x * fontScale, basePos.y * fontScale),
        ImVec2(baseSize.x * fontScale, baseSize.y * fontScale)};
  }
};

} // namespace

TEST(ImGuiWindowScalingTest, ScalesWithFontSize)
{
  const auto [posDefault, sizeDefault]
      = TestScalingWidget::computePlacement(13.0f);
  EXPECT_FLOAT_EQ(posDefault.x, 10.0f);
  EXPECT_FLOAT_EQ(posDefault.y, 20.0f);
  EXPECT_FLOAT_EQ(sizeDefault.x, 360.0f);
  EXPECT_FLOAT_EQ(sizeDefault.y, 400.0f);

  const auto [posScaled, sizeScaled]
      = TestScalingWidget::computePlacement(26.0f);
  EXPECT_FLOAT_EQ(posScaled.x, 20.0f);
  EXPECT_FLOAT_EQ(posScaled.y, 40.0f);
  EXPECT_FLOAT_EQ(sizeScaled.x, 720.0f);
  EXPECT_FLOAT_EQ(sizeScaled.y, 800.0f);
}
