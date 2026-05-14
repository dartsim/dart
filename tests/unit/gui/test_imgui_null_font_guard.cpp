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

#include <dart/gui/include_im_gui.hpp>

#include <gtest/gtest.h>

TEST(Issue2516, AddFontFromMemoryCompressedTTF_NullData)
{
  ImFontAtlas atlas;

  ImFont* font = atlas.AddFontFromMemoryCompressedTTF(
      nullptr, 0, 16.0f, nullptr, nullptr);

  EXPECT_EQ(font, nullptr);
}

TEST(Issue2516, AddFontFromMemoryTTF_NullData)
{
  ImFontAtlas atlas;

  ImFont* font
      = atlas.AddFontFromMemoryTTF(nullptr, 0, 16.0f, nullptr, nullptr);

  EXPECT_EQ(font, nullptr);
}

TEST(Issue2516, AddFontFromMemoryCompressedBase85TTF_NullData)
{
  ImFontAtlas atlas;

  ImFont* font = atlas.AddFontFromMemoryCompressedBase85TTF(
      nullptr, 16.0f, nullptr, nullptr);

  EXPECT_EQ(font, nullptr);
}
