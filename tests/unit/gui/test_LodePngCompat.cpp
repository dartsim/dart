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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER AND
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/external/lodepng/lodepng.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <cstdio>

namespace {

constexpr std::array<unsigned char, 8> kPngSignature
    = {{0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'}};

void expectPngSignature(const std::vector<unsigned char>& png)
{
  ASSERT_GE(png.size(), kPngSignature.size());
  EXPECT_TRUE(
      std::equal(kPngSignature.begin(), kPngSignature.end(), png.begin()));
}

} // namespace

TEST(LodePngCompat, EncodesRgbaVectorThroughLegacyHeader)
{
  const std::vector<unsigned char> rgba = {255, 0, 0, 255};

  std::vector<unsigned char> png;
  EXPECT_EQ(0u, lodepng::encode(png, rgba, 1, 1));

  expectPngSignature(png);
  EXPECT_STREQ("No error", lodepng_error_text(0));
}

TEST(LodePngCompat, EncodesRgbMemoryThroughLegacyHeader)
{
  const std::array<unsigned char, 3> rgb = {{0, 255, 0}};

  unsigned char* png = nullptr;
  size_t pngSize = 0;
  ASSERT_EQ(
      0u, lodepng_encode_memory(&png, &pngSize, rgb.data(), 1, 1, LCT_RGB, 8));

  const std::vector<unsigned char> pngVector(png, png + pngSize);
  lodepng_free(png);
  expectPngSignature(pngVector);
}

TEST(LodePngCompat, EncodesRgb24HelperThroughLegacyHeader)
{
  const std::array<unsigned char, 3> rgb = {{0, 255, 0}};

  unsigned char* png = nullptr;
  size_t pngSize = 0;
  ASSERT_EQ(0u, lodepng_encode24(&png, &pngSize, rgb.data(), 1, 1));

  const std::vector<unsigned char> pngVector(png, png + pngSize);
  lodepng_free(png);
  expectPngSignature(pngVector);
}

TEST(LodePngCompat, EncodesLegacyRawColorModes)
{
  const std::array<unsigned char, 1> grey1 = {{0x80}};
  const std::array<unsigned char, 4> greyAlpha16 = {{0x20, 0, 0xa0, 0}};
  const std::array<unsigned char, 8> rgba16
      = {{0x10, 0, 0x20, 0, 0x30, 0, 0xff, 0}};

  unsigned char* png = nullptr;
  size_t pngSize = 0;
  ASSERT_EQ(
      0u,
      lodepng_encode_memory(&png, &pngSize, grey1.data(), 1, 1, LCT_GREY, 1));
  std::vector<unsigned char> pngVector(png, png + pngSize);
  lodepng_free(png);
  expectPngSignature(pngVector);

  png = nullptr;
  pngSize = 0;
  ASSERT_EQ(
      0u,
      lodepng_encode_memory(
          &png, &pngSize, greyAlpha16.data(), 1, 1, LCT_GREY_ALPHA, 16));
  pngVector.assign(png, png + pngSize);
  lodepng_free(png);
  expectPngSignature(pngVector);

  png = nullptr;
  pngSize = 0;
  ASSERT_EQ(
      0u,
      lodepng_encode_memory(&png, &pngSize, rgba16.data(), 1, 1, LCT_RGBA, 16));
  pngVector.assign(png, png + pngSize);
  lodepng_free(png);
  expectPngSignature(pngVector);
}

TEST(LodePngCompat, RejectsPaletteModeWithoutPaletteState)
{
  const std::array<unsigned char, 1> paletteIndex = {{0}};

  unsigned char* png = nullptr;
  size_t pngSize = 0;
  EXPECT_EQ(
      68u,
      lodepng_encode_memory(
          &png, &pngSize, paletteIndex.data(), 1, 1, LCT_PALETTE, 8));
  EXPECT_EQ(nullptr, png);
  EXPECT_EQ(0u, pngSize);
  EXPECT_STREQ(
      "Palette encoding requires explicit palette state",
      lodepng_error_text(68));
}

TEST(LodePngCompat, WritesFileThroughLegacyHeader)
{
  const std::string fileName = "UNIT_gui_LodePngCompat_test.png";
  const std::vector<unsigned char> rgba = {0, 0, 255, 255};

  ASSERT_EQ(0u, lodepng::encode(fileName, rgba, 1, 1));

  std::ifstream input(fileName, std::ios::binary);
  ASSERT_TRUE(input);
  const std::vector<unsigned char> png{
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  input.close();
  std::remove(fileName.c_str());

  expectPngSignature(png);
}
