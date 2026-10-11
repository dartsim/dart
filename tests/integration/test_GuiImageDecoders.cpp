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

#include <dart/gui/detail/image_decoders.hpp>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <string>

namespace {
using Bytes = std::vector<std::uint8_t>;
using dart::gui::detail::ImageData;
struct ImageFile
{
  std::filesystem::path path;
  explicit ImageFile(const Bytes& bytes)
  {
    static std::atomic<unsigned> sequence{0};
    const auto now
        = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path()
           / ("dart-image-decoder-" + std::to_string(now) + "-"
              + std::to_string(sequence++) + ".data");
    std::ofstream file(path, std::ios::binary);
    file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  }
  ~ImageFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};
void little(
    Bytes& bytes, std::size_t offset, std::uint32_t value, unsigned size)
{
  for (unsigned i = 0; i < size; ++i)
    bytes[offset + i] = static_cast<std::uint8_t>(value >> (8 * i));
}
Bytes bmp(std::int32_t height, unsigned bits)
{
  Bytes bytes(54 + 2 * 4, 0);
  bytes[0] = 'B';
  bytes[1] = 'M';
  little(bytes, 10, 54, 4);
  little(bytes, 14, 40, 4);
  little(bytes, 18, 1, 4);
  little(bytes, 22, static_cast<std::uint32_t>(height), 4);
  little(bytes, 26, 1, 2);
  little(bytes, 28, bits, 2);
  // The two disk rows are red then blue, with zero reserved alpha bytes.
  bytes[56] = 255;
  bytes[58] = 255;
  return bytes;
}
Bytes tga(unsigned descriptor, bool rle)
{
  Bytes bytes(18, 0);
  bytes[2] = rle ? 10 : 2;
  bytes[12] = bytes[14] = 2;
  bytes[16] = 24;
  bytes[17] = descriptor;
  if (rle)
    bytes.push_back(3); // Four raw pixels in one RLE stream packet.
  bytes.insert(bytes.end(), {0, 0, 255, 0, 255, 0, 255, 0, 0, 0, 255, 255});
  return bytes;
}
Bytes ascii(const std::string& text)
{
  return {text.begin(), text.end()};
}
std::array<unsigned, 4> pixel(const ImageData& image, unsigned x, unsigned y)
{
  const auto i = (y * image.width + x) * 4;
  return {
      image.rgba[i], image.rgba[i + 1], image.rgba[i + 2], image.rgba[i + 3]};
}
} // namespace

TEST(GuiImageDecoders, BmpRowOriginsAndReservedAlpha)
{
  for (const auto height : {2, -2}) {
    for (const auto bits : {24u, 32u}) {
      const ImageFile file(bmp(height, bits));
      const auto image = dart::gui::detail::loadBmpImage(file.path);
      ASSERT_TRUE(image);
      EXPECT_EQ(image->width, 1u);
      EXPECT_EQ(image->height, 2u);
      const std::array<unsigned, 4> red{255, 0, 0, 255}, blue{0, 0, 255, 255};
      EXPECT_EQ(pixel(*image, 0, 0), height > 0 ? blue : red);
      EXPECT_EQ(pixel(*image, 0, 1), height > 0 ? red : blue);
    }
  }
}

TEST(GuiImageDecoders, BmpRejectsTruncationAndInvalidHeaders)
{
  auto bytes = bmp(2, 24);
  bytes.pop_back();
  EXPECT_FALSE(dart::gui::detail::loadBmpImage(ImageFile(bytes).path));
  bytes = bmp(2, 24);
  little(bytes, 22, 0x80000000u, 4);
  EXPECT_FALSE(dart::gui::detail::loadBmpImage(ImageFile(bytes).path));
  bytes = bmp(2, 24);
  little(bytes, 10, 14, 4);
  EXPECT_FALSE(dart::gui::detail::loadBmpImage(ImageFile(bytes).path));
}

TEST(GuiImageDecoders, TgaRawAndRleOrigins)
{
  const std::array<unsigned, 4> red{255, 0, 0, 255}, green{0, 255, 0, 255};
  const std::array<unsigned, 4> blue{0, 0, 255, 255}, yellow{255, 255, 0, 255};
  for (const auto rle : {false, true}) {
    for (const auto origin : {0u, 0x10u, 0x20u, 0x30u}) {
      const ImageFile file(tga(origin, rle));
      const auto image = dart::gui::detail::loadTgaImage(file.path);
      ASSERT_TRUE(image);
      const auto left = (origin & 0x10) ? 1u : 0u;
      const auto top = (origin & 0x20) ? 0u : 1u;
      EXPECT_EQ(pixel(*image, left, top), red);
      EXPECT_EQ(pixel(*image, 1 - left, top), green);
      EXPECT_EQ(pixel(*image, left, 1 - top), blue);
      EXPECT_EQ(pixel(*image, 1 - left, 1 - top), yellow);
    }
  }
}

TEST(GuiImageDecoders, TgaRepeatedPacketsAndMalformedStreams)
{
  auto bytes = tga(0x20, true);
  bytes.resize(18);
  bytes.insert(bytes.end(), {0x83, 0, 0, 255});
  auto image = dart::gui::detail::loadTgaImage(ImageFile(bytes).path);
  ASSERT_TRUE(image);
  for (unsigned y = 0; y < 2; ++y)
    for (unsigned x = 0; x < 2; ++x)
      EXPECT_EQ(pixel(*image, x, y), (std::array<unsigned, 4>{255, 0, 0, 255}));
  bytes[18] = 0x84; // Five pixels do not fit the four-pixel image.
  EXPECT_FALSE(dart::gui::detail::loadTgaImage(ImageFile(bytes).path));
  bytes.pop_back();
  EXPECT_FALSE(dart::gui::detail::loadTgaImage(ImageFile(bytes).path));
  bytes = tga(0x20, false);
  bytes.resize(18);
  bytes[0] = 250;
  EXPECT_FALSE(dart::gui::detail::loadTgaImage(ImageFile(bytes).path));
}

TEST(GuiImageDecoders, PpmCommentsSixteenBitAndBinaryWhitespace)
{
  const auto p3
      = ascii("P3\n# Header comment\n1 1\n100\n100 # sample comment\n50 0");
  const auto image = dart::gui::detail::loadPpmImage(ImageFile(p3).path);
  ASSERT_TRUE(image);
  EXPECT_EQ(pixel(*image, 0, 0), (std::array<unsigned, 4>{255, 128, 0, 255}));
  auto p6 = ascii("P6\r\n1 1\r\n65535\r\n");
  p6.insert(p6.end(), {255, 255, 128, 0, 0, 0});
  const auto wide = dart::gui::detail::loadPpmImage(ImageFile(p6).path);
  ASSERT_TRUE(wide);
  EXPECT_EQ(pixel(*wide, 0, 0), (std::array<unsigned, 4>{255, 128, 0, 255}));
  p6 = ascii("P6\n1 1\n255\n");
  p6.insert(p6.end(), {' ', '#', '\n'});
  const auto whitespace = dart::gui::detail::loadPpmImage(ImageFile(p6).path);
  ASSERT_TRUE(whitespace);
  EXPECT_EQ(
      pixel(*whitespace, 0, 0), (std::array<unsigned, 4>{32, 35, 10, 255}));
}

TEST(GuiImageDecoders, PpmRejectsTruncationOverflowAndOutOfRangeSamples)
{
  for (const auto& text :
       {"P6\n1 1\n255\nxy",
        "P3\n1 1\n10\n11 0 0",
        "P3\n42949672960 1\n255\n0 0 0",
        "P6\n1 1\n65536\n123456",
        "P3\n8192 8192\n255\n"}) {
    EXPECT_FALSE(dart::gui::detail::loadPpmImage(ImageFile(ascii(text)).path));
  }
}
