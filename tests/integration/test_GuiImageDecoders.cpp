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

#include <cstdio>

// jpeglib.h requires FILE and size_t to be declared first.
#include <jpeglib.h>
#include <png.h>

#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
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
void writePng(
    const ImageFile& file,
    int colorType,
    int bitDepth,
    int interlace,
    Bytes samples)
{
  const std::unique_ptr<FILE, int (*)(FILE*)> output(
      std::fopen(file.path.string().c_str(), "wb"), &std::fclose);
  ASSERT_TRUE(output);
  auto* png = png_create_write_struct(
      PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  ASSERT_NE(png, nullptr);
  auto* info = png_create_info_struct(png);
  if (!info) {
    png_destroy_write_struct(&png, nullptr);
    FAIL() << "Failed to create PNG encoder info";
  }
  if (setjmp(png_jmpbuf(png))) {
    png_destroy_write_struct(&png, &info);
    FAIL() << "Failed to encode PNG";
  }
  png_init_io(png, output.get());
  png_set_IHDR(
      png,
      info,
      2,
      2,
      bitDepth,
      colorType,
      interlace,
      PNG_COMPRESSION_TYPE_DEFAULT,
      PNG_FILTER_TYPE_DEFAULT);
  if (colorType == PNG_COLOR_TYPE_PALETTE) {
    png_color palette[] = {{255, 0, 0}, {0, 0, 255}};
    png_byte alpha[] = {255, 64};
    png_set_PLTE(png, info, palette, 2);
    png_set_tRNS(png, info, alpha, 2, nullptr);
  }
  png_write_info(png, info);
  png_bytep rows[] = {samples.data(), samples.data() + samples.size() / 2};
  png_write_image(png, rows);
  png_write_end(png, info);
  png_destroy_write_struct(&png, &info);
}
void writeJpeg(
    const ImageFile& file, J_COLOR_SPACE colorSpace, Bytes sample, bool adobe)
{
  const std::unique_ptr<FILE, int (*)(FILE*)> output(
      std::fopen(file.path.string().c_str(), "wb"), &std::fclose);
  ASSERT_TRUE(output);
  jpeg_compress_struct info{};
  jpeg_error_mgr error{};
  info.err = jpeg_std_error(&error);
  jpeg_create_compress(&info);
  jpeg_stdio_dest(&info, output.get());
  info.image_width = 3;
  info.image_height = 2;
  info.input_components = static_cast<int>(sample.size());
  info.in_color_space = colorSpace;
  jpeg_set_defaults(&info);
  jpeg_set_quality(&info, 100, TRUE);
  info.write_Adobe_marker = adobe;
  jpeg_start_compress(&info, TRUE);
  Bytes row;
  for (unsigned x = 0; x < info.image_width; ++x)
    row.insert(row.end(), sample.begin(), sample.end());
  while (info.next_scanline < info.image_height) {
    JSAMPROW rowPtr = row.data();
    ASSERT_EQ(jpeg_write_scanlines(&info, &rowPtr, 1), 1u);
  }
  jpeg_finish_compress(&info);
  jpeg_destroy_compress(&info);
}
} // namespace

TEST(GuiImageDecoders, PngValidColorFormats)
{
  struct Case
  {
    const char* name;
    int colorType;
    int bitDepth;
    int interlace;
    bool hasAlpha;
    Bytes samples;
    Bytes expected;
  };
  const Bytes rgb{255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255};
  const Bytes rgba{
      255, 0, 0, 255, 0, 255, 0, 128, 0, 0, 255, 64, 255, 255, 255, 0};
  const Bytes opaque{
      255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255};
  const Case cases[] = {
      {"RGB", PNG_COLOR_TYPE_RGB, 8, PNG_INTERLACE_NONE, false, rgb, opaque},
      {"RGBA", PNG_COLOR_TYPE_RGBA, 8, PNG_INTERLACE_NONE, true, rgba, rgba},
      {"gray",
       PNG_COLOR_TYPE_GRAY,
       8,
       PNG_INTERLACE_NONE,
       false,
       {0, 64, 128, 255},
       {0, 0, 0, 255, 64, 64, 64, 255, 128, 128, 128, 255, 255, 255, 255, 255}},
      {"palette",
       PNG_COLOR_TYPE_PALETTE,
       8,
       PNG_INTERLACE_NONE,
       true,
       {0, 1, 1, 0},
       {255, 0, 0, 255, 0, 0, 255, 64, 0, 0, 255, 64, 255, 0, 0, 255}},
      // Endpoint samples remain exact through libpng's 16-to-8-bit conversion.
      {"16-bit",
       PNG_COLOR_TYPE_RGB,
       16,
       PNG_INTERLACE_NONE,
       false,
       {255, 255, 0, 0, 0,   0,   0,   0,   255, 255, 0,   0,
        0,   0,   0, 0, 255, 255, 255, 255, 255, 255, 255, 255},
       opaque},
      {"interlaced",
       PNG_COLOR_TYPE_RGB,
       8,
       PNG_INTERLACE_ADAM7,
       false,
       rgb,
       opaque}};
  for (const auto& test : cases) {
    SCOPED_TRACE(test.name);
    const ImageFile file({});
    ASSERT_NO_FATAL_FAILURE(writePng(
        file, test.colorType, test.bitDepth, test.interlace, test.samples));
    const auto image = dart::gui::detail::loadPngImage(file.path);
    ASSERT_TRUE(image);
    EXPECT_EQ(image->width, 2u);
    EXPECT_EQ(image->height, 2u);
    EXPECT_EQ(image->hasAlpha, test.hasAlpha);
    EXPECT_EQ(image->rgba, test.expected);
  }
}

TEST(GuiImageDecoders, JpegValidColorFormats)
{
  struct Case
  {
    const char* name;
    J_COLOR_SPACE colorSpace;
    Bytes sample;
    bool adobe;
    std::array<unsigned, 4> expected;
  };
  const Case cases[]
      = {{"RGB", JCS_RGB, {48, 96, 144}, false, {48, 96, 144, 255}},
         {"gray", JCS_GRAYSCALE, {96}, false, {96, 96, 96, 255}},
         {"CMYK Adobe", JCS_CMYK, {64, 128, 192, 128}, true, {32, 64, 96, 255}},
         {"CMYK", JCS_CMYK, {64, 128, 192, 128}, false, {95, 63, 31, 255}}};
  for (const auto& test : cases) {
    SCOPED_TRACE(test.name);
    const ImageFile file({});
    ASSERT_NO_FATAL_FAILURE(
        writeJpeg(file, test.colorSpace, test.sample, test.adobe));
    const auto image = dart::gui::detail::loadJpegImage(file.path);
    ASSERT_TRUE(image);
    EXPECT_EQ(image->width, 3u);
    EXPECT_EQ(image->height, 2u);
    EXPECT_FALSE(image->hasAlpha);
    ASSERT_EQ(image->rgba.size(), 3u * 2u * 4u);
    for (unsigned y = 0; y < image->height; ++y) {
      for (unsigned x = 0; x < image->width; ++x) {
        const auto actual = pixel(*image, x, y);
        // JPEG is lossy even at quality 100; allow two levels per RGB channel.
        for (unsigned channel = 0; channel < 3; ++channel)
          EXPECT_NEAR(actual[channel], test.expected[channel], 2);
        EXPECT_EQ(actual[3], test.expected[3]);
      }
    }
  }
}

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

TEST(GuiImageDecoders, PngRejectsOversizedHeader)
{
  // IHDR claims 10000 x 10000 RGB pixels; IDAT has no pixel data.
  const ImageFile file({0x89, 'P',  'N',  'G',  0x0d, 0x0a, 0x1a, 0x0a, 0,
                        0,    0,    13,   'I',  'H',  'D',  'R',  0,    0,
                        0x27, 0x10, 0,    0,    0x27, 0x10, 8,    2,    0,
                        0,    0,    0x35, 0x2c, 0xf5, 0x70, 0,    0,    0,
                        0,    'I',  'D',  'A',  'T',  0x35, 0xaf, 0x06, 0x1e});
  testing::internal::CaptureStderr();
  const auto image = dart::gui::detail::loadPngImage(file.path);
  const auto errors = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(image);
  EXPECT_NE(
      errors.find("PNG texture exceeds decode budget"), std::string::npos);
}

TEST(GuiImageDecoders, JpegRejectsOversizedHeader)
{
  // SOF0 claims 65500 x 65500 grayscale pixels; SOS has no pixel data.
  const ImageFile file({0xff, 0xd8, 0xff, 0xc0, 0,    11, 8,    0xff, 0xdc,
                        0xff, 0xdc, 1,    1,    0x11, 0,  0xff, 0xda, 0,
                        8,    1,    1,    0,    0,    63, 0,    0xff, 0xd9});
  testing::internal::CaptureStderr();
  const auto image = dart::gui::detail::loadJpegImage(file.path);
  const auto errors = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(image);
  EXPECT_NE(
      errors.find("JPEG texture exceeds decode budget"), std::string::npos);
}
