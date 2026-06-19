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

#include "dart/gui/detail/PngWriter.hpp"

#include <gtest/gtest.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <cstdint>
#include <cstdio>

namespace {

std::uint32_t readUint32Big(
    const std::vector<unsigned char>& bytes, std::size_t offset)
{
  return (static_cast<std::uint32_t>(bytes[offset]) << 24)
         | (static_cast<std::uint32_t>(bytes[offset + 1]) << 16)
         | (static_cast<std::uint32_t>(bytes[offset + 2]) << 8)
         | static_cast<std::uint32_t>(bytes[offset + 3]);
}

std::uint16_t readUint16Little(
    const std::vector<unsigned char>& bytes, std::size_t offset)
{
  return static_cast<std::uint16_t>(bytes[offset])
         | static_cast<std::uint16_t>(bytes[offset + 1] << 8);
}

std::map<std::string, std::vector<unsigned char>> readChunks(
    const std::vector<unsigned char>& png)
{
  std::map<std::string, std::vector<unsigned char>> chunks;
  std::size_t offset = 8;
  while (offset + 12 <= png.size()) {
    const std::uint32_t length = readUint32Big(png, offset);
    offset += 4;
    const std::string type(png.begin() + offset, png.begin() + offset + 4);
    offset += 4;
    chunks[type] = std::vector<unsigned char>(
        png.begin() + offset, png.begin() + offset + length);
    offset += length + 4;
    if (type == "IEND")
      break;
  }

  return chunks;
}

std::vector<unsigned char> inflateStoredZlib(
    const std::vector<unsigned char>& zlib)
{
  std::vector<unsigned char> inflated;
  if (zlib.size() < 6) {
    ADD_FAILURE() << "zlib stream is too short";
    return inflated;
  }

  EXPECT_EQ(0x78, zlib[0]);
  EXPECT_EQ(0x01, zlib[1]);

  std::size_t offset = 2;
  const std::size_t adlerOffset = zlib.size() - 4;
  bool sawFinalBlock = false;
  while (!sawFinalBlock && offset + 5 <= adlerOffset) {
    const unsigned char blockHeader = zlib[offset++];
    sawFinalBlock = (blockHeader & 0x01) != 0;
    EXPECT_EQ(0, blockHeader & 0xfe);

    const std::uint16_t length = readUint16Little(zlib, offset);
    offset += 2;
    const std::uint16_t inverseLength = readUint16Little(zlib, offset);
    offset += 2;
    EXPECT_EQ(static_cast<std::uint16_t>(~length), inverseLength);

    if (offset + length > adlerOffset) {
      ADD_FAILURE() << "stored deflate block overruns IDAT payload";
      return inflated;
    }

    inflated.insert(
        inflated.end(), zlib.begin() + offset, zlib.begin() + offset + length);
    offset += length;
  }

  EXPECT_TRUE(sawFinalBlock);
  EXPECT_EQ(adlerOffset, offset);
  return inflated;
}

} // namespace

TEST(PngWriter, EncodesRgbaPngWithStoredZlibRows)
{
  const std::vector<unsigned char> rgba = {
      255,
      0,
      0,
      255, // red
      0,
      255,
      0,
      128, // green
      0,
      0,
      255,
      64, // blue
      255,
      255,
      255,
      0 // transparent white
  };

  std::vector<unsigned char> png;
  std::string errorMessage;
  ASSERT_TRUE(
      dart::gui::detail::encodeRgbaPng(rgba, 2, 2, &png, &errorMessage));
  EXPECT_TRUE(errorMessage.empty());

  const std::vector<unsigned char> signature
      = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
  ASSERT_GE(png.size(), signature.size());
  EXPECT_EQ(
      signature,
      std::vector<unsigned char>(png.begin(), png.begin() + signature.size()));

  const auto chunks = readChunks(png);
  ASSERT_TRUE(chunks.count("IHDR"));
  ASSERT_TRUE(chunks.count("IDAT"));
  ASSERT_TRUE(chunks.count("IEND"));

  const auto& ihdr = chunks.at("IHDR");
  ASSERT_EQ(13u, ihdr.size());
  EXPECT_EQ(2u, readUint32Big(ihdr, 0));
  EXPECT_EQ(2u, readUint32Big(ihdr, 4));
  EXPECT_EQ(8, ihdr[8]);
  EXPECT_EQ(6, ihdr[9]);
  EXPECT_EQ(0, ihdr[10]);
  EXPECT_EQ(0, ihdr[11]);
  EXPECT_EQ(0, ihdr[12]);

  const std::vector<unsigned char> expectedScanlines
      = {0, 255, 0, 0, 255, 0, 255, 0, 128, 0, 0, 0, 255, 64, 255, 255, 255, 0};
  EXPECT_EQ(expectedScanlines, inflateStoredZlib(chunks.at("IDAT")));
}

TEST(PngWriter, RejectsInvalidRgbaInput)
{
  std::vector<unsigned char> png;
  std::string errorMessage;

  EXPECT_FALSE(dart::gui::detail::encodeRgbaPng({}, 0, 1, &png, &errorMessage));
  EXPECT_FALSE(errorMessage.empty());

  EXPECT_FALSE(
      dart::gui::detail::encodeRgbaPng({1, 2, 3}, 1, 1, &png, &errorMessage));
  EXPECT_FALSE(errorMessage.empty());

  EXPECT_FALSE(dart::gui::detail::encodeRgbaPng(
      std::vector<unsigned char>(4), 1, 1, nullptr, &errorMessage));
  EXPECT_FALSE(errorMessage.empty());
}

TEST(PngWriter, WritesPngFile)
{
  const std::string fileName = "UNIT_gui_PngWriter_test.png";
  std::remove(fileName.c_str());

  const std::vector<unsigned char> rgba = {1, 2, 3, 255};
  std::string errorMessage;
  ASSERT_TRUE(
      dart::gui::detail::writeRgbaPng(fileName, rgba, 1, 1, &errorMessage));
  EXPECT_TRUE(errorMessage.empty());

  std::ifstream input(fileName, std::ios::binary);
  ASSERT_TRUE(input);

  std::vector<unsigned char> png{
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  ASSERT_GE(png.size(), 8u);
  EXPECT_EQ(0x89, png[0]);
  EXPECT_EQ('P', png[1]);
  EXPECT_EQ('N', png[2]);
  EXPECT_EQ('G', png[3]);

  std::remove(fileName.c_str());
}
