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

#include "dart/gui/detail/PngWriter.hpp"

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>

#include <cstdint>

namespace dart {
namespace gui {
namespace detail {
namespace {

constexpr std::array<unsigned char, 8> kPngSignature
    = {{0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'}};
constexpr std::size_t kMaxDeflateBlockSize = 65535;
constexpr std::uint32_t kAdlerModulus = 65521;

void setError(std::string* errorMessage, const std::string& message)
{
  if (errorMessage)
    *errorMessage = message;
}

void appendUint16Little(std::vector<unsigned char>& output, std::uint16_t value)
{
  output.push_back(static_cast<unsigned char>(value & 0xff));
  output.push_back(static_cast<unsigned char>((value >> 8) & 0xff));
}

void appendUint32Big(std::vector<unsigned char>& output, std::uint32_t value)
{
  output.push_back(static_cast<unsigned char>((value >> 24) & 0xff));
  output.push_back(static_cast<unsigned char>((value >> 16) & 0xff));
  output.push_back(static_cast<unsigned char>((value >> 8) & 0xff));
  output.push_back(static_cast<unsigned char>(value & 0xff));
}

std::uint32_t crc32(const unsigned char* data, std::size_t size)
{
  std::uint32_t crc = 0xffffffffu;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = 0u - (crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }

  return crc ^ 0xffffffffu;
}

std::uint32_t adler32(const std::vector<unsigned char>& data)
{
  std::uint32_t a = 1;
  std::uint32_t b = 0;

  for (const unsigned char byte : data) {
    a = (a + byte) % kAdlerModulus;
    b = (b + a) % kAdlerModulus;
  }

  return (b << 16) | a;
}

void appendChunk(
    std::vector<unsigned char>& png,
    const std::array<unsigned char, 4>& type,
    const std::vector<unsigned char>& data)
{
  appendUint32Big(png, static_cast<std::uint32_t>(data.size()));

  const std::size_t crcStart = png.size();
  png.insert(png.end(), type.begin(), type.end());
  png.insert(png.end(), data.begin(), data.end());

  appendUint32Big(png, crc32(&png[crcStart], type.size() + data.size()));
}

std::vector<unsigned char> createScanlines(
    const std::vector<unsigned char>& rgba, unsigned width, unsigned height)
{
  const std::size_t rowBytes = static_cast<std::size_t>(width) * 4;
  std::vector<unsigned char> scanlines;
  scanlines.reserve((rowBytes + 1) * static_cast<std::size_t>(height));

  for (unsigned row = 0; row < height; ++row) {
    scanlines.push_back(0);
    const auto rowStart = rgba.begin()
                          + static_cast<std::ptrdiff_t>(
                              static_cast<std::size_t>(row) * rowBytes);
    scanlines.insert(scanlines.end(), rowStart, rowStart + rowBytes);
  }

  return scanlines;
}

std::vector<unsigned char> createStoredZlibStream(
    const std::vector<unsigned char>& data)
{
  std::vector<unsigned char> stream;
  stream.reserve(data.size() + data.size() / kMaxDeflateBlockSize * 5 + 11);

  stream.push_back(0x78);
  stream.push_back(0x01);

  std::size_t offset = 0;
  do {
    const std::size_t blockSize
        = std::min(kMaxDeflateBlockSize, data.size() - offset);
    const bool isFinalBlock = offset + blockSize == data.size();
    stream.push_back(isFinalBlock ? 0x01 : 0x00);

    const auto length = static_cast<std::uint16_t>(blockSize);
    appendUint16Little(stream, length);
    appendUint16Little(stream, static_cast<std::uint16_t>(~length));

    stream.insert(
        stream.end(),
        data.begin() + static_cast<std::ptrdiff_t>(offset),
        data.begin() + static_cast<std::ptrdiff_t>(offset + blockSize));
    offset += blockSize;
  } while (offset < data.size());

  appendUint32Big(stream, adler32(data));

  return stream;
}

bool validateInput(
    const std::vector<unsigned char>& rgba,
    unsigned width,
    unsigned height,
    std::string* errorMessage)
{
  if (width == 0 || height == 0) {
    setError(errorMessage, "PNG dimensions must be non-zero");
    return false;
  }

  const std::size_t max = std::numeric_limits<std::size_t>::max();
  const std::size_t widthSize = width;
  const std::size_t heightSize = height;

  if (widthSize > max / 4 || heightSize > max / (widthSize * 4)) {
    setError(errorMessage, "PNG dimensions overflow the RGBA buffer size");
    return false;
  }

  const std::size_t expectedSize = widthSize * heightSize * 4;
  if (rgba.size() != expectedSize) {
    setError(errorMessage, "RGBA buffer size does not match PNG dimensions");
    return false;
  }

  if (heightSize > max / (widthSize * 4 + 1)) {
    setError(errorMessage, "PNG dimensions overflow the scanline buffer size");
    return false;
  }

  return true;
}

} // namespace

bool encodeRgbaPng(
    const std::vector<unsigned char>& rgba,
    unsigned width,
    unsigned height,
    std::vector<unsigned char>* png,
    std::string* errorMessage)
{
  if (!png) {
    setError(errorMessage, "PNG output buffer is null");
    return false;
  }

  png->clear();

  if (!validateInput(rgba, width, height, errorMessage))
    return false;

  png->insert(png->end(), kPngSignature.begin(), kPngSignature.end());

  std::vector<unsigned char> ihdr;
  ihdr.reserve(13);
  appendUint32Big(ihdr, width);
  appendUint32Big(ihdr, height);
  ihdr.push_back(8);
  ihdr.push_back(6);
  ihdr.push_back(0);
  ihdr.push_back(0);
  ihdr.push_back(0);
  appendChunk(*png, {{'I', 'H', 'D', 'R'}}, ihdr);

  const std::vector<unsigned char> scanlines
      = createScanlines(rgba, width, height);
  appendChunk(*png, {{'I', 'D', 'A', 'T'}}, createStoredZlibStream(scanlines));
  appendChunk(*png, {{'I', 'E', 'N', 'D'}}, {});

  if (errorMessage)
    errorMessage->clear();

  return true;
}

bool writeRgbaPng(
    const std::string& fileName,
    const std::vector<unsigned char>& rgba,
    unsigned width,
    unsigned height,
    std::string* errorMessage)
{
  std::vector<unsigned char> png;
  if (!encodeRgbaPng(rgba, width, height, &png, errorMessage))
    return false;

  std::ofstream output(fileName, std::ios::binary);
  if (!output) {
    setError(errorMessage, "Failed to open PNG file for writing: " + fileName);
    return false;
  }

  output.write(
      reinterpret_cast<const char*>(png.data()),
      static_cast<std::streamsize>(png.size()));
  if (!output) {
    setError(errorMessage, "Failed to write PNG file: " + fileName);
    return false;
  }

  if (errorMessage)
    errorMessage->clear();

  return true;
}

} // namespace detail
} // namespace gui
} // namespace dart
