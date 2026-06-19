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

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

const char* LODEPNG_VERSION_STRING = "DART native PNG compatibility";

namespace {

constexpr unsigned kErrorInvalidInput = 48;
constexpr unsigned kErrorInvalidColorType = 31;
constexpr unsigned kErrorInvalidBitDepth = 37;
constexpr unsigned kErrorInvalidPalette = 68;
constexpr unsigned kErrorFileOpen = 79;
constexpr unsigned kErrorUnsupportedColor = 82;
constexpr unsigned kErrorAllocation = 83;
constexpr unsigned kErrorFileWrite = 84;
constexpr unsigned kErrorOverflow = 92;

constexpr std::array<unsigned char, 8> kPngSignature
    = {{0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'}};
constexpr std::size_t kMaxDeflateBlockSize = 65535;
constexpr std::uint32_t kAdlerModulus = 65521;

void clearOutput(unsigned char** out, size_t* outsize)
{
  if (out)
    *out = nullptr;
  if (outsize)
    *outsize = 0;
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

bool checkedPixelBytes(
    unsigned width, unsigned height, std::size_t channels, std::size_t* bytes)
{
  const std::size_t max = std::numeric_limits<std::size_t>::max();
  const std::size_t widthSize = width;
  const std::size_t heightSize = height;

  if (width == 0 || height == 0 || channels == 0)
    return false;
  if (widthSize > max / channels)
    return false;
  if (heightSize > max / (widthSize * channels))
    return false;

  *bytes = widthSize * heightSize * channels;
  return true;
}

unsigned bitsPerPixel(LodePNGColorType colorType, unsigned bitDepth)
{
  switch (colorType) {
    case LCT_GREY:
      return bitDepth;
    case LCT_RGB:
      return 3 * bitDepth;
    case LCT_PALETTE:
      return bitDepth;
    case LCT_GREY_ALPHA:
      return 2 * bitDepth;
    case LCT_RGBA:
      return 4 * bitDepth;
    default:
      return 0;
  }
}

unsigned validateColorMode(LodePNGColorType colorType, unsigned bitDepth)
{
  switch (colorType) {
    case LCT_GREY:
      if (bitDepth == 1 || bitDepth == 2 || bitDepth == 4 || bitDepth == 8
          || bitDepth == 16) {
        return 0;
      }
      return kErrorInvalidBitDepth;
    case LCT_RGB:
      if (bitDepth == 8 || bitDepth == 16)
        return 0;
      return kErrorInvalidBitDepth;
    case LCT_PALETTE:
      if (bitDepth == 1 || bitDepth == 2 || bitDepth == 4 || bitDepth == 8)
        return kErrorInvalidPalette;
      return kErrorInvalidBitDepth;
    case LCT_GREY_ALPHA:
    case LCT_RGBA:
      if (bitDepth == 8 || bitDepth == 16)
        return 0;
      return kErrorInvalidBitDepth;
    default:
      return kErrorInvalidColorType;
  }
}

bool checkedRawBytes(
    unsigned width,
    unsigned height,
    LodePNGColorType colorType,
    unsigned bitDepth,
    std::size_t* bytes)
{
  const unsigned bpp = bitsPerPixel(colorType, bitDepth);
  if (bpp == 0 || width == 0 || height == 0)
    return false;

  const std::size_t max = std::numeric_limits<std::size_t>::max();
  const std::size_t widthSize = width;
  const std::size_t heightSize = height;
  if (widthSize > max / heightSize)
    return false;

  const std::size_t pixels = widthSize * heightSize;
  if (pixels > (max - 7) / bpp)
    return false;

  *bytes = (pixels * bpp + 7) / 8;
  return true;
}

unsigned char expandSample(unsigned value, unsigned bitDepth)
{
  if (bitDepth == 8 || bitDepth == 16)
    return static_cast<unsigned char>(value);

  const unsigned maxValue = (1u << bitDepth) - 1u;
  return static_cast<unsigned char>((value * 255u + maxValue / 2u) / maxValue);
}

unsigned char readPackedSample(
    const unsigned char* image, std::size_t index, unsigned bitDepth)
{
  const std::size_t bitOffset = index * bitDepth;
  const unsigned shift = 8u - bitDepth - static_cast<unsigned>(bitOffset % 8);
  const unsigned mask = (1u << bitDepth) - 1u;
  return expandSample((image[bitOffset / 8] >> shift) & mask, bitDepth);
}

unsigned makeRgba(
    const unsigned char* image,
    unsigned width,
    unsigned height,
    LodePNGColorType colorType,
    unsigned bitDepth,
    std::vector<unsigned char>* rgba)
{
  if (!image || !rgba)
    return kErrorInvalidInput;

  std::size_t inputBytes = 0;
  const unsigned validationError = validateColorMode(colorType, bitDepth);
  if (validationError)
    return validationError;
  if (!checkedRawBytes(width, height, colorType, bitDepth, &inputBytes))
    return kErrorOverflow;

  std::size_t outputBytes = 0;
  if (!checkedPixelBytes(width, height, 4, &outputBytes))
    return kErrorOverflow;

  if (colorType == LCT_RGBA) {
    if (bitDepth == 8) {
      rgba->assign(image, image + static_cast<std::ptrdiff_t>(inputBytes));
      return 0;
    }

    rgba->resize(outputBytes);
    for (std::size_t in = 0, out = 0; in < inputBytes; in += 8, out += 4) {
      (*rgba)[out] = image[in];
      (*rgba)[out + 1] = image[in + 2];
      (*rgba)[out + 2] = image[in + 4];
      (*rgba)[out + 3] = image[in + 6];
    }
    return 0;
  }

  if (colorType == LCT_RGB) {
    rgba->resize(outputBytes);
    if (bitDepth == 8) {
      for (std::size_t in = 0, out = 0; in < inputBytes; in += 3, out += 4) {
        (*rgba)[out] = image[in];
        (*rgba)[out + 1] = image[in + 1];
        (*rgba)[out + 2] = image[in + 2];
        (*rgba)[out + 3] = 255;
      }
      return 0;
    }

    for (std::size_t in = 0, out = 0; in < inputBytes; in += 6, out += 4) {
      (*rgba)[out] = image[in];
      (*rgba)[out + 1] = image[in + 2];
      (*rgba)[out + 2] = image[in + 4];
      (*rgba)[out + 3] = 255;
    }
    return 0;
  }

  if (colorType == LCT_GREY) {
    rgba->resize(outputBytes);
    const std::size_t pixels = static_cast<std::size_t>(width) * height;
    for (std::size_t pixel = 0; pixel < pixels; ++pixel) {
      const unsigned char grey = bitDepth < 8
                                     ? readPackedSample(image, pixel, bitDepth)
                                     : image[pixel * (bitDepth / 8)];
      const std::size_t out = pixel * 4;
      (*rgba)[out] = grey;
      (*rgba)[out + 1] = grey;
      (*rgba)[out + 2] = grey;
      (*rgba)[out + 3] = 255;
    }
    return 0;
  }

  if (colorType == LCT_GREY_ALPHA) {
    rgba->resize(outputBytes);
    const std::size_t pixels = static_cast<std::size_t>(width) * height;
    const std::size_t bytesPerPixel = bitDepth == 8 ? 2 : 4;
    for (std::size_t pixel = 0; pixel < pixels; ++pixel) {
      const std::size_t in = pixel * bytesPerPixel;
      const std::size_t out = pixel * 4;
      (*rgba)[out] = image[in];
      (*rgba)[out + 1] = image[in];
      (*rgba)[out + 2] = image[in];
      (*rgba)[out + 3] = image[in + bitDepth / 8];
    }
    return 0;
  }

  return kErrorUnsupportedColor;
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

unsigned encodeRgbaPng(
    const std::vector<unsigned char>& rgba,
    unsigned width,
    unsigned height,
    std::vector<unsigned char>* png)
{
  if (!png)
    return kErrorInvalidInput;

  std::size_t expectedSize = 0;
  if (!checkedPixelBytes(width, height, 4, &expectedSize))
    return kErrorOverflow;
  if (rgba.size() != expectedSize)
    return kErrorInvalidInput;

  png->clear();
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

  return 0;
}

} // namespace

unsigned lodepng_encode_memory(
    unsigned char** out,
    size_t* outsize,
    const unsigned char* image,
    unsigned w,
    unsigned h,
    LodePNGColorType colortype,
    unsigned bitdepth)
{
  clearOutput(out, outsize);
  if (!out || !outsize)
    return kErrorInvalidInput;

  std::vector<unsigned char> rgba;
  unsigned error = makeRgba(image, w, h, colortype, bitdepth, &rgba);
  if (error)
    return error;

  std::vector<unsigned char> png;
  error = encodeRgbaPng(rgba, w, h, &png);
  if (error)
    return error;

  auto* buffer = static_cast<unsigned char*>(std::malloc(png.size()));
  if (!buffer)
    return kErrorAllocation;

  std::memcpy(buffer, png.data(), png.size());
  *out = buffer;
  *outsize = png.size();
  return 0;
}

unsigned lodepng_encode32(
    unsigned char** out,
    size_t* outsize,
    const unsigned char* image,
    unsigned w,
    unsigned h)
{
  return lodepng_encode_memory(out, outsize, image, w, h, LCT_RGBA, 8);
}

unsigned lodepng_encode24(
    unsigned char** out,
    size_t* outsize,
    const unsigned char* image,
    unsigned w,
    unsigned h)
{
  return lodepng_encode_memory(out, outsize, image, w, h, LCT_RGB, 8);
}

unsigned lodepng_encode_file(
    const char* filename,
    const unsigned char* image,
    unsigned w,
    unsigned h,
    LodePNGColorType colortype,
    unsigned bitdepth)
{
  if (!filename)
    return kErrorInvalidInput;

  unsigned char* png = nullptr;
  size_t pngSize = 0;
  const unsigned error
      = lodepng_encode_memory(&png, &pngSize, image, w, h, colortype, bitdepth);
  if (error)
    return error;

  std::ofstream output(filename, std::ios::binary);
  if (!output) {
    std::free(png);
    return kErrorFileOpen;
  }

  output.write(
      reinterpret_cast<const char*>(png),
      static_cast<std::streamsize>(pngSize));
  std::free(png);

  return output ? 0 : kErrorFileWrite;
}

unsigned lodepng_encode32_file(
    const char* filename, const unsigned char* image, unsigned w, unsigned h)
{
  return lodepng_encode_file(filename, image, w, h, LCT_RGBA, 8);
}

unsigned lodepng_encode24_file(
    const char* filename, const unsigned char* image, unsigned w, unsigned h)
{
  return lodepng_encode_file(filename, image, w, h, LCT_RGB, 8);
}

void lodepng_free(void* ptr)
{
  std::free(ptr);
}

const char* lodepng_error_text(unsigned code)
{
  switch (code) {
    case 0:
      return "No error";
    case kErrorInvalidInput:
      return "Invalid PNG input or output argument";
    case kErrorInvalidColorType:
      return "Invalid lodepng color type";
    case kErrorInvalidBitDepth:
      return "Invalid bit depth for lodepng color type";
    case kErrorInvalidPalette:
      return "Palette encoding requires explicit palette state";
    case kErrorFileOpen:
      return "Failed to open PNG output file";
    case kErrorUnsupportedColor:
      return "Unsupported lodepng compatibility color mode";
    case kErrorAllocation:
      return "Failed to allocate PNG output buffer";
    case kErrorFileWrite:
      return "Failed to write PNG output file";
    case kErrorOverflow:
      return "PNG dimensions overflow the buffer size";
    default:
      return "Unsupported lodepng compatibility error";
  }
}

namespace lodepng {

unsigned encode(
    std::vector<unsigned char>& out,
    const unsigned char* in,
    unsigned w,
    unsigned h,
    LodePNGColorType colortype,
    unsigned bitdepth)
{
  unsigned char* png = nullptr;
  size_t pngSize = 0;
  const unsigned error
      = lodepng_encode_memory(&png, &pngSize, in, w, h, colortype, bitdepth);
  if (error)
    return error;

  out.assign(png, png + static_cast<std::ptrdiff_t>(pngSize));
  std::free(png);
  return 0;
}

unsigned encode(
    std::vector<unsigned char>& out,
    const std::vector<unsigned char>& in,
    unsigned w,
    unsigned h,
    LodePNGColorType colortype,
    unsigned bitdepth)
{
  return encode(out, in.data(), w, h, colortype, bitdepth);
}

unsigned encode(
    const std::string& filename,
    const unsigned char* in,
    unsigned w,
    unsigned h,
    LodePNGColorType colortype,
    unsigned bitdepth)
{
  return lodepng_encode_file(filename.c_str(), in, w, h, colortype, bitdepth);
}

unsigned encode(
    const std::string& filename,
    const std::vector<unsigned char>& in,
    unsigned w,
    unsigned h,
    LodePNGColorType colortype,
    unsigned bitdepth)
{
  return encode(filename, in.data(), w, h, colortype, bitdepth);
}

} // namespace lodepng
