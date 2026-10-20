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

#include "image_decoders.hpp"

#include <cstdio>

// jpeglib.h requires FILE and size_t to be declared first.
#include <jpeglib.h>
#include <png.h>

#if defined(DART_GUI_FILAMENT_JPEG_LIB_VERSION)
static_assert(
    JPEG_LIB_VERSION == DART_GUI_FILAMENT_JPEG_LIB_VERSION,
    "Configured JPEG headers do not match the jpeglib.h include path");
#endif

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>

#include <cctype>
#include <csetjmp>
#include <cstring>

namespace dart::gui::detail {
namespace {
constexpr std::size_t kMaximumDecodedBytes = 256u * 1024u * 1024u;

std::vector<std::uint8_t> readImageFile(const std::filesystem::path& path)
{
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  const auto length = file.tellg();
  if (!file || length <= 0
      || length > static_cast<std::streamoff>(kMaximumDecodedBytes))
    return {};
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
  file.seekg(0);
  file.read(reinterpret_cast<char*>(bytes.data()), length);
  return file ? std::move(bytes) : std::vector<std::uint8_t>{};
}

std::optional<ImageData> makeImage(std::uint32_t width, std::uint32_t height)
{
  const std::uint64_t pixels = static_cast<std::uint64_t>(width) * height;
  if (width == 0 || height == 0 || pixels > kMaximumDecodedBytes / 4)
    return std::nullopt;
  return ImageData{
      width,
      height,
      false,
      std::vector<std::uint8_t>(static_cast<std::size_t>(pixels) * 4, 255)};
}

struct JpegErrorManager
{
  jpeg_error_mgr base;
  jmp_buf jumpBuffer;
  char message[JMSG_LENGTH_MAX]{};
};

void handleJpegError(j_common_ptr cinfo)
{
  auto* error = reinterpret_cast<JpegErrorManager*>(cinfo->err);
  (*cinfo->err->format_message)(cinfo, error->message);
  longjmp(error->jumpBuffer, 1);
}

// Mutable decoder state and vectors stay in the caller's frame across longjmp.
bool decodeJpegImage(
    FILE* file,
    jpeg_decompress_struct& info,
    JpegErrorManager& error,
    std::optional<ImageData>& output,
    std::vector<std::uint8_t>& row)
{
  if (setjmp(error.jumpBuffer))
    return false;

  jpeg_create_decompress(&info);
  jpeg_stdio_src(&info, file);
  jpeg_read_header(&info, TRUE);
  output = makeImage(info.image_width, info.image_height);
  if (!output) {
    std::snprintf(
        error.message,
        sizeof(error.message),
        "JPEG texture exceeds decode budget");
    return false;
  }
  const bool cmyk
      = info.jpeg_color_space == JCS_CMYK || info.jpeg_color_space == JCS_YCCK;
  info.out_color_space = cmyk ? JCS_CMYK : JCS_RGB;
  jpeg_start_decompress(&info);
  if (info.output_width != output->width
      || info.output_height != output->height) {
    std::snprintf(
        error.message,
        sizeof(error.message),
        "JPEG output dimensions differ from allocated image");
    return false;
  }

  const std::size_t rowStride
      = static_cast<std::size_t>(info.output_width) * info.output_components;
  row.resize(rowStride);
  while (info.output_scanline < info.output_height) {
    JSAMPROW rowPtr = row.data();
    const std::uint32_t y = info.output_scanline;
    jpeg_read_scanlines(&info, &rowPtr, 1);

    for (std::uint32_t x = 0; x < output->width; ++x) {
      const auto* source
          = &row[static_cast<std::size_t>(x) * info.output_components];
      auto* target
          = &output
                 ->rgba[(static_cast<std::size_t>(y) * output->width + x) * 4u];
      if (cmyk) {
        for (int channel = 0; channel < 3; ++channel) {
          target[channel]
              = info.saw_Adobe_marker
                    ? static_cast<std::uint8_t>(
                        source[channel] * source[3] / 255)
                    : static_cast<std::uint8_t>(
                        (255 - source[channel]) * (255 - source[3]) / 255);
        }
        target[3] = 255;
      } else if (info.output_components == 1) {
        target[0] = source[0];
        target[1] = source[0];
        target[2] = source[0];
        target[3] = 255;
      } else {
        target[0] = source[0];
        target[1] = source[1];
        target[2] = source[2];
        target[3] = info.output_components >= 4 ? source[3] : 255;
      }
    }
  }

  jpeg_finish_decompress(&info);
  return true;
}

std::uint32_t littleEndian(
    const std::vector<std::uint8_t>& bytes, std::size_t offset, unsigned count)
{
  std::uint32_t value = 0;
  for (unsigned i = 0; i < count; ++i)
    value |= static_cast<std::uint32_t>(bytes[offset + i]) << (8 * i);
  return value;
}

bool readPpmNumber(
    const std::vector<std::uint8_t>& bytes,
    std::size_t& cursor,
    std::uint32_t& value)
{
  for (;;) {
    while (cursor < bytes.size() && std::isspace(bytes[cursor]))
      ++cursor;
    if (cursor >= bytes.size())
      return false;
    if (bytes[cursor] != '#')
      break;
    while (cursor < bytes.size() && bytes[cursor] != '\n')
      ++cursor;
  }
  value = 0;
  const auto start = cursor;
  while (cursor < bytes.size() && std::isdigit(bytes[cursor])) {
    const auto digit = bytes[cursor++] - '0';
    if (value > (std::numeric_limits<std::uint32_t>::max() - digit) / 10)
      return false;
    value = value * 10 + digit;
  }
  return cursor != start
         && (cursor == bytes.size() || std::isspace(bytes[cursor])
             || bytes[cursor] == '#');
}
} // namespace

std::optional<ImageData> loadPngImage(const std::filesystem::path& path)
{
  png_image image;
  std::memset(&image, 0, sizeof(image));
  image.version = PNG_IMAGE_VERSION;

  if (!png_image_begin_read_from_file(&image, path.string().c_str())) {
    std::cerr << "Failed to read PNG texture header: " << path.string() << " ("
              << image.message << ")\n";
    return std::nullopt;
  }

  auto output = makeImage(image.width, image.height);
  if (!output) {
    std::cerr << "PNG texture exceeds decode budget: " << path.string() << "\n";
    png_image_free(&image);
    return std::nullopt;
  }
  output->hasAlpha = (image.format & PNG_FORMAT_FLAG_ALPHA) != 0;
  image.format = PNG_FORMAT_RGBA;

  if (!png_image_finish_read(
          &image, nullptr, output->rgba.data(), 0, nullptr)) {
    std::cerr << "Failed to read PNG texture: " << path.string() << " ("
              << image.message << ")\n";
    png_image_free(&image);
    return std::nullopt;
  }

  return output;
}

std::optional<ImageData> loadJpegImage(const std::filesystem::path& path)
{
  FILE* file = std::fopen(path.string().c_str(), "rb");
  if (file == nullptr) {
    std::cerr << "Failed to open JPEG texture: " << path.string() << "\n";
    return std::nullopt;
  }

  jpeg_decompress_struct info{};
  JpegErrorManager error;
  info.err = jpeg_std_error(&error.base);
  error.base.error_exit = handleJpegError;
  std::optional<ImageData> output;
  std::vector<std::uint8_t> row;
  const bool success = decodeJpegImage(file, info, error, output, row);
  jpeg_destroy_decompress(&info);
  std::fclose(file);
  if (!success) {
    std::cerr << "Failed to read JPEG texture: " << path.string() << " ("
              << error.message << ")\n";
    return std::nullopt;
  }
  return output;
}

std::optional<ImageData> loadBmpImage(const std::filesystem::path& path)
{
  const auto bytes = readImageFile(path);
  if (bytes.size() < 54 || bytes[0] != 'B' || bytes[1] != 'M')
    return std::nullopt;
  const auto headerSize = littleEndian(bytes, 14, 4);
  if (headerSize < 40 || headerSize > bytes.size() - 14)
    return std::nullopt;
  const auto width = static_cast<std::int32_t>(littleEndian(bytes, 18, 4));
  // A negative two's-complement height stores rows top-down. Read the sign
  // bit directly instead of relying on an unsigned-to-signed conversion.
  const auto rawHeight = littleEndian(bytes, 22, 4);
  const bool topDown = (rawHeight & 0x80000000u) != 0;
  const auto height = topDown ? 0u - rawHeight : rawHeight;
  const auto bits = littleEndian(bytes, 28, 2);
  if (width <= 0 || height == 0 || height > 0x7fffffffu
      || littleEndian(bytes, 26, 2) != 1 || (bits != 24 && bits != 32)
      || littleEndian(bytes, 30, 4) != 0)
    return std::nullopt;
  const std::uint64_t rowBytes
      = ((static_cast<std::uint64_t>(width) * bits + 31) / 32) * 4;
  const auto offset = littleEndian(bytes, 10, 4);
  if (offset < 14 + headerSize || offset > bytes.size()
      || rowBytes * height > bytes.size() - offset)
    return std::nullopt;
  auto image = makeImage(static_cast<std::uint32_t>(width), height);
  if (!image)
    return std::nullopt;
  bool alphaPresent = false;
  for (std::uint32_t y = 0; y < height; ++y) {
    const auto targetY = topDown ? y : height - 1 - y;
    for (std::uint32_t x = 0; x < image->width; ++x) {
      const auto source
          = offset + y * rowBytes + static_cast<std::size_t>(x) * (bits / 8);
      const auto target
          = (static_cast<std::size_t>(targetY) * image->width + x) * 4;
      image->rgba[target] = bytes[source + 2];
      image->rgba[target + 1] = bytes[source + 1];
      image->rgba[target + 2] = bytes[source];
      if (bits == 32) {
        image->rgba[target + 3] = bytes[source + 3];
        alphaPresent |= bytes[source + 3] != 0;
      }
    }
  }
  // BI_RGB's fourth byte is commonly reserved rather than authored alpha.
  if (bits == 32 && !alphaPresent)
    for (std::size_t i = 3; i < image->rgba.size(); i += 4)
      image->rgba[i] = 255;
  image->hasAlpha = bits == 32 && alphaPresent;
  return image;
}

std::optional<ImageData> loadTgaImage(const std::filesystem::path& path)
{
  const auto bytes = readImageFile(path);
  if (bytes.size() < 18 || bytes[1] != 0 || (bytes[2] != 2 && bytes[2] != 10)
      || (bytes[16] != 24 && bytes[16] != 32))
    return std::nullopt;
  std::size_t cursor = 18 + bytes[0];
  const auto pixelBytes = bytes[16] / 8;
  const auto width = littleEndian(bytes, 12, 2);
  const auto height = littleEndian(bytes, 14, 2);
  const std::uint64_t pixelCount = static_cast<std::uint64_t>(width) * height;
  // Reject truncated headers before allocating potentially large images.
  const auto minimumBytes = bytes[2] == 2
                                ? pixelCount * pixelBytes
                                : ((pixelCount + 127) / 128) * (pixelBytes + 1);
  if (cursor > bytes.size() || minimumBytes > bytes.size() - cursor)
    return std::nullopt;
  auto image = makeImage(width, height);
  if (!image)
    return std::nullopt;
  image->hasAlpha = pixelBytes == 4 && (bytes[17] & 0x0f) != 0;
  const auto total = image->rgba.size() / 4;
  std::size_t produced = 0;
  while (produced < total) {
    std::size_t count = total - produced;
    bool repeated = false;
    if (bytes[2] == 10) {
      if (cursor >= bytes.size())
        return std::nullopt;
      const auto packet = bytes[cursor++];
      count = (packet & 0x7f) + 1;
      repeated = (packet & 0x80) != 0;
    }
    if (count > total - produced || cursor > bytes.size()
        || (!repeated && count > (bytes.size() - cursor) / pixelBytes)
        || (repeated
            && bytes.size() - cursor < static_cast<std::size_t>(pixelBytes)))
      return std::nullopt;
    for (std::size_t p = 0; p < count; ++p) {
      const auto source = cursor + (repeated ? 0 : p * pixelBytes);
      const auto linear = produced + p;
      const auto sourceY = linear / image->width;
      const auto sourceX = linear % image->width;
      const auto y = bytes[17] & 0x20 ? sourceY : image->height - 1 - sourceY;
      const auto x = bytes[17] & 0x10 ? image->width - 1 - sourceX : sourceX;
      const auto target = (y * image->width + x) * 4;
      image->rgba[target] = bytes[source + 2];
      image->rgba[target + 1] = bytes[source + 1];
      image->rgba[target + 2] = bytes[source];
      if (pixelBytes == 4 && (bytes[17] & 0x0f) != 0)
        image->rgba[target + 3] = bytes[source + 3];
    }
    cursor += (repeated ? 1 : count) * pixelBytes;
    produced += count;
  }
  return image;
}

std::optional<ImageData> loadPpmImage(const std::filesystem::path& path)
{
  const auto bytes = readImageFile(path);
  if (bytes.size() < 3 || bytes[0] != 'P'
      || (bytes[1] != '3' && bytes[1] != '6') || !std::isspace(bytes[2]))
    return std::nullopt;
  std::size_t cursor = 2;
  std::uint32_t width = 0, height = 0, maximum = 0;
  if (!readPpmNumber(bytes, cursor, width)
      || !readPpmNumber(bytes, cursor, height)
      || !readPpmNumber(bytes, cursor, maximum) || maximum == 0
      || maximum > 65535)
    return std::nullopt;
  const auto pixelCount = static_cast<std::uint64_t>(width) * height;
  const auto sampleBytes = maximum < 256 ? 1u : 2u;
  if (bytes[1] == '6') {
    if (cursor >= bytes.size() || !std::isspace(bytes[cursor]))
      return std::nullopt;
    const auto separator = bytes[cursor++];
    if (separator == '\r' && cursor < bytes.size() && bytes[cursor] == '\n')
      ++cursor;
    if (pixelCount > (bytes.size() - cursor) / (sampleBytes * 3))
      return std::nullopt;
  } else if (pixelCount > (bytes.size() - cursor) / 6) {
    // Each P3 pixel needs three numbers and their separating whitespace.
    return std::nullopt;
  }
  auto image = makeImage(width, height);
  if (!image)
    return std::nullopt;
  for (std::size_t pixel = 0; pixel < image->rgba.size() / 4; ++pixel) {
    for (unsigned channel = 0; channel < 3; ++channel) {
      std::uint32_t value = 0;
      if (bytes[1] == '3') {
        if (!readPpmNumber(bytes, cursor, value))
          return std::nullopt;
      } else {
        value = bytes[cursor++];
        if (sampleBytes == 2)
          value = value * 256 + bytes[cursor++];
      }
      if (value > maximum)
        return std::nullopt;
      image->rgba[pixel * 4 + channel]
          = static_cast<std::uint8_t>((value * 255 + maximum / 2) / maximum);
    }
  }
  return image;
}
} // namespace dart::gui::detail
