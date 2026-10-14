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

#include "textures.hpp"

#include "image_decoders.hpp"

#include <backend/PixelBufferDescriptor.h>
#include <filament/Engine.h>
#include <filament/MaterialInstance.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <jpeglib.h>
#include <png.h>

#if defined(DART_GUI_FILAMENT_JPEG_LIB_VERSION)
static_assert(
    JPEG_LIB_VERSION == DART_GUI_FILAMENT_JPEG_LIB_VERSION,
    "Configured JPEG headers do not match the jpeglib.h include path");
#endif

#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <cctype>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dart::gui::detail {
namespace {

::filament::backend::PixelBufferDescriptor makePixelBufferDescriptor(
    std::vector<std::uint8_t>&& data,
    ::filament::backend::PixelDataFormat format,
    ::filament::backend::PixelDataType type)
{
  auto* owned = new std::vector<std::uint8_t>(std::move(data));
  return ::filament::backend::PixelBufferDescriptor(
      owned->data(),
      owned->size(),
      format,
      type,
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<std::uint8_t>*>(user);
      },
      owned);
}

/// Number of mip levels for a full chain down to a 1x1 texel.
std::uint8_t computeMipLevels(std::uint32_t width, std::uint32_t height)
{
  std::uint32_t largest = std::max(width, height);
  std::uint8_t levels = 1u;
  while (largest > 1u) {
    largest >>= 1u;
    ++levels;
  }
  return levels;
}

std::string lowerExtension(const std::filesystem::path& path)
{
  std::string extension = path.extension().string();
  std::transform(
      extension.begin(), extension.end(), extension.begin(), [](char c) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      });
  return extension;
}

std::filesystem::path texturePathFromUriOrPath(std::string_view source)
{
  static constexpr std::string_view filePrefix = "file://";
  if (source.starts_with(filePrefix)) {
    return std::filesystem::path(std::string(source.substr(filePrefix.size())));
  }
  return std::filesystem::path(std::string(source));
}

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

  ImageData output;
  output.hasAlpha = (image.format & PNG_FORMAT_FLAG_ALPHA) != 0;
  image.format = PNG_FORMAT_RGBA;
  output.width = image.width;
  output.height = image.height;
  output.rgba.resize(PNG_IMAGE_SIZE(image));

  if (!png_image_finish_read(&image, nullptr, output.rgba.data(), 0, nullptr)) {
    std::cerr << "Failed to read PNG texture: " << path.string() << " ("
              << image.message << ")\n";
    png_image_free(&image);
    return std::nullopt;
  }

  return output;
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
    ImageData& output,
    std::vector<std::uint8_t>& row)
{
  if (setjmp(error.jumpBuffer))
    return false;

  jpeg_create_decompress(&info);
  jpeg_stdio_src(&info, file);
  jpeg_read_header(&info, TRUE);
  const bool cmyk
      = info.jpeg_color_space == JCS_CMYK || info.jpeg_color_space == JCS_YCCK;
  info.out_color_space = cmyk ? JCS_CMYK : JCS_RGB;
  jpeg_start_decompress(&info);

  output.width = info.output_width;
  output.height = info.output_height;
  output.rgba.resize(
      static_cast<std::size_t>(output.width) * output.height * 4u);

  const std::size_t rowStride
      = static_cast<std::size_t>(info.output_width) * info.output_components;
  row.resize(rowStride);
  while (info.output_scanline < info.output_height) {
    JSAMPROW rowPtr = row.data();
    const std::uint32_t y = info.output_scanline;
    jpeg_read_scanlines(&info, &rowPtr, 1);

    for (std::uint32_t x = 0; x < output.width; ++x) {
      const auto* source
          = &row[static_cast<std::size_t>(x) * info.output_components];
      auto* target
          = &output.rgba[(static_cast<std::size_t>(y) * output.width + x) * 4u];
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
  ImageData output;
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

std::optional<ImageData> loadTextureImage(const std::string& source)
{
  const std::filesystem::path path = texturePathFromUriOrPath(source);
  const std::string extension = lowerExtension(path);
  if (extension == ".png") {
    return loadPngImage(path);
  }
  if (extension == ".jpg" || extension == ".jpeg") {
    return loadJpegImage(path);
  }
  if (extension == ".bmp")
    return loadBmpImage(path);
  if (extension == ".tga")
    return loadTgaImage(path);
  if (extension == ".ppm" || extension == ".pnm")
    return loadPpmImage(path);

  std::cerr << "Unsupported texture image format: " << source << "\n";
  return std::nullopt;
}

::filament::Texture* createTexture(
    ::filament::Engine& engine, ImageData&& image, TextureColorSpace colorSpace)
{
  if (image.width == 0 || image.height == 0 || image.rgba.empty()) {
    return nullptr;
  }

  // Filament materials' default flipUV converts Assimp's bottom-left UVs to
  // the decoder's top-left image rows. Preserve both rows and tangent inputs.

  // A full mip chain plus trilinear/anisotropic sampling removes the aliasing
  // and shimmer that single-level textures show at distance and grazing angles.
  // generateMipmaps requires BLIT_SRC | BLIT_DST usage in addition to the
  // default sampleable/uploadable bits.
  const std::uint8_t levels = computeMipLevels(image.width, image.height);
  const ::filament::Texture::Usage usage
      = levels > 1u
            ? (::filament::Texture::Usage::DEFAULT
               | ::filament::Texture::Usage::GEN_MIPMAPPABLE)
            : ::filament::Texture::Usage::DEFAULT;
  auto* texture = ::filament::Texture::Builder()
                      .width(image.width)
                      .height(image.height)
                      .levels(levels)
                      .usage(usage)
                      .sampler(::filament::Texture::Sampler::SAMPLER_2D)
                      .format(
                          colorSpace == TextureColorSpace::Srgb
                              ? ::filament::Texture::InternalFormat::SRGB8_A8
                              : ::filament::Texture::InternalFormat::RGBA8)
                      .build(engine);
  texture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(image.rgba),
          ::filament::backend::PixelDataFormat::RGBA,
          ::filament::backend::PixelDataType::UBYTE));
  if (levels > 1u) {
    texture->generateMipmaps(engine);
  }
  return texture;
}

bool isBoundTexture(const TextureBinding* binding)
{
  return binding != nullptr && binding->texture != nullptr;
}

::filament::TextureSampler makeLegacyTextureSampler()
{
  // This sampler is also applied to the 1x1 solid fallback texture, which is
  // deliberately single-level; sampling a one-level texture with a mipmap min
  // filter is well-defined (it clamps to level 0), so the trilinear chain is
  // simply a no-op there.
  // OSG's default GL_CLAMP blends a black border at each mip. The material
  // helper restores that border over the portable CLAMP_TO_EDGE sampler.
  return ::filament::TextureSampler(
      ::filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
      ::filament::TextureSampler::MagFilter::LINEAR,
      ::filament::TextureSampler::WrapMode::CLAMP_TO_EDGE);
}

void setTextureParameter(
    ::filament::MaterialInstance& material,
    const TextureBinding& fallback,
    const char* textureName,
    const char* flagName,
    const TextureBinding* binding)
{
  const TextureBinding& active = isBoundTexture(binding) ? *binding : fallback;
  material.setParameter(
      textureName, active.texture, makeLegacyTextureSampler());
  material.setParameter(flagName, isBoundTexture(binding) ? 1.0f : 0.0f);
}

} // namespace

const TextureBinding* getOrLoadTextureBinding(
    ::filament::Engine& engine,
    TextureCache& cache,
    const std::string& source,
    TextureColorSpace colorSpace)
{
  if (source.empty()) {
    return nullptr;
  }

  const std::filesystem::path path = texturePathFromUriOrPath(source);
  std::error_code ec;
  const auto key = std::filesystem::weakly_canonical(path, ec).string();
  const std::string canonicalKey = ec ? source : key;
  const std::string cacheKey
      = std::string(colorSpace == TextureColorSpace::Srgb ? "srgb:" : "linear:")
        + canonicalKey;
  if (const auto found = cache.bindings.find(cacheKey);
      found != cache.bindings.end()) {
    return &found->second;
  }

  auto image = loadTextureImage(canonicalKey);
  if (!image) {
    return nullptr;
  }

  const bool hasAlpha = image->hasAlpha;
  auto* texture = createTexture(engine, std::move(*image), colorSpace);
  if (texture == nullptr) {
    return nullptr;
  }

  TextureBinding binding;
  binding.texture = texture;
  binding.hasAlpha = hasAlpha;
  try {
    return &cache.bindings.emplace(cacheKey, binding).first->second;
  } catch (...) {
    engine.destroy(texture);
    throw;
  }
}

void releaseUnusedTextures(
    ::filament::Engine& engine,
    TextureCache& cache,
    const std::unordered_set<::filament::Texture*>& active)
{
  // Removed material instances have already been destroyed. Driver commands
  // retain their normal submission order; no pending upload callback is lost.
  for (auto entry = cache.bindings.begin(); entry != cache.bindings.end();) {
    if (active.count(entry->second.texture) != 0) {
      ++entry;
    } else {
      engine.destroy(entry->second.texture);
      entry = cache.bindings.erase(entry);
    }
  }
}

bool hasTextureBindings(const PbrTextureBindings& textures)
{
  return textures.baseColor != nullptr || textures.metallic != nullptr
         || textures.roughness != nullptr
         || textures.metallicRoughness != nullptr || textures.normal != nullptr
         || textures.occlusion != nullptr || textures.emissive != nullptr;
}

void setBaseColorTextureParameters(
    ::filament::MaterialInstance& material,
    const TextureBinding& fallback,
    const TextureBinding* binding)
{
  setTextureParameter(
      material, fallback, "baseColorTexture", "useBaseColorTexture", binding);
  material.setParameter(
      "baseColorTextureHasAlpha",
      isBoundTexture(binding) && binding->hasAlpha ? 1.0f : 0.0f);
}

void setPbrTextureParameters(
    ::filament::MaterialInstance& material,
    const TextureBinding& fallback,
    const PbrTextureBindings& textures)
{
  setBaseColorTextureParameters(material, fallback, textures.baseColor);
  setTextureParameter(
      material,
      fallback,
      "metallicTexture",
      "useMetallicTexture",
      textures.metallic);
  setTextureParameter(
      material,
      fallback,
      "roughnessTexture",
      "useRoughnessTexture",
      textures.roughness);
  setTextureParameter(
      material,
      fallback,
      "metallicRoughnessTexture",
      "useMetallicRoughnessTexture",
      textures.metallicRoughness);
  setTextureParameter(
      material, fallback, "normalTexture", "useNormalTexture", textures.normal);
  setTextureParameter(
      material,
      fallback,
      "occlusionTexture",
      "useOcclusionTexture",
      textures.occlusion);
  setTextureParameter(
      material,
      fallback,
      "emissiveTexture",
      "useEmissiveTexture",
      textures.emissive);
}

::filament::Texture* createCheckerTexture(::filament::Engine& engine)
{
  static constexpr std::uint32_t size = 64;
  static constexpr std::uint32_t channels = 4;
  std::vector<std::uint8_t> pixels(size * size * channels);
  for (std::uint32_t y = 0; y < size; ++y) {
    for (std::uint32_t x = 0; x < size; ++x) {
      const bool light = ((x / 8u) + (y / 8u)) % 2u == 0u;
      const std::array<std::uint8_t, channels> color
          = light ? std::array<std::uint8_t, channels>{235, 245, 240, 255}
                  : std::array<std::uint8_t, channels>{42, 132, 145, 255};
      const std::size_t offset
          = (static_cast<std::size_t>(y) * size + x) * channels;
      std::copy(color.begin(), color.end(), pixels.begin() + offset);
    }
  }

  const std::uint8_t levels = computeMipLevels(size, size);
  const ::filament::Texture::Usage usage
      = levels > 1u
            ? (::filament::Texture::Usage::DEFAULT
               | ::filament::Texture::Usage::GEN_MIPMAPPABLE)
            : ::filament::Texture::Usage::DEFAULT;
  auto* texture = ::filament::Texture::Builder()
                      .width(size)
                      .height(size)
                      .levels(levels)
                      .usage(usage)
                      .sampler(::filament::Texture::Sampler::SAMPLER_2D)
                      .format(::filament::Texture::InternalFormat::SRGB8_A8)
                      .build(engine);
  texture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(pixels),
          ::filament::backend::PixelDataFormat::RGBA,
          ::filament::backend::PixelDataType::UBYTE));
  if (levels > 1u) {
    texture->generateMipmaps(engine);
  }
  return texture;
}

::filament::Texture* createSolidTexture(
    ::filament::Engine& engine,
    const std::array<std::uint8_t, 4>& color,
    TextureColorSpace colorSpace)
{
  auto* texture = ::filament::Texture::Builder()
                      .width(1)
                      .height(1)
                      .levels(1)
                      .sampler(::filament::Texture::Sampler::SAMPLER_2D)
                      .format(
                          colorSpace == TextureColorSpace::Srgb
                              ? ::filament::Texture::InternalFormat::SRGB8_A8
                              : ::filament::Texture::InternalFormat::RGBA8)
                      .build(engine);
  std::vector<std::uint8_t> pixels(color.begin(), color.end());
  texture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(pixels),
          ::filament::backend::PixelDataFormat::RGBA,
          ::filament::backend::PixelDataType::UBYTE));
  return texture;
}

} // namespace dart::gui::detail
