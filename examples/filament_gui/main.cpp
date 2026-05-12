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

#include "default_lit_material.hpp"
#include "debug_color_material.hpp"
#include "imgui_material.hpp"
#include "scene_extraction.hpp"
#include "textured_lit_material.hpp"
#include "transparent_lit_material.hpp"
#include "transparent_textured_lit_material.hpp"

#include <dart/all.hpp>
#include <dart/config.hpp>
#include <dart/io/read.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/urdf/All.hpp>

#include <GLFW/glfw3.h>
#include <backend/BufferDescriptor.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Options.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <filament/SwapChain.h>
#include <filament/ToneMapper.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <geometry/SurfaceOrientation.h>
#include <utils/EntityManager.h>

#if defined(__linux__)
  #define GLFW_EXPOSE_NATIVE_X11
  #include <GLFW/glfw3native.h>
#elif defined(_WIN32)
  #define GLFW_EXPOSE_NATIVE_WIN32
  #include <GLFW/glfw3native.h>
#elif defined(__APPLE__)
  #define GLFW_EXPOSE_NATIVE_COCOA
  #include <GLFW/glfw3native.h>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <imgui.h>
#include <jpeglib.h>
#include <png.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace {

using dart::dynamics::BoxShape;
using dart::dynamics::CollisionAspect;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::Shape;
using dart::dynamics::ShapePtr;
using dart::dynamics::ShapeNode;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;
using dart::examples::filament_gui::DebugDrawOptions;
using dart::examples::filament_gui::DebugLineDescriptor;
using dart::examples::filament_gui::OrbitCamera;
using dart::examples::filament_gui::OrbitCameraUpdate;
using dart::examples::filament_gui::PickRay;
using dart::examples::filament_gui::RenderableDescriptor;
using dart::examples::filament_gui::RenderableId;
using dart::examples::filament_gui::RunOptions;
using dart::examples::filament_gui::ShapeKind;
using dart::examples::filament_gui::ViewerLifecycleState;
using dart::examples::filament_gui::cameraEye;
using dart::examples::filament_gui::computePlaneDragTranslation;
using dart::examples::filament_gui::extractContactDebugLines;
using dart::examples::filament_gui::extractDebugLines;
using dart::examples::filament_gui::extractRenderables;
using dart::examples::filament_gui::intersectPlane;
using dart::examples::filament_gui::markFrameRendered;
using dart::examples::filament_gui::markFrameSkipped;
using dart::examples::filament_gui::markScreenshotRequested;
using dart::examples::filament_gui::markSimulationAdvanced;
using dart::examples::filament_gui::makeOrbitCameraBasis;
using dart::examples::filament_gui::makePerspectivePickRay;
using dart::examples::filament_gui::makeSelectionDebugLines;
using dart::examples::filament_gui::normalizeRunOptions;
using dart::examples::filament_gui::pickNearestRenderable;
using dart::examples::filament_gui::requestSingleStep;
using dart::examples::filament_gui::shouldAdvanceSimulation;
using dart::examples::filament_gui::shouldRequestScreenshot;
using dart::examples::filament_gui::shouldStopAfterFrame;
using dart::examples::filament_gui::togglePaused;
using dart::examples::filament_gui::translateFrameRenderable;
using dart::examples::filament_gui::updateOrbitCamera;
using dart::examples::filament_gui::writeRgbaPpm;
using dart::simulation::World;
using filament::math::float3;
using filament::math::float4;
using filament::math::mat4f;
using utils::Entity;
using utils::EntityManager;

struct Vertex
{
  float3 position;
  filament::math::short4 tangent;
  filament::math::float2 uv = {0.0f, 0.0f};
};

struct TextureBinding
{
  filament::Texture* texture = nullptr;
  filament::TextureSampler sampler;
};

enum class TextureColorSpace
{
  Linear,
  Srgb,
};

struct PbrTextureBindings
{
  const TextureBinding* baseColor = nullptr;
  const TextureBinding* metallic = nullptr;
  const TextureBinding* roughness = nullptr;
  const TextureBinding* metallicRoughness = nullptr;
  const TextureBinding* normal = nullptr;
  const TextureBinding* occlusion = nullptr;
  const TextureBinding* emissive = nullptr;
};

struct ImageData
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::uint8_t> rgba;
};

struct TextureCache
{
  std::unordered_map<std::string, TextureBinding> bindings;
  std::vector<filament::Texture*> ownedTextures;
};

struct MaterialSet
{
  filament::Material& defaultLit;
  filament::Material& texturedLit;
  filament::Material& transparentLit;
  filament::Material& transparentTexturedLit;
  TextureBinding checkerTexture;
  TextureBinding fallbackTexture;
};

struct Renderable
{
  Entity entity;
  filament::VertexBuffer* vertexBuffer = nullptr;
  filament::IndexBuffer* indexBuffer = nullptr;
  struct MaterialInstance
  {
    filament::MaterialInstance* instance = nullptr;
    float4 baseColor{0.0f, 0.0f, 0.0f, 1.0f};
    bool hasBaseColor = false;
    bool followsDescriptorColor = false;
  };
  std::vector<MaterialInstance> materials;
};

struct SceneRenderable
{
  RenderableId id = 0;
  Renderable renderable;
};

struct OverlayMesh
{
  Entity entity;
  filament::VertexBuffer* vertexBuffer = nullptr;
  filament::IndexBuffer* indexBuffer = nullptr;
  std::size_t vertexCount = 0;
  std::size_t indexCount = 0;
};

struct ImGuiVertex
{
  filament::math::float3 position;
  filament::math::float2 uv;
  std::uint32_t color = 0;
};

struct DebugVertex
{
  filament::math::float3 position;
  std::uint32_t color = 0;
};

struct ScreenshotCapture
{
  std::vector<std::uint8_t> pixels;
  std::mutex mutex;
  std::condition_variable condition;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  bool done = false;
};

struct CameraController
{
  OrbitCamera camera;
  double lastX = 0.0;
  double lastY = 0.0;
  double scrollDelta = 0.0;
  bool hasLastCursor = false;
};

struct ImGuiOverlay
{
  filament::View* view = nullptr;
  filament::Scene* scene = nullptr;
  filament::Camera* camera = nullptr;
  Entity cameraEntity;
  filament::Material* material = nullptr;
  filament::MaterialInstance* materialInstance = nullptr;
  filament::Texture* fontTexture = nullptr;
  OverlayMesh mesh;
};

template <typename T, std::size_t Size>
filament::backend::BufferDescriptor makeBufferDescriptor(
    const std::array<T, Size>& data)
{
  auto* owned = new std::array<T, Size>(data);
  return filament::backend::BufferDescriptor(
      owned->data(),
      owned->size() * sizeof(T),
      [](void*, std::size_t, void* user) {
        delete static_cast<std::array<T, Size>*>(user);
      },
      owned);
}

template <typename T>
filament::backend::BufferDescriptor makeBufferDescriptor(std::vector<T>&& data)
{
  auto* owned = new std::vector<T>(std::move(data));
  return filament::backend::BufferDescriptor(
      owned->data(),
      owned->size() * sizeof(T),
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<T>*>(user);
      },
      owned);
}

filament::backend::PixelBufferDescriptor makePixelBufferDescriptor(
    std::vector<std::uint8_t>&& data,
    filament::backend::PixelDataFormat format,
    filament::backend::PixelDataType type)
{
  auto* owned = new std::vector<std::uint8_t>(std::move(data));
  return filament::backend::PixelBufferDescriptor(
      owned->data(),
      owned->size(),
      format,
      type,
      [](void*, std::size_t, void* user) {
        delete static_cast<std::vector<std::uint8_t>*>(user);
      },
      owned);
}

filament::ColorGrading* createDebugColorGrading(filament::Engine& engine)
{
  filament::PBRNeutralToneMapper toneMapper;
  return filament::ColorGrading::Builder()
      .quality(filament::ColorGrading::QualityLevel::HIGH)
      .toneMapper(&toneMapper)
      .luminanceScaling(true)
      .gamutMapping(true)
      .build(engine);
}

filament::IndirectLight* createNeutralIndirectLight(filament::Engine& engine)
{
  static constexpr std::array<float3, 9> kDiffuseIrradiance = {{
      {0.22f, 0.24f, 0.28f},
      {-0.01f, -0.01f, -0.01f},
      {0.04f, 0.045f, 0.052f},
      {0.018f, 0.020f, 0.024f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
      {0.000f, 0.000f, 0.000f},
  }};

  return filament::IndirectLight::Builder()
      .irradiance(3, kDiffuseIrradiance.data())
      .intensity(52000.0f)
      .build(engine);
}

filament::Skybox* createNeutralSkybox(filament::Engine& engine)
{
  return filament::Skybox::Builder()
      .color({0.46f, 0.50f, 0.56f, 1.0f})
      .showSun(true)
      .build(engine);
}

filament::TextureSampler makeRepeatTextureSampler()
{
  filament::TextureSampler sampler(
      filament::TextureSampler::MinFilter::LINEAR,
      filament::TextureSampler::MagFilter::LINEAR,
      filament::TextureSampler::WrapMode::REPEAT);
  sampler.setAnisotropy(8.0f);
  return sampler;
}

void loadImGuiFont(ImGuiIO& io)
{
  std::vector<std::filesystem::path> candidates;
  if (const char* fontPath = std::getenv("DART_FILAMENT_GUI_FONT");
      fontPath != nullptr && std::strlen(fontPath) > 0) {
    candidates.emplace_back(fontPath);
  }

  candidates.emplace_back("/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf");
  candidates.emplace_back("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
  candidates.emplace_back(
      "/usr/share/fonts/liberation-sans-fonts/LiberationSans-Regular.ttf");
  candidates.emplace_back("/System/Library/Fonts/Supplemental/Arial.ttf");
  candidates.emplace_back("/Library/Fonts/Arial.ttf");
  candidates.emplace_back("C:/Windows/Fonts/segoeui.ttf");

  ImFontConfig config;
  config.OversampleH = 3;
  config.OversampleV = 2;
  config.PixelSnapH = false;
  for (const auto& path : candidates) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
      continue;
    }
    if (io.Fonts->AddFontFromFileTTF(path.string().c_str(), 15.0f, &config)
        != nullptr) {
      return;
    }
  }

  io.Fonts->AddFontDefault();
}

std::string lowerExtension(const std::filesystem::path& path)
{
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](char c) {
    return static_cast<char>(
        std::tolower(static_cast<unsigned char>(c)));
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

std::filesystem::path resolveExamplePath(const std::string& path)
{
  const std::filesystem::path input(path);
  if (input.is_absolute() || std::filesystem::exists(input)) {
    return std::filesystem::absolute(input);
  }

  const std::filesystem::path sourcePath
      = std::filesystem::path(DART_FILAMENT_GUI_REPOSITORY_ROOT) / input;
  if (std::filesystem::exists(sourcePath)) {
    return std::filesystem::absolute(sourcePath);
  }

  return std::filesystem::absolute(input);
}

std::optional<ImageData> loadPngImage(const std::filesystem::path& path)
{
  png_image image;
  std::memset(&image, 0, sizeof(image));
  image.version = PNG_IMAGE_VERSION;

  if (!png_image_begin_read_from_file(&image, path.string().c_str())) {
    std::cerr << "Failed to read PNG texture header: " << path.string()
              << " (" << image.message << ")\n";
    return std::nullopt;
  }

  image.format = PNG_FORMAT_RGBA;
  ImageData output;
  output.width = image.width;
  output.height = image.height;
  output.rgba.resize(PNG_IMAGE_SIZE(image));

  if (!png_image_finish_read(
          &image, nullptr, output.rgba.data(), 0, nullptr)) {
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

  if (setjmp(error.jumpBuffer)) {
    std::cerr << "Failed to read JPEG texture: " << path.string() << " ("
              << error.message << ")\n";
    jpeg_destroy_decompress(&info);
    std::fclose(file);
    return std::nullopt;
  }

  jpeg_create_decompress(&info);
  jpeg_stdio_src(&info, file);
  jpeg_read_header(&info, TRUE);
  jpeg_start_decompress(&info);

  ImageData output;
  output.width = info.output_width;
  output.height = info.output_height;
  output.rgba.resize(
      static_cast<std::size_t>(output.width) * output.height * 4u);

  const std::size_t rowStride
      = static_cast<std::size_t>(info.output_width) * info.output_components;
  std::vector<std::uint8_t> row(rowStride);
  while (info.output_scanline < info.output_height) {
    JSAMPROW rowPtr = row.data();
    const std::uint32_t y = info.output_scanline;
    jpeg_read_scanlines(&info, &rowPtr, 1);

    for (std::uint32_t x = 0; x < output.width; ++x) {
      const auto* source = &row[static_cast<std::size_t>(x)
                               * info.output_components];
      auto* target = &output.rgba[
          (static_cast<std::size_t>(y) * output.width + x) * 4u];
      if (info.output_components == 1) {
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
  jpeg_destroy_decompress(&info);
  std::fclose(file);
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

  std::cerr << "Unsupported texture image format: " << source << "\n";
  return std::nullopt;
}

filament::Texture* createTexture(
    filament::Engine& engine, ImageData&& image, TextureColorSpace colorSpace)
{
  if (image.width == 0 || image.height == 0 || image.rgba.empty()) {
    return nullptr;
  }

  auto* texture = filament::Texture::Builder()
                      .width(image.width)
                      .height(image.height)
                      .levels(1)
                      .sampler(filament::Texture::Sampler::SAMPLER_2D)
                      .format(
                          colorSpace == TextureColorSpace::Srgb
                              ? filament::Texture::InternalFormat::SRGB8_A8
                              : filament::Texture::InternalFormat::RGBA8)
                      .build(engine);
  texture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(image.rgba),
          filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE));
  return texture;
}

const TextureBinding* getOrLoadTextureBinding(
    filament::Engine& engine,
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

  auto* texture = createTexture(engine, std::move(*image), colorSpace);
  if (texture == nullptr) {
    return nullptr;
  }

  TextureBinding binding;
  binding.texture = texture;
  binding.sampler = makeRepeatTextureSampler();
  cache.ownedTextures.push_back(texture);
  const auto [inserted, _] = cache.bindings.emplace(cacheKey, binding);
  return &inserted->second;
}

bool hasTextureBindings(const PbrTextureBindings& textures)
{
  return textures.baseColor != nullptr || textures.metallic != nullptr
         || textures.roughness != nullptr
         || textures.metallicRoughness != nullptr || textures.normal != nullptr
         || textures.occlusion != nullptr || textures.emissive != nullptr;
}

bool isBoundTexture(const TextureBinding* binding)
{
  return binding != nullptr && binding->texture != nullptr;
}

void setTextureParameter(
    filament::MaterialInstance& material,
    const TextureBinding& fallback,
    const char* textureName,
    const char* flagName,
    const TextureBinding* binding)
{
  const TextureBinding& active = isBoundTexture(binding) ? *binding : fallback;
  material.setParameter(textureName, active.texture, active.sampler);
  material.setParameter(flagName, isBoundTexture(binding) ? 1.0f : 0.0f);
}

void setPbrTextureParameters(
    filament::MaterialInstance& material,
    const TextureBinding& fallback,
    const PbrTextureBindings& textures)
{
  setTextureParameter(
      material,
      fallback,
      "baseColorTexture",
      "useBaseColorTexture",
      textures.baseColor);
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
      material,
      fallback,
      "normalTexture",
      "useNormalTexture",
      textures.normal);
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

filament::Texture* createCheckerTexture(filament::Engine& engine)
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
      const std::size_t offset = (static_cast<std::size_t>(y) * size + x)
                                 * channels;
      std::copy(color.begin(), color.end(), pixels.begin() + offset);
    }
  }

  auto* texture = filament::Texture::Builder()
                      .width(size)
                      .height(size)
                      .levels(1)
                      .sampler(filament::Texture::Sampler::SAMPLER_2D)
                      .format(filament::Texture::InternalFormat::SRGB8_A8)
                      .build(engine);
  texture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(pixels),
          filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE));
  return texture;
}

filament::Texture* createSolidTexture(
    filament::Engine& engine,
    const std::array<std::uint8_t, 4>& color,
    TextureColorSpace colorSpace)
{
  auto* texture = filament::Texture::Builder()
                      .width(1)
                      .height(1)
                      .levels(1)
                      .sampler(filament::Texture::Sampler::SAMPLER_2D)
                      .format(
                          colorSpace == TextureColorSpace::Srgb
                              ? filament::Texture::InternalFormat::SRGB8_A8
                              : filament::Texture::InternalFormat::RGBA8)
                      .build(engine);
  std::vector<std::uint8_t> pixels(color.begin(), color.end());
  texture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(pixels),
          filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE));
  return texture;
}

struct DartScene
{
  dart::simulation::WorldPtr world;
};

enum class ExampleScene
{
  Mvp,
  DragAndDrop,
};

struct AppOptions
{
  RunOptions run;
  ExampleScene scene = ExampleScene::Mvp;
  bool showUi = true;
  bool showUiExplicit = false;
};

constexpr const char* kWamFixtureSkeletonName = "visual_wam_robot";
constexpr const char* kAtlasFixtureSkeletonName = "visual_atlas_torso_mesh";
constexpr const char* kAtlasRobotFixtureSkeletonName = "visual_atlas_robot";
constexpr const char* kPbrEnvironmentFixtureSkeletonName =
    "visual_pbr_environment";
constexpr std::size_t kMinPbrEnvironmentRenderableCount = 4;
constexpr std::size_t kMinDragAndDropFrameRenderableCount = 5;

const char* sceneName(ExampleScene scene)
{
  switch (scene) {
    case ExampleScene::Mvp:
      return "mvp";
    case ExampleScene::DragAndDrop:
      return "drag-and-drop";
  }
  return "mvp";
}

bool parseSceneName(std::string_view name, ExampleScene& scene)
{
  if (name == "mvp") {
    scene = ExampleScene::Mvp;
    return true;
  }
  if (name == "drag-and-drop") {
    scene = ExampleScene::DragAndDrop;
    return true;
  }
  return false;
}

OrbitCamera initialCameraForScene(ExampleScene scene)
{
  OrbitCamera camera;
  if (scene == ExampleScene::DragAndDrop) {
    camera.target = Eigen::Vector3d(0.35, 0.15, 0.9);
    camera.yaw = -0.72;
    camera.pitch = 0.58;
    camera.distance = 9.5;
  } else {
    camera.target = Eigen::Vector3d(0.15, 0.55, 0.75);
    camera.yaw = -0.95;
    camera.pitch = 0.38;
    camera.distance = 7.2;
  }
  return camera;
}

std::shared_ptr<dart::math::TriMesh<double>> createTetraMesh()
{
  auto mesh = std::make_shared<dart::math::TriMesh<double>>();
  mesh->reserveVertices(4);
  mesh->reserveTriangles(4);
  mesh->addVertex(-0.45, -0.35, -0.25);
  mesh->addVertex(0.45, -0.35, -0.25);
  mesh->addVertex(0.0, 0.45, -0.25);
  mesh->addVertex(0.0, 0.0, 0.55);
  mesh->addTriangle(0, 2, 1);
  mesh->addTriangle(0, 1, 3);
  mesh->addTriangle(1, 2, 3);
  mesh->addTriangle(2, 0, 3);
  mesh->computeVertexNormals();
  return mesh;
}

std::shared_ptr<MeshShape> loadExampleMeshShape(
    const std::string& path, const Eigen::Vector3d& scale)
{
  const std::filesystem::path resolvedPath = resolveExamplePath(path);
  dart::utils::MeshLoaderd loader;
  auto mesh = loader.load(resolvedPath.string());
  if (!mesh) {
    std::cerr << "Failed to load example mesh: " << resolvedPath.string()
              << "\n";
    return nullptr;
  }

  auto sharedMesh = std::shared_ptr<dart::math::TriMesh<double>>(
      std::move(mesh));
  const auto uri
      = dart::common::Uri::createFromStringOrPath(resolvedPath.string());
  return std::make_shared<MeshShape>(scale, std::move(sharedMesh), uri);
}

std::shared_ptr<MeshShape> loadRequiredExampleMeshShape(
    const std::string& path, const Eigen::Vector3d& scale)
{
  auto meshShape = loadExampleMeshShape(path, scale);
  if (!meshShape) {
    throw std::runtime_error("Failed to load required mesh fixture: " + path);
  }
  return meshShape;
}

dart::dynamics::SkeletonPtr loadRequiredWamRobotSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      "herb_description", dart::config::dataPath("urdf/wam"));
  const auto wamUri = dart::common::Uri::createFromPath(
      dart::config::dataPath("urdf/wam/wam.urdf"));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (!wam) {
    throw std::runtime_error(
        "Failed to load WAM robot fixture from " + wamUri.toString());
  }

  wam->setName(kWamFixtureSkeletonName);
  const std::array<std::pair<const char*, double>, 7> jointPositions{
      { {"/j1", 0.0},
        {"/j2", -0.55},
        {"/j3", 0.25},
        {"/j4", 1.05},
        {"/j5", 0.0},
        {"/j6", 0.7},
        {"/j7", 0.0} }};
  for (const auto& [name, position] : jointPositions) {
    auto* dof = wam->getDof(name);
    if (dof == nullptr) {
      throw std::runtime_error(
          "WAM robot fixture is missing expected DOF " + std::string(name));
    }
    dof->setPosition(position);
  }

  return wam;
}

dart::dynamics::SkeletonPtr loadRequiredAtlasRobotSkeleton()
{
  const auto atlasUri = dart::common::Uri::createFromString(
      "dart://sample/sdf/atlas/atlas_v3_no_head.sdf");
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (!atlas) {
    throw std::runtime_error(
        "Failed to load Atlas robot fixture from " + atlasUri.toString());
  }

  atlas->setName(kAtlasRobotFixtureSkeletonName);
  auto* rootBody = atlas->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<FreeJoint*>(rootBody->getParentJoint()) != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation() = Eigen::Vector3d(0.35, 1.8, 0.95);
    FreeJoint::setTransformOf(rootBody, transform);
  }

  return atlas;
}

dart::dynamics::SkeletonPtr createRequiredPbrEnvironmentSkeleton()
{
  auto environment = Skeleton::create(kPbrEnvironmentFixtureSkeletonName);
  auto* body = environment->createJointAndBodyNodePair<WeldJoint>().second;
  auto pbrPanelShape = loadRequiredExampleMeshShape(
      "data/gltf/pbr_multi_material.gltf", Eigen::Vector3d(0.85, 0.85, 0.85));

  const auto addPanel = [&](const std::string& name,
                            const Eigen::Isometry3d& transform) {
    auto* shapeNode = body->createShapeNodeWith<VisualAspect>(pbrPanelShape);
    shapeNode->setName(name);
    shapeNode->setRelativeTransform(transform);
    shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d::Ones());
  };

  constexpr double halfPi = 1.5707963267948966;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-2.7, 2.0, 0.12);
  addPanel("pbr_environment_floor_left", transform);

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-1.15, 2.0, 0.12);
  addPanel("pbr_environment_floor_right", transform);

  transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(-2.7, 2.9, 1.05);
  addPanel("pbr_environment_backdrop_left", transform);

  transform = Eigen::Isometry3d::Identity();
  transform.linear()
      = Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(-1.15, 2.9, 1.05);
  addPanel("pbr_environment_backdrop_right", transform);

  return environment;
}

AppOptions parseOptions(int argc, char* argv[])
{
  AppOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--frames" && i + 1 < argc) {
      options.run.maxFrames = std::atoi(argv[++i]);
    } else if (arg == "--width" && i + 1 < argc) {
      options.run.width = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--height" && i + 1 < argc) {
      options.run.height = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--screenshot" && i + 1 < argc) {
      options.run.screenshotPath = argv[++i];
    } else if (arg == "--headless") {
      options.run.headless = true;
    } else if (arg == "--hide-ui") {
      options.showUi = false;
      options.showUiExplicit = true;
    } else if (arg == "--show-ui") {
      options.showUi = true;
      options.showUiExplicit = true;
    } else if (arg == "--scene" && i + 1 < argc) {
      const std::string_view sceneArg(argv[++i]);
      if (!parseSceneName(sceneArg, options.scene)) {
        std::cerr << "Unknown scene '" << sceneArg
                  << "'. Expected 'mvp' or 'drag-and-drop'.\n";
        std::exit(2);
      }
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--frames N] [--width N] [--height N]"
                   " [--screenshot PATH] [--headless]"
                   " [--hide-ui|--show-ui]"
                   " [--scene mvp|drag-and-drop]\n";
      std::exit(0);
    }
  }
  normalizeRunOptions(options.run);
  if (options.run.headless && !options.showUiExplicit) {
    options.showUi = false;
  }
  return options;
}

void* getNativeWindow(GLFWwindow* window)
{
#if defined(__linux__)
  return reinterpret_cast<void*>(glfwGetX11Window(window));
#elif defined(_WIN32)
  return glfwGetWin32Window(window);
#elif defined(__APPLE__)
  return glfwGetCocoaWindow(window);
#else
  (void)window;
  return nullptr;
#endif
}

void handleScroll(GLFWwindow* window, double, double yOffset)
{
  auto* controller
      = static_cast<CameraController*>(glfwGetWindowUserPointer(window));
  if (controller != nullptr) {
    controller->scrollDelta += yOffset;
  }
}

bool isKeyDown(GLFWwindow* window, int key)
{
  return window != nullptr && glfwGetKey(window, key) == GLFW_PRESS;
}

bool isDragModifierDown(GLFWwindow* window)
{
  return isKeyDown(window, GLFW_KEY_LEFT_CONTROL)
         || isKeyDown(window, GLFW_KEY_RIGHT_CONTROL);
}

Eigen::Vector3d selectedNudgeFromKeyboard(
    GLFWwindow* window, const OrbitCamera& camera, double stepSize)
{
  if (window == nullptr || stepSize <= 0.0) {
    return Eigen::Vector3d::Zero();
  }

  const double speedMultiplier
      = isKeyDown(window, GLFW_KEY_LEFT_SHIFT)
            || isKeyDown(window, GLFW_KEY_RIGHT_SHIFT)
        ? 3.0
        : 1.0;
  const double step = stepSize * speedMultiplier;

  const auto basis = makeOrbitCameraBasis(camera);
  Eigen::Vector3d forward = basis.forward;
  forward.z() = 0.0;
  if (forward.squaredNorm() < 1e-12) {
    forward = -Eigen::Vector3d::UnitY();
  } else {
    forward.normalize();
  }

  Eigen::Vector3d right = basis.right;
  right.z() = 0.0;
  if (right.squaredNorm() < 1e-12) {
    right = Eigen::Vector3d::UnitX();
  } else {
    right.normalize();
  }

  Eigen::Vector3d nudge = Eigen::Vector3d::Zero();
  if (isKeyDown(window, GLFW_KEY_LEFT)) {
    nudge -= right * step;
  }
  if (isKeyDown(window, GLFW_KEY_RIGHT)) {
    nudge += right * step;
  }
  if (isKeyDown(window, GLFW_KEY_UP)) {
    nudge += forward * step;
  }
  if (isKeyDown(window, GLFW_KEY_DOWN)) {
    nudge -= forward * step;
  }
  if (isKeyDown(window, GLFW_KEY_PAGE_UP)) {
    nudge += Eigen::Vector3d::UnitZ() * step;
  }
  if (isKeyDown(window, GLFW_KEY_PAGE_DOWN)) {
    nudge -= Eigen::Vector3d::UnitZ() * step;
  }
  return nudge;
}

DartScene createMvpDartScene()
{
  DartScene scene;

  auto createDynamicBox = [](const std::string& name,
                             const Eigen::Vector3d& size,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& color) {
    auto box = Skeleton::create(name);
    auto [joint, body] = box->createJointAndBodyNodePair<FreeJoint>();
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = position;
    joint->setTransformFromParentBodyNode(tf);

    auto boxShape = std::make_shared<BoxShape>(size);
    auto boxShapeNode = body->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(boxShape);
    boxShapeNode->getVisualAspect()->setColor(color);
    body->setInertia(
        dart::dynamics::Inertia(
            1.0,
            Eigen::Vector3d::Zero(),
            boxShapeNode->getShape()->computeInertia(1.0)));
    return box;
  };

  auto blueBox = createDynamicBox(
      "falling_blue_box",
      Eigen::Vector3d(0.35, 0.35, 0.35),
      Eigen::Vector3d(-0.35, 0.0, 1.5),
      dart::Color::Blue());
  auto orangeBox = createDynamicBox(
      "falling_orange_box",
      Eigen::Vector3d(0.28, 0.28, 0.28),
      Eigen::Vector3d(0.35, 0.1, 2.15),
      dart::Color::Orange());
  auto contactBox = createDynamicBox(
      "contact_green_box",
      Eigen::Vector3d(0.3, 0.3, 0.3),
      Eigen::Vector3d(-1.05, 0.25, 0.18),
      dart::Color::Green());

  auto ground = Skeleton::create("ground");
  auto groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
  auto groundShapeNode = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(8.0, 8.0, 0.1)));
  groundShapeNode->getVisualAspect()->setColor(dart::Color::LightGray());

  auto createStaticVisual = [](const std::string& name,
                               const ShapePtr& shape,
                               const Eigen::Vector3d& position,
                               const Eigen::Vector3d& color,
                               double alpha = 1.0) {
    auto skeleton = Skeleton::create(name);
    auto [joint, body] = skeleton->createJointAndBodyNodePair<WeldJoint>();
    (void)joint;
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = position;
    auto* shapeNode = body->createShapeNodeWith<VisualAspect>(shape);
    shapeNode->setRelativeTransform(tf);
    shapeNode->getVisualAspect()->setRGBA(
        Eigen::Vector4d(color.x(), color.y(), color.z(), alpha));
    return skeleton;
  };

  scene.world = World::create("filament_gui_mvp");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(blueBox);
  scene.world->addSkeleton(orangeBox);
  scene.world->addSkeleton(contactBox);
  scene.world->addSkeleton(ground);
  scene.world->addSkeleton(createStaticVisual(
      "visual_sphere",
      std::make_shared<dart::dynamics::SphereShape>(0.3),
      Eigen::Vector3d(0.0, 0.0, 2.7),
      Eigen::Vector3d(0.2, 0.72, 0.55)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_cylinder",
      std::make_shared<dart::dynamics::CylinderShape>(0.22, 0.7),
      Eigen::Vector3d(-0.75, -0.15, 0.8),
      Eigen::Vector3d(0.78, 0.58, 0.18)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_capsule",
      std::make_shared<dart::dynamics::CapsuleShape>(0.18, 0.62),
      Eigen::Vector3d(0.0, -0.15, 0.85),
      Eigen::Vector3d(0.72, 0.28, 0.68)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_cone",
      std::make_shared<dart::dynamics::ConeShape>(0.25, 0.65),
      Eigen::Vector3d(0.75, -0.15, 0.8),
      Eigen::Vector3d(0.92, 0.38, 0.2)));
  scene.world->addSkeleton(createStaticVisual(
      "visual_ellipsoid",
      std::make_shared<dart::dynamics::EllipsoidShape>(
          Eigen::Vector3d(0.7, 0.4, 0.32)),
      Eigen::Vector3d(1.5, -0.15, 0.8),
      Eigen::Vector3d(0.25, 0.48, 0.88),
      0.55));
  scene.world->addSkeleton(createStaticVisual(
      "visual_mesh",
      std::make_shared<MeshShape>(
          Eigen::Vector3d(0.8, 0.8, 0.8),
          createTetraMesh(),
          dart::common::Uri{}),
      Eigen::Vector3d(-1.5, -0.15, 0.9),
      Eigen::Vector3d(0.45, 0.4, 0.92)));
  scene.world->addSkeleton(createStaticVisual(
      kAtlasFixtureSkeletonName,
      loadRequiredExampleMeshShape(
          "data/sdf/atlas/utorso.dae", Eigen::Vector3d(0.75, 0.75, 0.75)),
      Eigen::Vector3d(2.2, 0.25, 1.1),
      Eigen::Vector3d(0.72, 0.72, 0.78)));
  if (auto wamBaseMesh = loadExampleMeshShape(
          "data/urdf/wam/meshes/wam/wam1.dae",
          Eigen::Vector3d(0.65, 0.65, 0.65))) {
    scene.world->addSkeleton(createStaticVisual(
        "visual_wam_textured_mesh",
        wamBaseMesh,
        Eigen::Vector3d(2.35, 0.55, 0.45),
        Eigen::Vector3d::Ones()));
  }
  scene.world->addSkeleton(loadRequiredWamRobotSkeleton());
  scene.world->addSkeleton(loadRequiredAtlasRobotSkeleton());
  if (auto pbrMesh = loadExampleMeshShape(
          "data/gltf/pbr_triangle.gltf",
          Eigen::Vector3d(0.9, 0.9, 0.9))) {
    scene.world->addSkeleton(createStaticVisual(
        "visual_gltf_pbr_mesh",
        pbrMesh,
        Eigen::Vector3d(2.2, -0.75, 0.35),
        Eigen::Vector3d::Ones()));
  }
  if (auto multiMaterialPbrMesh = loadExampleMeshShape(
          "data/gltf/pbr_multi_material.gltf",
          Eigen::Vector3d(0.5, 0.5, 0.5))) {
    scene.world->addSkeleton(createStaticVisual(
        "visual_gltf_multi_material_pbr_mesh",
        multiMaterialPbrMesh,
        Eigen::Vector3d(1.45, -1.05, 0.55),
        Eigen::Vector3d::Ones()));
  }
  scene.world->addSkeleton(createRequiredPbrEnvironmentSkeleton());
  scene.world->addSkeleton(createStaticVisual(
      "visual_plane",
      std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d(0.0, 2.65, 0.08),
      Eigen::Vector3d(0.25, 0.7, 0.78),
      0.65));

  Eigen::Isometry3d simpleFrameTransform = Eigen::Isometry3d::Identity();
  simpleFrameTransform.translation() = Eigen::Vector3d(-2.25, 1.05, 0.55);
  auto simpleFrame = SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      "visual_draggable_simple_frame",
      simpleFrameTransform);
  simpleFrame->setShape(
      std::make_shared<BoxShape>(Eigen::Vector3d(0.35, 0.35, 0.35)));
  simpleFrame->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.95, 0.72, 0.18, 0.82));
  scene.world->addSimpleFrame(simpleFrame);

  return scene;
}

DartScene createDragAndDropScene()
{
  DartScene scene;
  scene.world = World::create("filament_gui_drag_and_drop");
  scene.world->setGravity(Eigen::Vector3d::Zero());

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(4.0, -4.0, 0.0);
  auto anchor = std::make_shared<SimpleFrame>(
      dart::dynamics::Frame::World(), "interactive frame", transform);
  anchor->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.45, 0.45, 0.45)));
  anchor->getVisualAspect(true)->setColor(Eigen::Vector3d(0.95, 0.7, 0.15));
  scene.world->addSimpleFrame(anchor);

  transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-4.0, 4.0, 0.0);
  auto draggable = anchor->spawnChildSimpleFrame("draggable", transform);
  draggable->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(1.0, 1.0, 1.0)));
  draggable->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0.0, 0.0));
  scene.world->addSimpleFrame(draggable);

  const auto addMarker = [&](const std::string& name,
                             const Eigen::Vector3d& position,
                             const Eigen::Vector3d& color) {
    Eigen::Isometry3d markerTransform = Eigen::Isometry3d::Identity();
    markerTransform.translation() = position;
    auto marker = std::make_shared<SimpleFrame>(
        dart::dynamics::Frame::World(), name, markerTransform);
    marker->setShape(
        std::make_shared<BoxShape>(Eigen::Vector3d(0.25, 0.25, 0.25)));
    marker->getVisualAspect(true)->setColor(color);
    scene.world->addSimpleFrame(marker);
  };

  addMarker("X", Eigen::Vector3d(8.0, 0.0, 0.0), Eigen::Vector3d(0.9, 0.0, 0.0));
  addMarker("Y", Eigen::Vector3d(0.0, 8.0, 0.0), Eigen::Vector3d(0.0, 0.9, 0.0));
  addMarker("Z", Eigen::Vector3d(0.0, 0.0, 8.0), Eigen::Vector3d(0.0, 0.0, 0.9));

  return scene;
}

DartScene createDartScene(ExampleScene scene)
{
  switch (scene) {
    case ExampleScene::Mvp:
      return createMvpDartScene();
    case ExampleScene::DragAndDrop:
      return createDragAndDropScene();
  }
  return createMvpDartScene();
}

float3 toFloat3(const Eigen::Vector3d& vector)
{
  return {
      static_cast<float>(vector.x()),
      static_cast<float>(vector.y()),
      static_cast<float>(vector.z())};
}

float4 toRgba(const Eigen::Vector4d& rgba)
{
  return {
      static_cast<float>(rgba.x()),
      static_cast<float>(rgba.y()),
      static_cast<float>(rgba.z()),
      static_cast<float>(rgba.w())};
}

float3 rgb(const float4& color)
{
  return {color.x, color.y, color.z};
}

float4 withRgb(const float4& color, const float3& newRgb)
{
  return {newRgb.x, newRgb.y, newRgb.z, color.w};
}

float luminance(const float3& color)
{
  return 0.2126f * color.x + 0.7152f * color.y + 0.0722f * color.z;
}

float4 ensureReadableDisplayColor(const float4& color)
{
  constexpr float kMinLuminance = 0.18f;
  const float3 colorRgb = rgb(color);
  const float currentLuminance = luminance(colorRgb);
  if (currentLuminance >= kMinLuminance || color.w <= 0.0f) {
    return color;
  }

  constexpr float3 kNeutralMeshColor{0.46f, 0.48f, 0.52f};
  const float blend
      = std::clamp((kMinLuminance - currentLuminance) / kMinLuminance, 0.0f, 1.0f);
  return {
      std::clamp(color.x * (1.0f - blend) + kNeutralMeshColor.x * blend, 0.0f, 1.0f),
      std::clamp(color.y * (1.0f - blend) + kNeutralMeshColor.y * blend, 0.0f, 1.0f),
      std::clamp(color.z * (1.0f - blend) + kNeutralMeshColor.z * blend, 0.0f, 1.0f),
      color.w};
}

bool isTransparent(const float4& color)
{
  return color.w < 0.999f;
}

float3 selectionTint(const float3& color)
{
  return {
      std::min(1.0f, color.x * 0.45f + 0.55f),
      std::min(1.0f, color.y * 0.45f + 0.48f),
      std::min(1.0f, color.z * 0.45f + 0.12f)};
}

mat4f toFilamentTransform(const Eigen::Isometry3d& tf)
{
  const Eigen::Matrix4d m = tf.matrix();
  mat4f out;
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out[col][row] = static_cast<float>(m(row, col));
    }
  }
  return out;
}

void updateCameraController(
    GLFWwindow* window,
    CameraController& controller,
    bool suppressLeftMouseOrbit = false)
{
  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(window, &x, &y);

  if (!controller.hasLastCursor) {
    controller.lastX = x;
    controller.lastY = y;
    controller.hasLastCursor = true;
  }

  const double dx = x - controller.lastX;
  const double dy = y - controller.lastY;
  controller.lastX = x;
  controller.lastY = y;

  const bool orbit
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS
        && !suppressLeftMouseOrbit && !isDragModifierDown(window);
  const bool pan
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS
        || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;

  OrbitCameraUpdate update;
  update.deltaX = dx;
  update.deltaY = dy;
  update.scrollDelta = controller.scrollDelta;
  update.orbit = orbit;
  update.pan = pan;
  updateOrbitCamera(controller.camera, update);
  controller.scrollDelta = 0.0;
}

filament::MaterialInstance* addRenderableMaterial(
    Renderable& renderable,
    filament::Material& material,
    const std::optional<float4>& baseColor = std::nullopt,
    bool followsDescriptorColor = false)
{
  auto* instance = material.createInstance();
  renderable.materials.push_back(
      {instance,
       baseColor.value_or(float4{0.0f, 0.0f, 0.0f, 1.0f}),
       baseColor.has_value(),
       followsDescriptorColor});
  return instance;
}

void updateRenderableSelection(
    Renderable& renderable, const float4& descriptorColor, bool selected)
{
  for (Renderable::MaterialInstance& material : renderable.materials) {
    if (material.instance == nullptr || !material.hasBaseColor) {
      continue;
    }
    if (material.followsDescriptorColor) {
      material.baseColor = descriptorColor;
    }
    material.instance->setParameter(
        "baseColor",
        selected ? withRgb(material.baseColor, selectionTint(rgb(material.baseColor)))
                 : material.baseColor);
  }
}

void configureLitMaterialInstance(
    filament::MaterialInstance& material,
    const float4& color,
    float metallic,
    float roughness,
    const float3& emissiveColor,
    const PbrTextureBindings& textures = {},
    const TextureBinding* fallbackTexture = nullptr)
{
  material.setParameter("baseColor", color);
  material.setParameter("metallic", std::clamp(metallic, 0.0f, 1.0f));
  material.setParameter("roughness", std::clamp(roughness, 0.04f, 1.0f));
  material.setParameter("reflectance", 0.5f);
  material.setParameter("emissiveColor", emissiveColor);
  if (hasTextureBindings(textures)) {
    if (fallbackTexture == nullptr || fallbackTexture->texture == nullptr) {
      std::cerr << "Textured Filament material is missing fallback texture\n";
      std::exit(1);
    }
    setPbrTextureParameters(material, *fallbackTexture, textures);
  }
}

filament::Material& selectLitMaterial(
    const MaterialSet& materials, bool usesTextures, const float4& color)
{
  if (isTransparent(color)) {
    return usesTextures ? materials.transparentTexturedLit
                        : materials.transparentLit;
  }
  return usesTextures ? materials.texturedLit : materials.defaultLit;
}

Renderable createBoxRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const float3& halfExtents,
    const float4& color)
{
  static constexpr std::array<std::uint16_t, 36> indices = {
      0,  1,  2,  0,  2,  3,  4,  6,  5,  4,  7,  6,  8,  9,  10, 8,  10, 11,
      12, 14, 13, 12, 15, 14, 16, 17, 18, 16, 18, 19, 20, 22, 21, 20, 23, 22};
  static constexpr std::array<filament::math::ushort3, 12> triangles = {{
      {0, 1, 2},
      {0, 2, 3},
      {4, 6, 5},
      {4, 7, 6},
      {8, 9, 10},
      {8, 10, 11},
      {12, 14, 13},
      {12, 15, 14},
      {16, 17, 18},
      {16, 18, 19},
      {20, 22, 21},
      {20, 23, 22},
  }};

  const auto hx = halfExtents.x;
  const auto hy = halfExtents.y;
  const auto hz = halfExtents.z;
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  std::array<Vertex, 24> vertices = {{
      {{-hx, -hy, hz}, tangent},  {{hx, -hy, hz}, tangent},
      {{hx, hy, hz}, tangent},    {{-hx, hy, hz}, tangent},
      {{-hx, -hy, -hz}, tangent}, {{hx, -hy, -hz}, tangent},
      {{hx, hy, -hz}, tangent},   {{-hx, hy, -hz}, tangent},
      {{-hx, hy, -hz}, tangent},  {{hx, hy, -hz}, tangent},
      {{hx, hy, hz}, tangent},    {{-hx, hy, hz}, tangent},
      {{-hx, -hy, -hz}, tangent}, {{hx, -hy, -hz}, tangent},
      {{hx, -hy, hz}, tangent},   {{-hx, -hy, hz}, tangent},
      {{hx, -hy, -hz}, tangent},  {{hx, hy, -hz}, tangent},
      {{hx, hy, hz}, tangent},    {{hx, -hy, hz}, tangent},
      {{-hx, -hy, -hz}, tangent}, {{-hx, hy, -hz}, tangent},
      {{-hx, hy, hz}, tangent},   {{-hx, -hy, hz}, tangent},
  }};

  std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
      filament::geometry::SurfaceOrientation::Builder()
          .vertexCount(vertices.size())
          .positions(&vertices[0].position, sizeof(Vertex))
          .triangleCount(triangles.size())
          .triangles(triangles.data())
          .build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    std::exit(1);
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));

  Renderable renderable;
  renderable.vertexBuffer
      = filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                filament::VertexAttribute::POSITION,
                0,
                filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(Vertex, position),
                sizeof(Vertex))
            .attribute(
                filament::VertexAttribute::TANGENTS,
                0,
                filament::VertexBuffer::AttributeType::SHORT4,
                offsetof(Vertex, tangent),
                sizeof(Vertex))
            .attribute(
                filament::VertexAttribute::UV0,
                0,
                filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(Vertex, uv),
                sizeof(Vertex))
            .normalized(filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(vertices));

  renderable.indexBuffer
      = filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(filament::IndexBuffer::IndexType::USHORT)
            .build(engine);
  renderable.indexBuffer->setBuffer(engine, makeBufferDescriptor(indices));

  auto* materialInstance
      = addRenderableMaterial(renderable, material, color, true);
  configureLitMaterialInstance(
      *materialInstance,
      color,
      0.0f,
      0.55f,
      float3{0.0f, 0.0f, 0.0f});

  renderable.entity = EntityManager::get().create();
  filament::RenderableManager::Builder(1)
      .boundingBox(
          {{-halfExtents.x, -halfExtents.y, -halfExtents.z},
           {halfExtents.x, halfExtents.y, halfExtents.z}})
      .material(0, materialInstance)
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::TRIANGLES,
          renderable.vertexBuffer,
          renderable.indexBuffer,
          0,
          indices.size())
      .castShadows(true)
      .receiveShadows(true)
      .build(engine, renderable.entity);
  auto& renderables = engine.getRenderableManager();
  renderables.setScreenSpaceContactShadows(
      renderables.getInstance(renderable.entity), true);

  return renderable;
}

void appendTriangle(
    std::vector<std::uint32_t>& indices,
    std::vector<filament::math::uint3>& triangles,
    std::uint32_t a,
    std::uint32_t b,
    std::uint32_t c)
{
  indices.push_back(a);
  indices.push_back(b);
  indices.push_back(c);
  triangles.push_back({a, b, c});
}

struct Bounds
{
  float3 min;
  float3 max;
};

Bounds computeBounds(const std::vector<Vertex>& vertices)
{
  Bounds bounds{vertices.front().position, vertices.front().position};
  for (const Vertex& vertex : vertices) {
    bounds.min.x = std::min(bounds.min.x, vertex.position.x);
    bounds.min.y = std::min(bounds.min.y, vertex.position.y);
    bounds.min.z = std::min(bounds.min.z, vertex.position.z);
    bounds.max.x = std::max(bounds.max.x, vertex.position.x);
    bounds.max.y = std::max(bounds.max.y, vertex.position.y);
    bounds.max.z = std::max(bounds.max.z, vertex.position.z);
  }
  return bounds;
}

Bounds computeDebugBounds(const std::vector<DebugVertex>& vertices)
{
  Bounds bounds{vertices.front().position, vertices.front().position};
  for (const DebugVertex& vertex : vertices) {
    bounds.min.x = std::min(bounds.min.x, vertex.position.x);
    bounds.min.y = std::min(bounds.min.y, vertex.position.y);
    bounds.min.z = std::min(bounds.min.z, vertex.position.z);
    bounds.max.x = std::max(bounds.max.x, vertex.position.x);
    bounds.max.y = std::max(bounds.max.y, vertex.position.y);
    bounds.max.z = std::max(bounds.max.z, vertex.position.z);
  }
  return bounds;
}

std::uint8_t toByte(double channel)
{
  return static_cast<std::uint8_t>(
      std::clamp(channel, 0.0, 1.0) * 255.0 + 0.5);
}

std::uint32_t packColor(const Eigen::Vector4d& rgba)
{
  const auto red = static_cast<std::uint32_t>(toByte(rgba.x()));
  const auto green = static_cast<std::uint32_t>(toByte(rgba.y()));
  const auto blue = static_cast<std::uint32_t>(toByte(rgba.z()));
  const auto alpha = static_cast<std::uint32_t>(toByte(rgba.w()));
  return red | (green << 8u) | (blue << 16u) | (alpha << 24u);
}

std::optional<Renderable> createDebugLineRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const std::vector<DebugLineDescriptor>& lines)
{
  if (lines.empty()) {
    return std::nullopt;
  }

  std::vector<DebugVertex> vertices;
  std::vector<std::uint32_t> indices;
  vertices.reserve(lines.size() * 2u);
  indices.reserve(lines.size() * 2u);

  for (const DebugLineDescriptor& line : lines) {
    const auto start = static_cast<std::uint32_t>(vertices.size());
    const std::uint32_t color = packColor(line.rgba);
    vertices.push_back({toFloat3(line.from), color});
    vertices.push_back({toFloat3(line.to), color});
    indices.push_back(start);
    indices.push_back(start + 1u);
  }

  const Bounds bounds = computeDebugBounds(vertices);
  const std::size_t indexCount = indices.size();
  Renderable renderable;
  renderable.vertexBuffer
      = filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                filament::VertexAttribute::POSITION,
                0,
                filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(DebugVertex, position),
                sizeof(DebugVertex))
            .attribute(
                filament::VertexAttribute::COLOR,
                0,
                filament::VertexBuffer::AttributeType::UBYTE4,
                offsetof(DebugVertex, color),
                sizeof(DebugVertex))
            .normalized(filament::VertexAttribute::COLOR)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  auto* materialInstance = addRenderableMaterial(renderable, material);
  renderable.entity = EntityManager::get().create();
  filament::RenderableManager::Builder(1)
      .boundingBox({bounds.min, bounds.max})
      .material(0, materialInstance)
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::LINES,
          renderable.vertexBuffer,
          renderable.indexBuffer,
          0,
          indexCount)
      .castShadows(false)
      .receiveShadows(false)
      .build(engine, renderable.entity);

  return renderable;
}

Renderable createTriangleMeshRenderable(
    filament::Engine& engine,
    filament::Material& material,
    std::vector<Vertex> vertices,
    std::vector<std::uint32_t> indices,
    std::vector<filament::math::uint3> triangles,
    const float4& color,
    const float3& minBounds,
    const float3& maxBounds,
    const PbrTextureBindings& textures = {},
    const TextureBinding* fallbackTexture = nullptr,
    float metallic = 0.0f,
    float roughness = 0.58f,
    float3 emissiveColor = {0.0f, 0.0f, 0.0f},
    bool followsDescriptorColor = true)
{
  const std::size_t indexCount = indices.size();
  std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
      filament::geometry::SurfaceOrientation::Builder()
          .vertexCount(vertices.size())
          .positions(&vertices[0].position, sizeof(Vertex))
          .triangleCount(triangles.size())
          .triangles(triangles.data())
          .build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    std::exit(1);
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));

  Renderable renderable;
  renderable.vertexBuffer
      = filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                filament::VertexAttribute::POSITION,
                0,
                filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(Vertex, position),
                sizeof(Vertex))
            .attribute(
                filament::VertexAttribute::TANGENTS,
                0,
                filament::VertexBuffer::AttributeType::SHORT4,
                offsetof(Vertex, tangent),
                sizeof(Vertex))
            .attribute(
                filament::VertexAttribute::UV0,
                0,
                filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(Vertex, uv),
                sizeof(Vertex))
            .normalized(filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  auto* materialInstance = addRenderableMaterial(
      renderable, material, color, followsDescriptorColor);
  configureLitMaterialInstance(
      *materialInstance,
      color,
      metallic,
      roughness,
      emissiveColor,
      textures,
      fallbackTexture);

  renderable.entity = EntityManager::get().create();
  filament::RenderableManager::Builder(1)
      .boundingBox({minBounds, maxBounds})
      .material(0, materialInstance)
      .geometry(
          0,
          filament::RenderableManager::PrimitiveType::TRIANGLES,
          renderable.vertexBuffer,
          renderable.indexBuffer,
          0,
          indexCount)
      .castShadows(true)
      .receiveShadows(true)
      .build(engine, renderable.entity);
  auto& renderables = engine.getRenderableManager();
  renderables.setScreenSpaceContactShadows(
      renderables.getInstance(renderable.entity), true);

  return renderable;
}

Renderable createEllipsoidRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const Eigen::Vector3d& radii,
    const float4& color)
{
  static constexpr std::uint32_t segments = 32;
  static constexpr std::uint32_t rings = 16;
  static constexpr double pi = 3.14159265358979323846;
  const filament::math::short4 tangent = {0, 0, 0, 32767};

  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve((rings + 1) * (segments + 1));
  indices.reserve(rings * segments * 6);
  triangles.reserve(rings * segments * 2);

  for (std::uint32_t ring = 0; ring <= rings; ++ring) {
    const double theta = -pi / 2.0 + pi * ring / rings;
    const double z = std::sin(theta);
    const double radial = std::cos(theta);
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double phi = 2.0 * pi * segment / segments;
      vertices.push_back(
          Vertex{{static_cast<float>(radii.x() * radial * std::cos(phi)),
                  static_cast<float>(radii.y() * radial * std::sin(phi)),
                  static_cast<float>(radii.z() * z)},
                 tangent});
    }
  }

  for (std::uint32_t ring = 0; ring < rings; ++ring) {
    for (std::uint32_t segment = 0; segment < segments; ++segment) {
      const std::uint32_t a = ring * (segments + 1) + segment;
      const std::uint32_t b = a + 1;
      const std::uint32_t c = (ring + 1) * (segments + 1) + segment;
      const std::uint32_t d = c + 1;
      appendTriangle(indices, triangles, a, b, c);
      appendTriangle(indices, triangles, b, d, c);
    }
  }

  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      color,
      toFloat3(-radii),
      toFloat3(radii));
}

Renderable createCylinderRenderable(
    filament::Engine& engine,
    filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  static constexpr std::uint32_t segments = 40;
  static constexpr double pi = 3.14159265358979323846;
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const float halfHeight = static_cast<float>(height * 0.5);

  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve((segments + 1) * 4 + 2);

  for (const float z : {-halfHeight, halfHeight}) {
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double phi = 2.0 * pi * segment / segments;
      vertices.push_back(
          Vertex{{static_cast<float>(radius * std::cos(phi)),
                  static_cast<float>(radius * std::sin(phi)),
                  z},
                 tangent});
    }
  }

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    const std::uint32_t a = segment;
    const std::uint32_t b = segment + 1;
    const std::uint32_t c = (segments + 1) + segment;
    const std::uint32_t d = c + 1;
    appendTriangle(indices, triangles, a, b, c);
    appendTriangle(indices, triangles, b, d, c);
  }

  const std::uint32_t bottomCenter = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, -halfHeight}, tangent});
  const std::uint32_t topCenter = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, halfHeight}, tangent});
  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    appendTriangle(indices, triangles, bottomCenter, segment + 1, segment);
    appendTriangle(
        indices,
        triangles,
        topCenter,
        (segments + 1) + segment,
        (segments + 1) + segment + 1);
  }

  const float r = static_cast<float>(radius);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      color,
      {-r, -r, -halfHeight},
      {r, r, halfHeight});
}

Renderable createConeRenderable(
    filament::Engine& engine,
    filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  static constexpr std::uint32_t segments = 40;
  static constexpr double pi = 3.14159265358979323846;
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const float halfHeight = static_cast<float>(height * 0.5);

  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(segments + 3);

  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * pi * segment / segments;
    vertices.push_back(
        Vertex{{static_cast<float>(radius * std::cos(phi)),
                static_cast<float>(radius * std::sin(phi)),
                -halfHeight},
               tangent});
  }
  const std::uint32_t tip = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, halfHeight}, tangent});
  const std::uint32_t center = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, -halfHeight}, tangent});

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    appendTriangle(indices, triangles, segment, segment + 1, tip);
    appendTriangle(indices, triangles, center, segment + 1, segment);
  }

  const float r = static_cast<float>(radius);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      color,
      {-r, -r, -halfHeight},
      {r, r, halfHeight});
}

Renderable createCapsuleRenderable(
    filament::Engine& engine,
    filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  static constexpr std::uint32_t segments = 32;
  static constexpr std::uint32_t hemisphereRings = 8;
  static constexpr double pi = 3.14159265358979323846;
  const filament::math::short4 tangent = {0, 0, 0, 32767};

  struct Ring
  {
    double z;
    double radius;
  };
  std::vector<Ring> rings;
  rings.reserve(hemisphereRings * 2 + 2);
  for (std::uint32_t i = 0; i <= hemisphereRings; ++i) {
    const double phi = pi / 2.0 - (pi / 2.0) * i / hemisphereRings;
    rings.push_back(
        {height * 0.5 + radius * std::sin(phi), radius * std::cos(phi)});
  }
  if (height > 0.0) {
    rings.push_back({-height * 0.5, radius});
  }
  for (std::uint32_t i = 1; i <= hemisphereRings; ++i) {
    const double phi = -(pi / 2.0) * i / hemisphereRings;
    rings.push_back(
        {-height * 0.5 + radius * std::sin(phi), radius * std::cos(phi)});
  }

  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(rings.size() * (segments + 1));
  for (const Ring& ring : rings) {
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double theta = 2.0 * pi * segment / segments;
      vertices.push_back(
          Vertex{{static_cast<float>(ring.radius * std::cos(theta)),
                  static_cast<float>(ring.radius * std::sin(theta)),
                  static_cast<float>(ring.z)},
                 tangent});
    }
  }

  for (std::uint32_t ring = 0; ring + 1 < rings.size(); ++ring) {
    for (std::uint32_t segment = 0; segment < segments; ++segment) {
      const std::uint32_t a = ring * (segments + 1) + segment;
      const std::uint32_t b = a + 1;
      const std::uint32_t c = (ring + 1) * (segments + 1) + segment;
      const std::uint32_t d = c + 1;
      appendTriangle(indices, triangles, a, b, c);
      appendTriangle(indices, triangles, b, d, c);
    }
  }

  const float r = static_cast<float>(radius);
  const float halfTotalHeight = static_cast<float>(height * 0.5 + radius);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      color,
      {-r, -r, -halfTotalHeight},
      {r, r, halfTotalHeight});
}

std::optional<Renderable> createMeshRenderable(
    filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const MeshShape& meshShape,
    const float4& color)
{
  const auto triMesh = meshShape.getTriMesh();
  if (!triMesh || triMesh->getVertices().empty()
      || triMesh->getTriangles().empty()) {
    return std::nullopt;
  }
  if (triMesh->getVertices().size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const Eigen::Vector3d& scale = meshShape.getScale();
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const auto textureCoords = meshShape.getTextureCoords();
  const bool hasTextureCoords
      = meshShape.getTextureCoordComponents() >= 2
        && textureCoords.size() == triMesh->getVertices().size();

  std::vector<Vertex> vertices;
  vertices.reserve(triMesh->getVertices().size());
  const auto& meshVertices = triMesh->getVertices();
  for (std::size_t i = 0; i < meshVertices.size(); ++i) {
    filament::math::float2 uv = {0.0f, 0.0f};
    if (hasTextureCoords) {
      uv = {
          static_cast<float>(textureCoords[i].x()),
          static_cast<float>(textureCoords[i].y())};
    }
    vertices.push_back(
        Vertex{toFloat3(scale.cwiseProduct(meshVertices[i])), tangent, uv});
  }

  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  indices.reserve(triMesh->getTriangles().size() * 3);
  triangles.reserve(triMesh->getTriangles().size());
  for (const auto& triangle : triMesh->getTriangles()) {
    if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size()
        || triangle[2] >= vertices.size()) {
      return std::nullopt;
    }
    appendTriangle(
        indices,
        triangles,
        static_cast<std::uint32_t>(triangle[0]),
        static_cast<std::uint32_t>(triangle[1]),
        static_cast<std::uint32_t>(triangle[2]));
  }

  struct MeshMaterialState
  {
    float4 baseColor;
    PbrTextureBindings textures;
    float metallic = 0.0f;
    float roughness = 0.58f;
    float3 emissiveColor{0.0f, 0.0f, 0.0f};
    bool followsDescriptorColor = true;
  };

  const auto makeMaterialState = [&](unsigned int materialIndex) {
    MeshMaterialState state;
    state.baseColor = color;

    if (meshShape.getColorMode() != MeshShape::MATERIAL_COLOR
        || meshShape.getNumMaterials() == 0) {
      return state;
    }

    const auto* meshMaterial = meshShape.getMaterial(materialIndex);
    if (meshMaterial != nullptr) {
      state.followsDescriptorColor = false;
      state.baseColor = {
          meshMaterial->diffuse.x(),
          meshMaterial->diffuse.y(),
          meshMaterial->diffuse.z(),
          meshMaterial->diffuse.w()};
      state.metallic = meshMaterial->metallicFactor;
      state.roughness = meshMaterial->roughnessFactor;
      state.emissiveColor = {
          meshMaterial->emissive.x(),
          meshMaterial->emissive.y(),
          meshMaterial->emissive.z()};

      if (hasTextureCoords) {
        const auto loadBinding
            = [&](const std::string& source,
                  TextureColorSpace colorSpace) -> const TextureBinding* {
          return getOrLoadTextureBinding(
              engine, textureCache, source, colorSpace);
        };

        const std::string& baseColorTexturePath
            = !meshMaterial->baseColorTexturePath.empty()
                  ? meshMaterial->baseColorTexturePath
                  : (!meshMaterial->textureImagePaths.empty()
                         ? meshMaterial->textureImagePaths[0]
                         : meshMaterial->baseColorTexturePath);
        state.textures.baseColor = loadBinding(
            baseColorTexturePath, TextureColorSpace::Srgb);
        state.textures.metallic = loadBinding(
            meshMaterial->metallicTexturePath, TextureColorSpace::Linear);
        state.textures.roughness = loadBinding(
            meshMaterial->roughnessTexturePath, TextureColorSpace::Linear);
        state.textures.metallicRoughness = loadBinding(
            meshMaterial->metallicRoughnessTexturePath,
            TextureColorSpace::Linear);
        state.textures.normal = loadBinding(
            meshMaterial->normalTexturePath, TextureColorSpace::Linear);
        state.textures.occlusion = loadBinding(
            meshMaterial->occlusionTexturePath, TextureColorSpace::Linear);
        state.textures.emissive = loadBinding(
            meshMaterial->emissiveTexturePath, TextureColorSpace::Srgb);
      }
    }

    state.baseColor = ensureReadableDisplayColor(state.baseColor);
    return state;
  };

  const auto makeMaterialInstance
      = [&](Renderable& renderable,
            const MeshMaterialState& state) -> filament::MaterialInstance* {
    const bool usesTextures = hasTextureBindings(state.textures);
    auto* materialInstance = addRenderableMaterial(
        renderable,
        selectLitMaterial(materials, usesTextures, state.baseColor),
        state.baseColor,
        state.followsDescriptorColor);
    configureLitMaterialInstance(
        *materialInstance,
        state.baseColor,
        state.metallic,
        state.roughness,
        state.emissiveColor,
        state.textures,
        usesTextures ? &materials.fallbackTexture : nullptr);
    return materialInstance;
  };

  const Bounds bounds = computeBounds(vertices);
  const auto subMeshRanges = meshShape.getSubMeshRanges();
  std::vector<MeshShape::SubMeshRange> parts;
  parts.reserve(subMeshRanges.size());
  std::size_t coveredTriangleCount = 0;
  bool useSubMeshes = !subMeshRanges.empty();
  for (const MeshShape::SubMeshRange& range : subMeshRanges) {
    if (range.triangleCount == 0) {
      continue;
    }
    if (range.triangleOffset + range.triangleCount
        > triMesh->getTriangles().size()) {
      useSubMeshes = false;
      break;
    }
    coveredTriangleCount += range.triangleCount;
    parts.push_back(range);
  }
  useSubMeshes = useSubMeshes && !parts.empty()
                 && coveredTriangleCount == triMesh->getTriangles().size();
  if (!useSubMeshes) {
    const MeshMaterialState state = makeMaterialState(0u);
    const bool usesTextures = hasTextureBindings(state.textures);
    return createTriangleMeshRenderable(
        engine,
        selectLitMaterial(materials, usesTextures, state.baseColor),
        std::move(vertices),
        std::move(indices),
        std::move(triangles),
        state.baseColor,
        bounds.min,
        bounds.max,
        state.textures,
        usesTextures ? &materials.fallbackTexture : nullptr,
        state.metallic,
        state.roughness,
        state.emissiveColor,
        state.followsDescriptorColor);
  }

  std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
      filament::geometry::SurfaceOrientation::Builder()
          .vertexCount(vertices.size())
          .positions(&vertices[0].position, sizeof(Vertex))
          .triangleCount(triangles.size())
          .triangles(triangles.data())
          .build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    std::exit(1);
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));

  Renderable renderable;
  renderable.vertexBuffer
      = filament::VertexBuffer::Builder()
            .vertexCount(vertices.size())
            .bufferCount(1)
            .attribute(
                filament::VertexAttribute::POSITION,
                0,
                filament::VertexBuffer::AttributeType::FLOAT3,
                offsetof(Vertex, position),
                sizeof(Vertex))
            .attribute(
                filament::VertexAttribute::TANGENTS,
                0,
                filament::VertexBuffer::AttributeType::SHORT4,
                offsetof(Vertex, tangent),
                sizeof(Vertex))
            .attribute(
                filament::VertexAttribute::UV0,
                0,
                filament::VertexBuffer::AttributeType::FLOAT2,
                offsetof(Vertex, uv),
                sizeof(Vertex))
            .normalized(filament::VertexAttribute::TANGENTS)
            .build(engine);
  renderable.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));

  renderable.indexBuffer
      = filament::IndexBuffer::Builder()
            .indexCount(indices.size())
            .bufferType(filament::IndexBuffer::IndexType::UINT)
            .build(engine);
  renderable.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));

  renderable.entity = EntityManager::get().create();
  auto builder = filament::RenderableManager::Builder(parts.size());
  builder.boundingBox({bounds.min, bounds.max})
      .castShadows(true)
      .receiveShadows(true);
  for (std::size_t partIndex = 0; partIndex < parts.size(); ++partIndex) {
    const auto& part = parts[partIndex];
    auto* materialInstance
        = makeMaterialInstance(renderable, makeMaterialState(part.materialIndex));
    builder.material(partIndex, materialInstance)
        .geometry(
            partIndex,
            filament::RenderableManager::PrimitiveType::TRIANGLES,
            renderable.vertexBuffer,
            renderable.indexBuffer,
            part.triangleOffset * 3u,
            part.triangleCount * 3u);
  }
  builder.build(engine, renderable.entity);

  auto& renderables = engine.getRenderableManager();
  renderables.setScreenSpaceContactShadows(
      renderables.getInstance(renderable.entity), true);

  return renderable;
}

Renderable createPlaneRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const Eigen::Vector3d& normal,
    double offset,
    const float4& color,
    const TextureBinding* textureBinding = nullptr,
    const TextureBinding* fallbackTexture = nullptr)
{
  static constexpr double halfExtent = 1.0;
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  Eigen::Vector3d unitNormal = normal;
  if (unitNormal.squaredNorm() < 1e-12) {
    unitNormal = Eigen::Vector3d::UnitZ();
  } else {
    unitNormal.normalize();
  }
  const Eigen::Vector3d seed
      = std::abs(unitNormal.z()) < 0.9 ? Eigen::Vector3d::UnitZ()
                                       : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d axisU = seed.cross(unitNormal).normalized();
  const Eigen::Vector3d axisV = unitNormal.cross(axisU).normalized();
  const Eigen::Vector3d center = unitNormal * offset;

  std::vector<Vertex> vertices{
      {toFloat3(center - axisU * halfExtent - axisV * halfExtent),
       tangent,
       {0.0f, 0.0f}},
      {toFloat3(center + axisU * halfExtent - axisV * halfExtent),
       tangent,
       {4.0f, 0.0f}},
      {toFloat3(center + axisU * halfExtent + axisV * halfExtent),
       tangent,
       {4.0f, 4.0f}},
      {toFloat3(center - axisU * halfExtent + axisV * halfExtent),
       tangent,
       {0.0f, 4.0f}},
  };
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  indices.reserve(6);
  triangles.reserve(2);
  appendTriangle(indices, triangles, 0, 1, 2);
  appendTriangle(indices, triangles, 0, 2, 3);

  const Bounds bounds = computeBounds(vertices);
  PbrTextureBindings textures;
  textures.baseColor = textureBinding;
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      color,
      bounds.min,
      bounds.max,
      textures,
      textureBinding != nullptr ? fallbackTexture : nullptr);
}

std::optional<Renderable> createRenderableFromDescriptor(
    filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const RenderableDescriptor& descriptor)
{
  const float4 color = toRgba(descriptor.material.rgba);
  auto& solidMaterial = selectLitMaterial(materials, false, color);
  auto& texturedMaterial = selectLitMaterial(materials, true, color);
  std::optional<Renderable> renderable;
  switch (descriptor.geometry.kind) {
    case ShapeKind::Box:
      renderable = createBoxRenderable(
          engine,
          solidMaterial,
          toFloat3(descriptor.geometry.size * 0.5),
          color);
      break;
    case ShapeKind::Sphere:
      renderable = createEllipsoidRenderable(
          engine,
          solidMaterial,
          Eigen::Vector3d::Constant(descriptor.geometry.radius),
          color);
      break;
    case ShapeKind::Ellipsoid:
      renderable = createEllipsoidRenderable(
          engine, solidMaterial, descriptor.geometry.size * 0.5, color);
      break;
    case ShapeKind::Cylinder:
      renderable = createCylinderRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::Cone:
      renderable = createConeRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::Capsule:
      renderable = createCapsuleRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::Mesh:
      if (const auto* meshShape = dynamic_cast<const MeshShape*>(
              descriptor.shape)) {
        renderable = createMeshRenderable(
            engine, materials, textureCache, *meshShape, color);
      }
      break;
    case ShapeKind::Plane:
      renderable = createPlaneRenderable(
          engine,
          texturedMaterial,
          descriptor.geometry.normal,
          descriptor.geometry.offset,
          color,
          &materials.checkerTexture,
          &materials.fallbackTexture);
      break;
    case ShapeKind::Unsupported:
      break;
  }

  if (renderable) {
    auto& renderables = engine.getRenderableManager();
    const auto instance = renderables.getInstance(renderable->entity);
    renderables.setCastShadows(instance, descriptor.material.castsShadows);
    renderables.setReceiveShadows(instance, descriptor.material.receivesShadows);
    renderables.setScreenSpaceContactShadows(
        instance,
        descriptor.material.castsShadows
            || descriptor.material.receivesShadows);
  }

  return renderable;
}

void destroyRenderable(filament::Engine& engine, Renderable& renderable)
{
  engine.destroy(renderable.entity);
  for (Renderable::MaterialInstance& material : renderable.materials) {
    if (material.instance != nullptr) {
      engine.destroy(material.instance);
      material.instance = nullptr;
    }
  }
  renderable.materials.clear();
  engine.destroy(renderable.vertexBuffer);
  engine.destroy(renderable.indexBuffer);
  EntityManager::get().destroy(renderable.entity);
}

void destroyOverlayMesh(
    filament::Engine& engine, filament::Scene* scene, OverlayMesh& mesh)
{
  if (mesh.entity) {
    if (scene != nullptr) {
      scene->remove(mesh.entity);
    }
    engine.destroy(mesh.entity);
    EntityManager::get().destroy(mesh.entity);
    mesh.entity.clear();
  }
  if (mesh.vertexBuffer != nullptr) {
    engine.destroy(mesh.vertexBuffer);
    mesh.vertexBuffer = nullptr;
  }
  if (mesh.indexBuffer != nullptr) {
    engine.destroy(mesh.indexBuffer);
    mesh.indexBuffer = nullptr;
  }
  mesh.vertexCount = 0;
  mesh.indexCount = 0;
}

ImGuiOverlay createImGuiOverlay(filament::Engine& engine)
{
  ImGuiOverlay overlay;
  overlay.view = engine.createView();
  overlay.scene = engine.createScene();
  overlay.cameraEntity = EntityManager::get().create();
  overlay.camera = engine.createCamera(overlay.cameraEntity);
  overlay.view->setScene(overlay.scene);
  overlay.view->setCamera(overlay.camera);
  overlay.view->setBlendMode(filament::BlendMode::TRANSLUCENT);
  overlay.view->setPostProcessingEnabled(false);

  overlay.material = filament::Material::Builder()
                         .package(
                             dart::examples::filament_gui::kImGuiMaterial,
                             dart::examples::filament_gui::kImGuiMaterialSize)
                         .build(engine);
  overlay.materialInstance = overlay.material->createInstance();

  unsigned char* pixels = nullptr;
  int width = 0;
  int height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  std::vector<std::uint8_t> fontPixels(
      pixels, pixels + static_cast<std::size_t>(width) * height * 4);

  overlay.fontTexture = filament::Texture::Builder()
                            .width(static_cast<std::uint32_t>(width))
                            .height(static_cast<std::uint32_t>(height))
                            .levels(1)
                            .sampler(filament::Texture::Sampler::SAMPLER_2D)
                            .format(filament::Texture::InternalFormat::RGBA8)
                            .build(engine);
  overlay.fontTexture->setImage(
      engine,
      0,
      makePixelBufferDescriptor(
          std::move(fontPixels),
          filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE));

  const filament::TextureSampler sampler(
      filament::TextureSampler::MinFilter::LINEAR,
      filament::TextureSampler::MagFilter::LINEAR);
  overlay.materialInstance->setParameter(
      "fontTexture", overlay.fontTexture, sampler);
  return overlay;
}

void updateImGuiOverlay(
    filament::Engine& engine,
    ImGuiOverlay& overlay,
    const ImDrawData* drawData,
    std::uint32_t width,
    std::uint32_t height)
{
  overlay.view->setViewport({0, 0, width, height});
  overlay.camera->setProjection(
      filament::Camera::Projection::ORTHO,
      0.0,
      static_cast<double>(width),
      static_cast<double>(height),
      0.0,
      0.01,
      10.0);
  overlay.camera->lookAt({0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0});

  if (drawData == nullptr || drawData->TotalVtxCount <= 0
      || drawData->TotalIdxCount <= 0) {
    destroyOverlayMesh(engine, overlay.scene, overlay.mesh);
    return;
  }

  const ImVec2 displayPos = drawData->DisplayPos;
  std::vector<ImGuiVertex> vertices;
  std::vector<std::uint32_t> indices;
  vertices.reserve(static_cast<std::size_t>(drawData->TotalVtxCount));
  indices.reserve(static_cast<std::size_t>(drawData->TotalIdxCount));

  for (int listIndex = 0; listIndex < drawData->CmdListsCount; ++listIndex) {
    const ImDrawList* commandList = drawData->CmdLists[listIndex];
    const std::uint32_t vertexBase
        = static_cast<std::uint32_t>(vertices.size());

    for (const ImDrawVert& vertex : commandList->VtxBuffer) {
      vertices.push_back(
          ImGuiVertex{
              {vertex.pos.x - displayPos.x, vertex.pos.y - displayPos.y, 0.0f},
              {vertex.uv.x, 1.0f - vertex.uv.y},
              vertex.col});
    }

    for (const ImDrawIdx index : commandList->IdxBuffer) {
      indices.push_back(vertexBase + static_cast<std::uint32_t>(index));
    }
  }

  const std::size_t vertexCount = vertices.size();
  const std::size_t indexCount = indices.size();
  if (!overlay.mesh.entity || overlay.mesh.vertexCount != vertexCount
      || overlay.mesh.indexCount != indexCount) {
    destroyOverlayMesh(engine, overlay.scene, overlay.mesh);

    overlay.mesh.vertexBuffer
        = filament::VertexBuffer::Builder()
              .vertexCount(vertexCount)
              .bufferCount(1)
              .attribute(
                  filament::VertexAttribute::POSITION,
                  0,
                  filament::VertexBuffer::AttributeType::FLOAT3,
                  offsetof(ImGuiVertex, position),
                  sizeof(ImGuiVertex))
              .attribute(
                  filament::VertexAttribute::UV0,
                  0,
                  filament::VertexBuffer::AttributeType::FLOAT2,
                  offsetof(ImGuiVertex, uv),
                  sizeof(ImGuiVertex))
              .attribute(
                  filament::VertexAttribute::COLOR,
                  0,
                  filament::VertexBuffer::AttributeType::UBYTE4,
                  offsetof(ImGuiVertex, color),
                  sizeof(ImGuiVertex))
              .normalized(filament::VertexAttribute::COLOR)
              .build(engine);

    overlay.mesh.indexBuffer
        = filament::IndexBuffer::Builder()
              .indexCount(indexCount)
              .bufferType(filament::IndexBuffer::IndexType::UINT)
              .build(engine);

    overlay.mesh.entity = EntityManager::get().create();
    filament::RenderableManager::Builder(1)
        .boundingBox(
            {{0.0f, 0.0f, -1.0f},
             {static_cast<float>(width), static_cast<float>(height), 1.0f}})
        .material(0, overlay.materialInstance)
        .geometry(
            0,
            filament::RenderableManager::PrimitiveType::TRIANGLES,
            overlay.mesh.vertexBuffer,
            overlay.mesh.indexBuffer,
            0,
            indexCount)
        .culling(false)
        .castShadows(false)
        .receiveShadows(false)
        .build(engine, overlay.mesh.entity);
    overlay.scene->addEntity(overlay.mesh.entity);
    overlay.mesh.vertexCount = vertexCount;
    overlay.mesh.indexCount = indexCount;
  }

  overlay.mesh.vertexBuffer->setBufferAt(
      engine, 0, makeBufferDescriptor(std::move(vertices)));
  overlay.mesh.indexBuffer->setBuffer(
      engine, makeBufferDescriptor(std::move(indices)));
}

void destroyImGuiOverlay(filament::Engine& engine, ImGuiOverlay& overlay)
{
  destroyOverlayMesh(engine, overlay.scene, overlay.mesh);
  if (overlay.materialInstance != nullptr) {
    engine.destroy(overlay.materialInstance);
    overlay.materialInstance = nullptr;
  }
  if (overlay.fontTexture != nullptr) {
    engine.destroy(overlay.fontTexture);
    overlay.fontTexture = nullptr;
  }
  if (overlay.material != nullptr) {
    engine.destroy(overlay.material);
    overlay.material = nullptr;
  }
  if (overlay.camera != nullptr) {
    engine.destroyCameraComponent(overlay.cameraEntity);
    overlay.camera = nullptr;
  }
  if (overlay.cameraEntity) {
    EntityManager::get().destroy(overlay.cameraEntity);
    overlay.cameraEntity.clear();
  }
  if (overlay.view != nullptr) {
    engine.destroy(overlay.view);
    overlay.view = nullptr;
  }
  if (overlay.scene != nullptr) {
    engine.destroy(overlay.scene);
    overlay.scene = nullptr;
  }
}

void saveScreenshot(const ScreenshotCapture& capture, const std::string& path)
{
  std::string error;
  if (!writeRgbaPpm(
          path,
          capture.width,
          capture.height,
          capture.pixels,
          false,
          &error)) {
    std::cerr << error << "\n";
    std::exit(1);
  }
}

void requestScreenshot(
    filament::Renderer& renderer,
    ScreenshotCapture& capture,
    std::uint32_t width,
    std::uint32_t height)
{
  {
    std::lock_guard<std::mutex> lock(capture.mutex);
    capture.width = width;
    capture.height = height;
    capture.done = false;
    capture.pixels.assign(
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4,
        0);
  }

  filament::backend::PixelBufferDescriptor pixels(
      capture.pixels.data(),
      capture.pixels.size(),
      filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
      filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
      1,
      0,
      0,
      width,
      [](void*, std::size_t, void* user) {
        auto* capture = static_cast<ScreenshotCapture*>(user);
        {
          std::lock_guard<std::mutex> lock(capture->mutex);
          capture->done = true;
        }
        capture->condition.notify_one();
      },
      &capture);
  renderer.readPixels(0, 0, width, height, std::move(pixels));
}

bool waitForScreenshot(filament::Engine& engine, ScreenshotCapture& capture)
{
  using namespace std::chrono_literals;
  engine.flushAndWait();

  std::unique_lock<std::mutex> lock(capture.mutex);
  return capture.done || capture.condition.wait_for(lock, 5s, [&capture] {
    return capture.done;
  });
}

} // namespace

int main(int argc, char* argv[])
{
  const AppOptions appOptions = parseOptions(argc, argv);
  const RunOptions& options = appOptions.run;

  GLFWwindow* window = nullptr;
#if defined(__linux__)
  if (!options.headless) {
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
  }
#endif
  if (!options.headless) {
    if (!glfwInit()) {
      std::cerr << "Failed to initialize GLFW\n";
      return 1;
    }

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = glfwCreateWindow(
        options.width,
        options.height,
        "DART + Filament (experimental)",
        nullptr,
        nullptr);
    if (window == nullptr) {
      std::cerr << "Failed to create GLFW window\n";
      glfwTerminate();
      return 1;
    }
  }

  CameraController cameraController;
  cameraController.camera = initialCameraForScene(appOptions.scene);
  if (window != nullptr) {
    glfwSetWindowUserPointer(window, &cameraController);
    glfwSetScrollCallback(window, handleScroll);
  }

  auto* engine = filament::Engine::create(filament::Engine::Backend::OPENGL);
  auto* renderer = engine->createRenderer();
  auto* swapChain = options.headless
                        ? engine->createSwapChain(
                              static_cast<std::uint32_t>(options.width),
                              static_cast<std::uint32_t>(options.height))
                        : engine->createSwapChain(getNativeWindow(window));
  auto* view = engine->createView();
  auto* scene = engine->createScene();
  auto cameraEntity = EntityManager::get().create();
  auto* camera = engine->createCamera(cameraEntity);
  view->setScene(scene);
  view->setCamera(camera);
  auto* colorGrading = createDebugColorGrading(*engine);
  view->setColorGrading(colorGrading);
  view->setShadowingEnabled(true);
  view->setShadowType(
      options.headless ? filament::ShadowType::PCF : filament::ShadowType::PCSS);
  filament::SoftShadowOptions softShadowOptions;
  softShadowOptions.penumbraScale = 2.2f;
  softShadowOptions.penumbraRatioScale = 2.8f;
  view->setSoftShadowOptions(softShadowOptions);
  auto* indirectLight = createNeutralIndirectLight(*engine);
  auto* skybox = createNeutralSkybox(*engine);
  scene->setIndirectLight(indirectLight);
  scene->setSkybox(skybox);
  if (!options.headless) {
    filament::RenderQuality renderQuality;
    renderQuality.hdrColorBuffer = filament::QualityLevel::HIGH;
    view->setRenderQuality(renderQuality);
    filament::AmbientOcclusionOptions ambientOcclusionOptions;
    ambientOcclusionOptions.enabled = true;
    ambientOcclusionOptions.aoType = filament::AmbientOcclusionOptions::
        AmbientOcclusionType::GTAO;
    ambientOcclusionOptions.radius = 0.35f;
    ambientOcclusionOptions.intensity = 0.38f;
    ambientOcclusionOptions.quality = filament::QualityLevel::MEDIUM;
    ambientOcclusionOptions.lowPassFilter = filament::QualityLevel::HIGH;
    view->setAmbientOcclusionOptions(ambientOcclusionOptions);
    filament::TemporalAntiAliasingOptions temporalAntiAliasingOptions;
    temporalAntiAliasingOptions.enabled = true;
    temporalAntiAliasingOptions.feedback = 0.10f;
    temporalAntiAliasingOptions.jitterPattern = filament::
        TemporalAntiAliasingOptions::JitterPattern::HALTON_23_X16;
    view->setTemporalAntiAliasingOptions(temporalAntiAliasingOptions);
    filament::MultiSampleAntiAliasingOptions multiSampleAntiAliasingOptions;
    multiSampleAntiAliasingOptions.enabled = true;
    multiSampleAntiAliasingOptions.sampleCount = 4;
    view->setMultiSampleAntiAliasingOptions(multiSampleAntiAliasingOptions);
    view->setAntiAliasing(filament::AntiAliasing::FXAA);
    view->setDithering(filament::Dithering::TEMPORAL);
  }

  auto* material
      = filament::Material::Builder()
            .package(
                dart::examples::filament_gui::kDefaultLitMaterial,
                dart::examples::filament_gui::kDefaultLitMaterialSize)
            .build(*engine);
  auto* texturedMaterial
      = filament::Material::Builder()
            .package(
                dart::examples::filament_gui::kTexturedLitMaterial,
                dart::examples::filament_gui::kTexturedLitMaterialSize)
            .build(*engine);
  auto* transparentMaterial
      = filament::Material::Builder()
            .package(
                dart::examples::filament_gui::kTransparentLitMaterial,
                dart::examples::filament_gui::kTransparentLitMaterialSize)
            .build(*engine);
  auto* transparentTexturedMaterial
      = filament::Material::Builder()
            .package(
                dart::examples::filament_gui::kTransparentTexturedLitMaterial,
                dart::examples::filament_gui::
                    kTransparentTexturedLitMaterialSize)
            .build(*engine);
  auto* debugMaterial
      = filament::Material::Builder()
            .package(
                dart::examples::filament_gui::kDebugColorMaterial,
                dart::examples::filament_gui::kDebugColorMaterialSize)
            .build(*engine);
  auto* checkerTexture = createCheckerTexture(*engine);
  TextureBinding checkerBinding;
  checkerBinding.texture = checkerTexture;
  checkerBinding.sampler = makeRepeatTextureSampler();
  auto* fallbackTexture = createSolidTexture(
      *engine,
      std::array<std::uint8_t, 4>{255, 255, 255, 255},
      TextureColorSpace::Linear);
  TextureBinding fallbackBinding;
  fallbackBinding.texture = fallbackTexture;
  fallbackBinding.sampler = makeRepeatTextureSampler();
  const MaterialSet materials{
      *material,
      *texturedMaterial,
      *transparentMaterial,
      *transparentTexturedMaterial,
      checkerBinding,
      fallbackBinding};
  TextureCache textureCache;

  DartScene dartScene = createDartScene(appOptions.scene);
  const auto initialDescriptors = extractRenderables(*dartScene.world);
  const std::size_t wamDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kWamFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t atlasDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kAtlasFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t atlasRobotDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kAtlasRobotFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t pbrEnvironmentDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t dragAndDropFrameDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return !descriptor.shapeFrameName.empty()
                   && descriptor.material.visible;
          }));
  if (appOptions.scene == ExampleScene::Mvp) {
    if (wamDescriptorCount < 5) {
      std::cerr << "Expected the WAM robot fixture to provide at least five "
                   "visible mesh renderables, but extracted "
                << wamDescriptorCount << "\n";
      return 1;
    }
    if (atlasDescriptorCount != 1) {
      std::cerr << "Expected the Atlas mesh fixture to provide one visible mesh "
                   "renderable, but extracted "
                << atlasDescriptorCount << "\n";
      return 1;
    }
    if (atlasRobotDescriptorCount < 20) {
      std::cerr << "Expected the Atlas robot fixture to provide at least twenty "
                   "visible mesh renderables, but extracted "
                << atlasRobotDescriptorCount << "\n";
      return 1;
    }
    if (pbrEnvironmentDescriptorCount < kMinPbrEnvironmentRenderableCount) {
      std::cerr << "Expected the PBR environment fixture to provide at least "
                << kMinPbrEnvironmentRenderableCount
                << " visible mesh renderables, but extracted "
                << pbrEnvironmentDescriptorCount << "\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::DragAndDrop
             && dragAndDropFrameDescriptorCount
                    < kMinDragAndDropFrameRenderableCount) {
    std::cerr << "Expected the drag-and-drop scene to provide at least "
              << kMinDragAndDropFrameRenderableCount
              << " visible frame renderables, but extracted "
              << dragAndDropFrameDescriptorCount << "\n";
    return 1;
  }

  std::vector<SceneRenderable> sceneRenderables;
  std::size_t createdWamRenderableCount = 0;
  std::size_t createdAtlasRenderableCount = 0;
  std::size_t createdAtlasRobotRenderableCount = 0;
  std::size_t createdPbrEnvironmentRenderableCount = 0;
  std::size_t createdDragAndDropFrameRenderableCount = 0;
  for (const RenderableDescriptor& descriptor : initialDescriptors) {
    if (!descriptor.material.visible) {
      continue;
    }

    auto renderable = createRenderableFromDescriptor(
        *engine, materials, textureCache, descriptor);
    if (!renderable) {
      continue;
    }
    if (descriptor.skeletonName == kWamFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdWamRenderableCount;
    }
    if (descriptor.skeletonName == kAtlasFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdAtlasRenderableCount;
    }
    if (descriptor.skeletonName == kAtlasRobotFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdAtlasRobotRenderableCount;
    }
    if (descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdPbrEnvironmentRenderableCount;
    }
    if (!descriptor.shapeFrameName.empty()) {
      ++createdDragAndDropFrameRenderableCount;
    }

    SceneRenderable sceneRenderable;
    sceneRenderable.id = descriptor.id;
    sceneRenderable.renderable = *renderable;
    scene->addEntity(sceneRenderable.renderable.entity);
    engine->getTransformManager().setTransform(
        engine->getTransformManager().getInstance(
            sceneRenderable.renderable.entity),
        toFilamentTransform(descriptor.worldTransform));
    sceneRenderables.push_back(sceneRenderable);
  }
  if (sceneRenderables.empty()) {
    std::cerr << "No supported visible DART renderables were extracted\n";
    return 1;
  }
  if (appOptions.scene == ExampleScene::Mvp) {
    if (createdWamRenderableCount < wamDescriptorCount) {
      std::cerr << "Only " << createdWamRenderableCount << " of "
                << wamDescriptorCount
                << " WAM robot mesh renderables were created\n";
      return 1;
    }
    if (createdAtlasRenderableCount != atlasDescriptorCount) {
      std::cerr << "Only " << createdAtlasRenderableCount << " of "
                << atlasDescriptorCount
                << " Atlas mesh renderables were created\n";
      return 1;
    }
    if (createdAtlasRobotRenderableCount < atlasRobotDescriptorCount) {
      std::cerr << "Only " << createdAtlasRobotRenderableCount << " of "
                << atlasRobotDescriptorCount
                << " Atlas robot mesh renderables were created\n";
      return 1;
    }
    if (createdPbrEnvironmentRenderableCount < pbrEnvironmentDescriptorCount) {
      std::cerr << "Only " << createdPbrEnvironmentRenderableCount << " of "
                << pbrEnvironmentDescriptorCount
                << " PBR environment mesh renderables were created\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::DragAndDrop
             && createdDragAndDropFrameRenderableCount
                    < dragAndDropFrameDescriptorCount) {
    std::cerr << "Only " << createdDragAndDropFrameRenderableCount << " of "
              << dragAndDropFrameDescriptorCount
              << " drag-and-drop frame renderables were created\n";
    return 1;
  }

  DebugDrawOptions staticDebugOptions;
  staticDebugOptions.drawBodyFrames = true;
  staticDebugOptions.drawCentersOfMass = true;
  staticDebugOptions.drawContacts = false;
  std::optional<Renderable> debugOverlay;
  auto refreshDebugOverlay = [&]() {
    if (debugOverlay) {
      scene->remove(debugOverlay->entity);
      destroyRenderable(*engine, *debugOverlay);
      debugOverlay.reset();
    }

    debugOverlay = createDebugLineRenderable(
        *engine,
        *debugMaterial,
        extractDebugLines(*dartScene.world, staticDebugOptions));
    if (debugOverlay) {
      scene->addEntity(debugOverlay->entity);
    }
  };
  refreshDebugOverlay();
  if (!debugOverlay) {
    std::cerr << "No debug overlay lines were extracted\n";
    return 1;
  }

  DebugDrawOptions contactDebugOptions;
  contactDebugOptions.drawGrid = false;
  contactDebugOptions.drawWorldFrame = false;
  contactDebugOptions.drawBodyFrames = false;
  contactDebugOptions.drawCentersOfMass = false;
  std::optional<Renderable> contactDebugOverlay;
  auto refreshContactDebugOverlay = [&]() {
    if (contactDebugOverlay) {
      scene->remove(contactDebugOverlay->entity);
      destroyRenderable(*engine, *contactDebugOverlay);
      contactDebugOverlay.reset();
    }

    contactDebugOverlay = createDebugLineRenderable(
        *engine,
        *debugMaterial,
        extractContactDebugLines(
            dartScene.world->getLastCollisionResult(), contactDebugOptions));
    if (contactDebugOverlay) {
      scene->addEntity(contactDebugOverlay->entity);
    }
  };

  std::optional<Renderable> selectionDebugOverlay;
  auto refreshSelectionDebugOverlay
      = [&](const std::vector<RenderableDescriptor>& descriptors,
            RenderableId selectedRenderableId) {
          if (selectionDebugOverlay) {
            scene->remove(selectionDebugOverlay->entity);
            destroyRenderable(*engine, *selectionDebugOverlay);
            selectionDebugOverlay.reset();
          }

          if (selectedRenderableId == 0) {
            return;
          }

          const auto selectedDescriptor = std::find_if(
              descriptors.begin(),
              descriptors.end(),
              [&](const RenderableDescriptor& candidate) {
                return candidate.id == selectedRenderableId;
              });
          if (selectedDescriptor == descriptors.end()
              || !selectedDescriptor->material.visible) {
            return;
          }

          selectionDebugOverlay = createDebugLineRenderable(
              *engine,
              *debugMaterial,
              makeSelectionDebugLines(*selectedDescriptor));
          if (selectionDebugOverlay) {
            scene->addEntity(selectionDebugOverlay->entity);
          }
        };

  auto lightEntity = EntityManager::get().create();
  filament::LightManager::ShadowOptions shadowOptions;
  shadowOptions.mapSize = 2048;
  shadowOptions.shadowCascades = 4;
  shadowOptions.cascadeSplitPositions[0] = 0.08f;
  shadowOptions.cascadeSplitPositions[1] = 0.22f;
  shadowOptions.cascadeSplitPositions[2] = 0.55f;
  shadowOptions.shadowFar = 20.0f;
  shadowOptions.shadowFarHint = 10.0f;
  shadowOptions.screenSpaceContactShadows = !options.headless;
  shadowOptions.maxShadowDistance = 0.8f;
  shadowOptions.shadowBulbRadius = 0.16f;
  filament::LightManager::Builder(filament::LightManager::Type::SUN)
      .color({1.0f, 0.96f, 0.88f})
      .intensity(82000.0f)
      .direction({-0.30f, -0.42f, -1.0f})
      .castShadows(true)
      .shadowOptions(shadowOptions)
      .build(*engine, lightEntity);
  scene->addEntity(lightEntity);

  auto fillLightEntity = EntityManager::get().create();
  filament::LightManager::Builder(filament::LightManager::Type::DIRECTIONAL)
      .color({0.80f, 0.88f, 1.0f})
      .intensity(62000.0f)
      .direction({0.42f, 0.18f, -0.7f})
      .castShadows(false)
      .build(*engine, fillLightEntity);
  scene->addEntity(fillLightEntity);

  auto rimLightEntity = EntityManager::get().create();
  filament::LightManager::Builder(filament::LightManager::Type::DIRECTIONAL)
      .color({0.88f, 0.93f, 1.0f})
      .intensity(42000.0f)
      .direction({-0.65f, 0.40f, -0.45f})
      .castShadows(false)
      .build(*engine, rimLightEntity);
  scene->addEntity(rimLightEntity);

  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  auto& imguiStyle = ImGui::GetStyle();
  imguiStyle.WindowRounding = 4.0f;
  imguiStyle.Colors[ImGuiCol_WindowBg].w = 0.72f;
  auto& imguiIo = ImGui::GetIO();
  loadImGuiFont(imguiIo);
  imguiIo.Fonts->Build();
  ImGuiOverlay imguiOverlay = createImGuiOverlay(*engine);

  ViewerLifecycleState lifecycle;
  bool wasSpacePressed = false;
  bool wasStepPressed = false;
  bool wasLeftMousePressed = false;
  bool leftMouseStartedOnPanel = false;
  bool leftMouseStartedDrag = false;
  double leftMousePressX = 0.0;
  double leftMousePressY = 0.0;
  PickRay selectedDragLastRay;
  Eigen::Vector3d selectedDragPlanePoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d selectedDragPlaneNormal = Eigen::Vector3d::UnitX();
  RenderableId selectedRenderableId = 0;
  std::string selectedLabel = "none";
  bool screenshotSucceeded = options.screenshotPath.empty();
  ScreenshotCapture screenshotCapture;
  bool renderUnavailable = false;

  while (options.headless || !glfwWindowShouldClose(window)) {
    if (window != nullptr) {
      glfwPollEvents();
      if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
      }
      const bool isSpacePressed
          = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
      if (isSpacePressed && !wasSpacePressed) {
        togglePaused(lifecycle);
      }
      wasSpacePressed = isSpacePressed;

      const bool isStepPressed = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
      if (isStepPressed && !wasStepPressed) {
        requestSingleStep(lifecycle, false);
      }
      wasStepPressed = isStepPressed;
    }

    int width = options.width;
    int height = options.height;
    if (window != nullptr) {
      glfwGetFramebufferSize(window, &width, &height);
    }
    width = std::max(1, width);
    height = std::max(1, height);
    imguiIo.DisplaySize
        = ImVec2(static_cast<float>(width), static_cast<float>(height));
    imguiIo.DeltaTime = static_cast<float>(dartScene.world->getTimeStep());
    view->setViewport(
        {0,
         0,
         static_cast<std::uint32_t>(width),
         static_cast<std::uint32_t>(height)});
    const double aspect
        = static_cast<double>(width) / static_cast<double>(height);
    camera->setProjection(
        45.0, aspect, 0.05, 100.0, filament::Camera::Fov::VERTICAL);
    if (window != nullptr) {
      updateCameraController(window, cameraController, leftMouseStartedDrag);
    }
    const Eigen::Vector3d eye = cameraEye(cameraController.camera);
    camera->lookAt(
        {eye.x(), eye.y(), eye.z()},
        {cameraController.camera.target.x(),
         cameraController.camera.target.y(),
         cameraController.camera.target.z()},
        {0.0, 0.0, 1.0});

    if (shouldAdvanceSimulation(lifecycle)) {
      dartScene.world->step();
      markSimulationAdvanced(lifecycle);
      refreshContactDebugOverlay();
    }

    auto& transforms = engine->getTransformManager();
    auto descriptors = extractRenderables(*dartScene.world);
    if (window != nullptr && selectedRenderableId != 0) {
      const Eigen::Vector3d nudge = selectedNudgeFromKeyboard(
          window, cameraController.camera, 0.035);
      if (nudge.squaredNorm() > 0.0) {
        const auto selectedDescriptor = std::find_if(
            descriptors.begin(),
            descriptors.end(),
            [&](const RenderableDescriptor& candidate) {
              return candidate.id == selectedRenderableId;
            });
        if (selectedDescriptor != descriptors.end()
            && translateFrameRenderable(*selectedDescriptor, nudge)) {
          lifecycle.paused = true;
          descriptors = extractRenderables(*dartScene.world);
        }
      }
    }
    bool selectedRenderableStillVisible = selectedRenderableId == 0;
    for (SceneRenderable& sceneRenderable : sceneRenderables) {
      const auto descriptor = std::find_if(
          descriptors.begin(),
          descriptors.end(),
          [&](const RenderableDescriptor& candidate) {
            return candidate.id == sceneRenderable.id;
          });
      if (descriptor == descriptors.end() || !descriptor->material.visible) {
        continue;
      }

      transforms.setTransform(
          transforms.getInstance(sceneRenderable.renderable.entity),
          toFilamentTransform(descriptor->worldTransform));
      const bool isSelected = descriptor->id == selectedRenderableId;
      if (isSelected) {
        selectedRenderableStillVisible = true;
      }
      updateRenderableSelection(
          sceneRenderable.renderable,
          toRgba(descriptor->material.rgba),
          isSelected);
    }
    if (!selectedRenderableStillVisible) {
      selectedRenderableId = 0;
      selectedLabel = "none";
    }

    if (window != nullptr) {
      double cursorX = 0.0;
      double cursorY = 0.0;
      glfwGetCursorPos(window, &cursorX, &cursorY);
      const bool isLeftMousePressed
          = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
      if (isLeftMousePressed && !wasLeftMousePressed) {
        leftMousePressX = cursorX;
        leftMousePressY = cursorY;
        leftMouseStartedDrag = false;
        leftMouseStartedOnPanel
            = cursorX >= 20.0 && cursorX <= 360.0 && cursorY >= 20.0
              && cursorY <= 285.0;

        if (!leftMouseStartedOnPanel && selectedRenderableId != 0
            && isDragModifierDown(window)) {
          const auto selectedDescriptor = std::find_if(
              descriptors.begin(),
              descriptors.end(),
              [&](const RenderableDescriptor& candidate) {
                return candidate.id == selectedRenderableId;
              });
          if (selectedDescriptor != descriptors.end()) {
            const auto basis = makeOrbitCameraBasis(cameraController.camera);
            const PickRay ray = makePerspectivePickRay(
                cameraController.camera, cursorX, cursorY, width, height);
            const Eigen::Vector3d planePoint
                = selectedDescriptor->worldTransform.translation();
            const Eigen::Vector3d planeNormal = basis.forward;
            if (intersectPlane(ray, planePoint, planeNormal)) {
              leftMouseStartedDrag = true;
              selectedDragLastRay = ray;
              selectedDragPlanePoint = planePoint;
              selectedDragPlaneNormal = planeNormal;
              lifecycle.paused = true;
            }
          }
        }
      }
      if (isLeftMousePressed && leftMouseStartedDrag) {
        const PickRay ray = makePerspectivePickRay(
            cameraController.camera, cursorX, cursorY, width, height);
        const auto translation = computePlaneDragTranslation(
            selectedDragLastRay,
            ray,
            selectedDragPlanePoint,
            selectedDragPlaneNormal);
        if (translation && translation->squaredNorm() > 1e-12) {
          const auto selectedDescriptor = std::find_if(
              descriptors.begin(),
              descriptors.end(),
              [&](const RenderableDescriptor& candidate) {
                return candidate.id == selectedRenderableId;
              });
          if (selectedDescriptor != descriptors.end()
              && translateFrameRenderable(*selectedDescriptor, *translation)) {
            selectedDragLastRay = ray;
            lifecycle.paused = true;
            descriptors = extractRenderables(*dartScene.world);
          }
        }
      }
      if (!isLeftMousePressed && wasLeftMousePressed) {
        const double dragDistance = std::hypot(
            cursorX - leftMousePressX, cursorY - leftMousePressY);
        if (!leftMouseStartedOnPanel && !leftMouseStartedDrag
            && dragDistance < 4.0) {
          const auto hit = pickNearestRenderable(
              descriptors,
              makePerspectivePickRay(
                  cameraController.camera, cursorX, cursorY, width, height));
          if (hit) {
            selectedRenderableId = hit->id;
            const RenderableDescriptor& descriptor
                = descriptors[hit->renderableIndex];
            selectedLabel = descriptor.skeletonName.empty()
                                ? descriptor.shapeFrameName
                                : descriptor.skeletonName + "/"
                                      + descriptor.bodyName;
            if (!descriptor.shapeNodeName.empty()) {
              selectedLabel += "/" + descriptor.shapeNodeName;
            }
            selectedLabel += " (" + descriptor.geometry.shapeType + ")";
          } else {
            selectedRenderableId = 0;
            selectedLabel = "none";
          }
        }
        leftMouseStartedDrag = false;
      }
      wasLeftMousePressed = isLeftMousePressed;
    }
    refreshSelectionDebugOverlay(descriptors, selectedRenderableId);

    if (appOptions.showUi) {
      ImGui::NewFrame();
      ImGui::SetNextWindowPos({20.0f, 20.0f}, ImGuiCond_Always);
      ImGui::SetNextWindowBgAlpha(0.72f);
      ImGui::Begin(
          "DART",
          nullptr,
          ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
      ImGui::Text("scene: %s", sceneName(appOptions.scene));
      ImGui::Text("time: %.3f", dartScene.world->getTime());
      ImGui::Text(
          "contacts: %zu",
          dartScene.world->getLastCollisionResult().getNumContacts());
      ImGui::TextWrapped("selected: %s", selectedLabel.c_str());
      if (ImGui::Button(lifecycle.paused ? "Resume" : "Pause")) {
        togglePaused(lifecycle);
      }
      ImGui::SameLine();
      if (ImGui::Button("Step")) {
        requestSingleStep(lifecycle);
      }
      bool debugOptionsChanged = false;
      debugOptionsChanged
          |= ImGui::Checkbox("Grid", &staticDebugOptions.drawGrid);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("World", &staticDebugOptions.drawWorldFrame);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Body", &staticDebugOptions.drawBodyFrames);
      debugOptionsChanged
          |= ImGui::Checkbox("COM", &staticDebugOptions.drawCentersOfMass);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Contacts", &contactDebugOptions.drawContacts);
      debugOptionsChanged |= ImGui::Checkbox(
          "Normals", &contactDebugOptions.drawContactNormals);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Forces", &contactDebugOptions.drawContactForces);
      if (debugOptionsChanged) {
        refreshDebugOverlay();
        refreshContactDebugOverlay();
      }
      ImGui::End();
      ImGui::Render();
      updateImGuiOverlay(
          *engine,
          imguiOverlay,
          ImGui::GetDrawData(),
          static_cast<std::uint32_t>(width),
          static_cast<std::uint32_t>(height));
    }

    const bool shouldCaptureScreenshot
        = shouldRequestScreenshot(options, lifecycle);

    bool didRenderFrame = false;
    if (renderer->beginFrame(swapChain)) {
      renderer->render(view);
      if (appOptions.showUi) {
        renderer->render(imguiOverlay.view);
      }
      if (shouldCaptureScreenshot) {
        requestScreenshot(
            *renderer,
            screenshotCapture,
            static_cast<std::uint32_t>(width),
            static_cast<std::uint32_t>(height));
        markScreenshotRequested(lifecycle);
      }
      renderer->endFrame();
      didRenderFrame = true;
    } else {
      markFrameSkipped(lifecycle);
      if (options.headless && lifecycle.skippedFrames > 1000) {
        std::cerr << "No headless Filament frame was available\n";
        renderUnavailable = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (didRenderFrame) {
      markFrameRendered(lifecycle);
      if (shouldStopAfterFrame(options, lifecycle)) {
        break;
      }
    }
  }

  if (!options.screenshotPath.empty() && !lifecycle.screenshotRequested) {
    std::cerr << "No rendered frame was available for screenshot capture\n";
  }
  if (lifecycle.screenshotRequested) {
    screenshotSucceeded = waitForScreenshot(*engine, screenshotCapture);
    if (screenshotSucceeded) {
      saveScreenshot(screenshotCapture, options.screenshotPath);
    } else {
      std::cerr << "Timed out waiting for Filament screenshot readback\n";
    }
  }
  if (options.maxFrames >= 0) {
    std::cout << "Final contacts: "
              << dartScene.world->getLastCollisionResult().getNumContacts()
              << "\n";
  }

  destroyImGuiOverlay(*engine, imguiOverlay);
  ImGui::DestroyContext();

  scene->remove(lightEntity);
  scene->remove(fillLightEntity);
  scene->remove(rimLightEntity);
  scene->setIndirectLight(nullptr);
  scene->setSkybox(nullptr);
  view->setColorGrading(nullptr);
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    scene->remove(sceneRenderable.renderable.entity);
  }
  if (debugOverlay) {
    scene->remove(debugOverlay->entity);
  }
  if (contactDebugOverlay) {
    scene->remove(contactDebugOverlay->entity);
  }
  if (selectionDebugOverlay) {
    scene->remove(selectionDebugOverlay->entity);
  }
  engine->destroy(lightEntity);
  engine->destroy(fillLightEntity);
  engine->destroy(rimLightEntity);
  EntityManager::get().destroy(lightEntity);
  EntityManager::get().destroy(fillLightEntity);
  EntityManager::get().destroy(rimLightEntity);
  engine->destroy(indirectLight);
  engine->destroy(skybox);
  engine->destroy(colorGrading);
  for (SceneRenderable& sceneRenderable : sceneRenderables) {
    destroyRenderable(*engine, sceneRenderable.renderable);
  }
  if (contactDebugOverlay) {
    destroyRenderable(*engine, *contactDebugOverlay);
  }
  if (selectionDebugOverlay) {
    destroyRenderable(*engine, *selectionDebugOverlay);
  }
  if (debugOverlay) {
    destroyRenderable(*engine, *debugOverlay);
  }
  for (auto* texture : textureCache.ownedTextures) {
    engine->destroy(texture);
  }
  engine->destroy(fallbackTexture);
  engine->destroy(checkerTexture);
  engine->destroy(debugMaterial);
  engine->destroy(transparentTexturedMaterial);
  engine->destroy(transparentMaterial);
  engine->destroy(texturedMaterial);
  engine->destroy(material);
  engine->destroyCameraComponent(cameraEntity);
  EntityManager::get().destroy(cameraEntity);
  engine->destroy(view);
  engine->destroy(scene);
  engine->destroy(renderer);
  engine->destroy(swapChain);
  filament::Engine::destroy(&engine);

  if (window != nullptr) {
    glfwDestroyWindow(window);
    glfwTerminate();
  }
  return screenshotSucceeded && !renderUnavailable ? 0 : 1;
}
