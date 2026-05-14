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
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/profile.hpp>
#include <dart/config.hpp>
#include <dart/io/read.hpp>
#include <dart/utils/composite_resource_retriever.hpp>
#include <dart/utils/dart_resource_retriever.hpp>
#include <dart/utils/http_resource_retriever.hpp>
#include <dart/utils/mesh_loader.hpp>
#include <dart/utils/package_resource_retriever.hpp>
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
using dart::dynamics::ConvexMeshShape;
using dart::dynamics::DynamicsAspect;
using dart::dynamics::FreeJoint;
using dart::dynamics::HeightmapShapef;
using dart::dynamics::HeightmapShaped;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;
using dart::dynamics::MeshShape;
using dart::dynamics::PlaneShape;
using dart::dynamics::PointCloudShape;
using dart::dynamics::Shape;
using dart::dynamics::ShapePtr;
using dart::dynamics::ShapeNode;
using dart::dynamics::SimpleFrame;
using dart::dynamics::Skeleton;
using dart::dynamics::SoftBodyNode;
using dart::dynamics::SoftBodyNodeHelper;
using dart::dynamics::SoftMeshShape;
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;

#if DART_HAVE_OCTOMAP
using dart::dynamics::VoxelGridShape;
#endif

using dart::examples::filament_gui::DebugDrawOptions;
using dart::examples::filament_gui::DebugLineDescriptor;
using dart::examples::filament_gui::GeometryDescriptor;
using dart::examples::filament_gui::MeshMaterialDescriptor;
using dart::examples::filament_gui::MeshPartDescriptor;
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
using dart::examples::filament_gui::makeRenderableId;
using dart::examples::filament_gui::makeSelectionDebugLines;
using dart::examples::filament_gui::normalizeRunOptions;
using dart::examples::filament_gui::pickNearestRenderable;
using dart::examples::filament_gui::planRenderableSetUpdate;
using dart::examples::filament_gui::requestSingleStep;
using dart::examples::filament_gui::shouldAdvanceSimulation;
using dart::examples::filament_gui::shouldRequestScreenshot;
using dart::examples::filament_gui::shouldStopAfterFrame;
using dart::examples::filament_gui::togglePaused;
using dart::examples::filament_gui::translateFrameRenderable;
using dart::examples::filament_gui::updateOrbitCamera;
using dart::examples::filament_gui::writeRgbaPpm;
using dart::simulation::World;
using filament::math::float2;
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
  filament::Material& debugColor;
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

struct ProfileAccumulator
{
  using Clock = std::chrono::steady_clock;

  std::size_t frames = 0;
  std::size_t renderedFrames = 0;
  std::size_t skippedFrames = 0;
  std::size_t simulationSteps = 0;
  double frameMs = 0.0;
  double simulatedMs = 0.0;
  double inputMs = 0.0;
  double viewportCameraMs = 0.0;
  double simulationMs = 0.0;
  double contactDebugMs = 0.0;
  double extractionMs = 0.0;
  double syncMs = 0.0;
  double interactionMs = 0.0;
  double selectionDebugMs = 0.0;
  double uiMs = 0.0;
  double beginFrameMs = 0.0;
  double renderMs = 0.0;
  double screenshotWaitMs = 0.0;
  double screenshotSaveMs = 0.0;
  double maxFrameMs = 0.0;
  double maxRenderMs = 0.0;
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

double elapsedMs(ProfileAccumulator::Clock::time_point start)
{
  return std::chrono::duration<double, std::milli>(
             ProfileAccumulator::Clock::now() - start)
      .count();
}

void printProfile(const ProfileAccumulator& profile)
{
  if (profile.frames == 0) {
    return;
  }

  const double frames = static_cast<double>(profile.frames);
  const double renderedFrames
      = static_cast<double>(std::max<std::size_t>(profile.renderedFrames, 1));
  const double averageFrameMs = profile.frameMs / frames;
  const double fps = averageFrameMs > 0.0 ? 1000.0 / averageFrameMs : 0.0;
  const double renderedFps
      = profile.frameMs > 0.0
            ? 1000.0 * static_cast<double>(profile.renderedFrames)
                  / profile.frameMs
            : 0.0;
  const auto avg = [&](double value) { return value / frames; };
  const auto avgRendered = [&](double value) { return value / renderedFrames; };

  std::cout << "Profile frames: " << profile.frames
            << " rendered=" << profile.renderedFrames
            << " skipped=" << profile.skippedFrames << "\n";
  std::cout << "Profile average: frame=" << averageFrameMs
            << " ms loop_fps=" << fps << " rendered_fps=" << renderedFps
            << " max_frame=" << profile.maxFrameMs
            << " ms max_render=" << profile.maxRenderMs << " ms\n";
  std::cout << "Profile phases (ms/frame): input=" << avg(profile.inputMs)
            << " viewport_camera=" << avg(profile.viewportCameraMs)
            << " simulation=" << avg(profile.simulationMs)
            << " contact_debug=" << avg(profile.contactDebugMs)
            << " extraction=" << avg(profile.extractionMs)
            << " sync=" << avg(profile.syncMs)
            << " interaction=" << avg(profile.interactionMs)
            << " selection_debug=" << avg(profile.selectionDebugMs)
            << " ui=" << avg(profile.uiMs)
            << " begin_frame=" << avg(profile.beginFrameMs)
            << " render=" << avg(profile.renderMs) << "\n";
  std::cout << "Profile per rendered frame (ms): elapsed="
            << avgRendered(profile.frameMs)
            << " begin_frame=" << avgRendered(profile.beginFrameMs)
            << " simulation=" << avgRendered(profile.simulationMs)
            << " extraction=" << avgRendered(profile.extractionMs)
            << " sync=" << avgRendered(profile.syncMs)
            << " render=" << avgRendered(profile.renderMs) << "\n";
  const double realTimeFactor
      = profile.frameMs > 0.0 ? profile.simulatedMs / profile.frameMs : 0.0;
  std::cout << "Profile simulation: steps=" << profile.simulationSteps
            << " simulated=" << profile.simulatedMs
            << " ms real_time_factor=" << realTimeFactor << "\n";
  if (profile.screenshotWaitMs > 0.0 || profile.screenshotSaveMs > 0.0) {
    std::cout << "Profile screenshot: wait=" << profile.screenshotWaitMs
              << " ms save=" << profile.screenshotSaveMs << " ms\n";
  }
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

void loadImGuiFont(ImGuiIO& io, float guiScale)
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
  const float fontSize = 15.0f * guiScale;
  for (const auto& path : candidates) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
      continue;
    }
    if (io.Fonts->AddFontFromFileTTF(path.string().c_str(), fontSize, &config)
        != nullptr) {
      return;
    }
  }

  config.SizePixels = 13.0f * guiScale;
  io.Fonts->AddFontDefault(&config);
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

struct G1IkHandle
{
  RenderableId targetRenderableId = 0;
  std::string label;
  int hotkey = 0;
  std::shared_ptr<SimpleFrame> target;
  InverseKinematicsPtr ik;
};

struct DartScene
{
  dart::simulation::WorldPtr world;
  std::vector<G1IkHandle> ikHandles;
};

enum class ExampleScene
{
  Mvp,
  DragAndDrop,
  G1,
};

struct AppOptions
{
  RunOptions run;
  ExampleScene scene = ExampleScene::Mvp;
  bool showUi = true;
  bool showUiExplicit = false;
  bool profile = false;
  bool orbitLight = true;
  double orbitLightPeriodSeconds = 80.0;
  float guiScale = 1.0f;
  std::string g1PackageName = "g1_description";
  std::string g1PackageUri
      = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/"
        "master/robots/g1_description";
  std::string g1RobotUri = "package://g1_description/g1_29dof.urdf";
};

constexpr const char* kWamFixtureSkeletonName = "visual_wam_robot";
constexpr const char* kAtlasFixtureSkeletonName = "visual_atlas_torso_mesh";
constexpr const char* kAtlasRobotFixtureSkeletonName = "visual_atlas_robot";
constexpr const char* kPyramidFixtureSkeletonName = "visual_pyramid";
constexpr const char* kMultiSphereFixtureSkeletonName = "visual_multi_sphere";
constexpr const char* kLineSegmentFixtureSkeletonName = "visual_line_segments";
constexpr const char* kConvexMeshFixtureSkeletonName = "visual_convex_mesh";
constexpr const char* kPointCloudFixtureSkeletonName = "visual_point_cloud";
constexpr const char* kHeightmapFixtureSkeletonName = "visual_heightmap";
constexpr const char* kSoftMeshFixtureSkeletonName = "visual_soft_mesh";
constexpr const char* kVoxelGridFixtureSkeletonName = "visual_voxel_grid";
constexpr const char* kPbrEnvironmentFixtureSkeletonName =
    "visual_pbr_environment";
constexpr const char* kG1FixtureSkeletonName = "visual_g1_robot";
constexpr std::size_t kMinPbrEnvironmentRenderableCount = 4;
constexpr std::size_t kMinDragAndDropFrameRenderableCount = 5;
constexpr std::size_t kMinG1RenderableCount = 20;

const char* sceneName(ExampleScene scene)
{
  switch (scene) {
    case ExampleScene::Mvp:
      return "mvp";
    case ExampleScene::DragAndDrop:
      return "drag-and-drop";
    case ExampleScene::G1:
      return "g1";
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
  if (name == "g1") {
    scene = ExampleScene::G1;
    return true;
  }
  return false;
}

OrbitCamera initialCameraForScene(ExampleScene scene)
{
  OrbitCamera camera;
  switch (scene) {
    case ExampleScene::DragAndDrop:
      camera.target = Eigen::Vector3d(0.35, 0.15, 0.9);
      camera.yaw = -0.72;
      camera.pitch = 0.58;
      camera.distance = 9.5;
      break;
    case ExampleScene::G1:
      camera.target = Eigen::Vector3d(0.0, 0.0, 0.85);
      camera.yaw = -0.78;
      camera.pitch = 0.24;
      camera.distance = 3.4;
      break;
    case ExampleScene::Mvp:
      camera.target = Eigen::Vector3d(0.15, 0.55, 0.75);
      camera.yaw = -0.95;
      camera.pitch = 0.38;
      camera.distance = 7.2;
      break;
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

void disableSkeletonCollisionAndGravity(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return;
  }

  skeleton->disableSelfCollisionCheck();
  skeleton->setAdjacentBodyCheck(false);
  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->setCollidable(false);
    body->setGravityMode(false);
    body->eachShapeNodeWith<CollisionAspect>([](ShapeNode* shapeNode) {
      shapeNode->getCollisionAspect()->setCollidable(false);
    });
  }
}

void makeVisualOnlySkeleton(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return;
  }

  skeleton->setMobile(false);
  disableSkeletonCollisionAndGravity(skeleton);
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

  makeVisualOnlySkeleton(wam);
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

  makeVisualOnlySkeleton(atlas);
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

std::optional<std::string> getLastPathSegment(std::string value)
{
  if (value.empty()) {
    return std::nullopt;
  }

  const std::size_t terminator = value.find_first_of("?#");
  if (terminator != std::string::npos) {
    value.erase(terminator);
  }

  while (value.ends_with('/') || value.ends_with('\\')) {
    value.pop_back();
  }

  if (value.empty()) {
    return std::nullopt;
  }

  const std::size_t slash = value.find_last_of("/\\");
  std::string segment
      = value.substr(slash == std::string::npos ? 0 : slash + 1);

  if (segment.empty()) {
    return std::nullopt;
  }

  return segment;
}

std::optional<std::string> inferPackageNameFromRobotUri(
    const std::string& robotUri)
{
  if (robotUri.empty()) {
    return std::nullopt;
  }

  dart::common::Uri uri;
  if (!uri.fromStringOrPath(robotUri)) {
    return std::nullopt;
  }

  if (!uri.mScheme || *uri.mScheme != "package" || !uri.mAuthority) {
    return std::nullopt;
  }

  return uri.mAuthority.get();
}

std::optional<std::string> inferPackageNameFromPackageUri(
    const std::string& packageUri)
{
  if (packageUri.empty()) {
    return std::nullopt;
  }

  dart::common::Uri uri;
  if (uri.fromStringOrPath(packageUri)) {
    if (uri.mScheme && *uri.mScheme == "package" && uri.mAuthority) {
      return uri.mAuthority.get();
    }

    if (uri.mPath) {
      if (auto segment = getLastPathSegment(uri.mPath.get())) {
        return segment;
      }
    }
  }

  return getLastPathSegment(packageUri);
}

dart::common::ResourceRetrieverPtr createG1ResourceRetriever(
    const AppOptions& options)
{
  auto local = std::make_shared<dart::common::LocalResourceRetriever>();
  auto dartRetriever = std::make_shared<dart::utils::DartResourceRetriever>();
  auto http = std::make_shared<dart::utils::HttpResourceRetriever>();

  auto passthrough = std::make_shared<dart::utils::CompositeResourceRetriever>();
  passthrough->addSchemaRetriever("file", local);
  passthrough->addSchemaRetriever("dart", dartRetriever);
  passthrough->addSchemaRetriever("http", http);
  passthrough->addSchemaRetriever("https", http);
  passthrough->addDefaultRetriever(local);

  auto packageRetriever
      = std::make_shared<dart::utils::PackageResourceRetriever>(passthrough);
  packageRetriever->addPackageDirectory(
      options.g1PackageName, options.g1PackageUri);

  auto resolver = std::make_shared<dart::utils::CompositeResourceRetriever>();
  resolver->addSchemaRetriever("package", packageRetriever);
  resolver->addSchemaRetriever("file", local);
  resolver->addSchemaRetriever("dart", dartRetriever);
  resolver->addSchemaRetriever("http", http);
  resolver->addSchemaRetriever("https", http);
  resolver->addDefaultRetriever(local);

  return resolver;
}

dart::dynamics::SkeletonPtr createG1Ground()
{
  auto ground = Skeleton::create("g1_ground");
  auto* body = ground->createJointAndBodyNodePair<WeldJoint>().second;

  constexpr double thickness = 0.04;
  auto* shapeNode = body->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d(4.0, 4.0, thickness)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -thickness / 2.0));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.86, 0.88, 0.9, 1.0));
  return ground;
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (!skeleton) {
    return std::nullopt;
  }

  bool hasBounds = false;
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();

  const auto includePoint = [&](const Eigen::Vector3d& point) {
    if (!hasBounds) {
      min = point;
      max = point;
      hasBounds = true;
      return;
    }
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  };

  for (std::size_t i = 0; i < skeleton->getNumBodyNodes(); ++i) {
    auto* body = skeleton->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    body->eachShapeNodeWith<VisualAspect>([&](const ShapeNode* shapeNode) {
      if (shapeNode == nullptr || shapeNode->getShape() == nullptr
          || shapeNode->getVisualAspect()->isHidden()) {
        return;
      }

      const auto& bounds = shapeNode->getShape()->getBoundingBox();
      const Eigen::Vector3d localMin = bounds.getMin();
      const Eigen::Vector3d localMax = bounds.getMax();
      if (!localMin.allFinite() || !localMax.allFinite()) {
        return;
      }

      const Eigen::Isometry3d transform = shapeNode->getWorldTransform();
      for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
          for (int z = 0; z < 2; ++z) {
            includePoint(transform * Eigen::Vector3d(
                                         x == 0 ? localMin.x() : localMax.x(),
                                         y == 0 ? localMin.y() : localMax.y(),
                                         z == 0 ? localMin.z() : localMax.z()));
          }
        }
      }
    });
  }

  if (!hasBounds) {
    return std::nullopt;
  }

  return std::make_pair(min, max);
}

dart::dynamics::SkeletonPtr loadG1Skeleton(const AppOptions& options)
{
  dart::io::ReadOptions readOptions;
  readOptions.resourceRetriever = createG1ResourceRetriever(options);
  const dart::common::Uri robotUri(options.g1RobotUri);
  auto robot = dart::io::readSkeleton(robotUri, readOptions);
  if (!robot) {
    throw std::runtime_error(
        "Failed to load G1 robot fixture from " + options.g1RobotUri);
  }

  robot->setName(kG1FixtureSkeletonName);
  if (auto* rootBody = robot->getRootBodyNode()) {
    if (auto* freeJoint
        = dynamic_cast<FreeJoint*>(rootBody->getParentJoint())) {
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      FreeJoint::setTransformOf(freeJoint, transform);
      if (const auto bounds = computeVisualWorldBounds(robot)) {
        constexpr double g1GroundClearance = 0.015;
        transform.translation().z() = g1GroundClearance - bounds->first.z();
        FreeJoint::setTransformOf(freeJoint, transform);
      }
    }
  }
  disableSkeletonCollisionAndGravity(robot);
  return robot;
}

dart::math::SupportGeometry makeG1FootSupportGeometry()
{
  dart::math::SupportGeometry geometry;
  geometry.emplace_back(Eigen::Vector3d(0.12, 0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, -0.06, 0.0));
  geometry.emplace_back(Eigen::Vector3d(-0.12, 0.06, 0.0));
  return geometry;
}

void addG1IkTargets(
    DartScene& scene, const dart::dynamics::SkeletonPtr& robot)
{
  struct Config
  {
    const char* bodyNode;
    const char* targetName;
    const char* label;
    int hotkey;
    Eigen::Vector4d color;
    bool supportContact;
  };

  const auto footSupportGeometry = makeG1FootSupportGeometry();
  const std::array<Config, 4> configs{{
      {"left_rubber_hand",
       "ik_target_left_hand",
       "1 left hand",
       GLFW_KEY_1,
       {0.18, 0.55, 1.0, 0.92},
       false},
      {"right_rubber_hand",
       "ik_target_right_hand",
       "2 right hand",
       GLFW_KEY_2,
       {1.0, 0.40, 0.24, 0.92},
       false},
      {"left_ankle_roll_link",
       "ik_target_left_foot",
       "3 left foot",
       GLFW_KEY_3,
       {0.26, 0.86, 0.34, 0.92},
       true},
      {"right_ankle_roll_link",
       "ik_target_right_foot",
       "4 right foot",
       GLFW_KEY_4,
       {0.95, 0.72, 0.18, 0.92},
       true},
  }};

  if (!scene.world || !robot) {
    return;
  }

  for (const Config& config : configs) {
    auto* bodyNode = robot->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      std::cerr << "Unable to find G1 body node '" << config.bodyNode
                << "' for IK target.\n";
      continue;
    }

    auto* endEffector = bodyNode->createEndEffector(
        std::string(config.targetName) + "_effector");
    if (config.supportContact) {
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    }

    auto ik = endEffector->getIK(true);
    ik->setGradientMethod<InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);

    auto target = SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    target->setShape(std::make_shared<SphereShape>(0.055));
    target->getVisualAspect(true)->setRGBA(config.color);
    scene.world->addSimpleFrame(target);
    ik->setTarget(target);

    G1IkHandle handle;
    handle.targetRenderableId = makeRenderableId(*target);
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = std::move(target);
    handle.ik = std::move(ik);
    scene.ikHandles.push_back(std::move(handle));
  }
}

AppOptions parseOptions(int argc, char* argv[])
{
  AppOptions options;
  bool g1PackageNameExplicit = false;
  bool g1PackageUriExplicit = false;
  bool g1RobotUriExplicit = false;

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
    } else if (arg == "--orbit-light") {
      options.orbitLight = true;
    } else if (arg == "--no-orbit-light") {
      options.orbitLight = false;
    } else if (arg == "--orbit-light-period" && i + 1 < argc) {
      char* end = nullptr;
      const char* value = argv[++i];
      const double orbitLightPeriodSeconds = std::strtod(value, &end);
      if (end == value || *end != '\0' || !std::isfinite(orbitLightPeriodSeconds)
          || orbitLightPeriodSeconds <= 0.0) {
        std::cerr << "Invalid --orbit-light-period value '" << value
                  << "'. Expected a positive number of seconds.\n";
        std::exit(2);
      }
      options.orbitLightPeriodSeconds = orbitLightPeriodSeconds;
    } else if (arg == "--gui-scale" && i + 1 < argc) {
      char* end = nullptr;
      const char* value = argv[++i];
      const float guiScale = std::strtof(value, &end);
      if (end == value || *end != '\0' || !std::isfinite(guiScale)
          || guiScale <= 0.0f) {
        std::cerr << "Invalid --gui-scale value '" << value
                  << "'. Expected a positive number.\n";
        std::exit(2);
      }
      options.guiScale = std::clamp(guiScale, 0.5f, 4.0f);
    } else if (arg == "--profile") {
      options.profile = true;
    } else if (arg == "--scene" && i + 1 < argc) {
      const std::string_view sceneArg(argv[++i]);
      if (!parseSceneName(sceneArg, options.scene)) {
        std::cerr << "Unknown scene '" << sceneArg
                  << "'. Expected 'mvp', 'drag-and-drop', or 'g1'.\n";
        std::exit(2);
      }
    } else if (
        (arg == "--g1-package-uri" || arg == "--package-uri")
        && i + 1 < argc) {
      options.g1PackageUri = argv[++i];
      g1PackageUriExplicit = true;
    } else if (
        (arg == "--g1-robot-uri" || arg == "--robot-uri") && i + 1 < argc) {
      options.g1RobotUri = argv[++i];
      g1RobotUriExplicit = true;
    } else if (
        (arg == "--g1-package-name" || arg == "--package-name")
        && i + 1 < argc) {
      options.g1PackageName = argv[++i];
      g1PackageNameExplicit = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--frames N] [--width N] [--height N]"
                   " [--screenshot PATH] [--headless]"
                   " [--hide-ui|--show-ui]"
                   " [--orbit-light|--no-orbit-light]"
                   " [--orbit-light-period SECONDS]"
                   " [--gui-scale N]"
                   " [--profile]"
                   " [--scene mvp|drag-and-drop|g1]"
                   " [--g1-package-uri URI] [--g1-robot-uri URI]"
                   " [--g1-package-name NAME]\n";
      std::exit(0);
    }
  }

  if (!g1PackageNameExplicit) {
    if (g1RobotUriExplicit) {
      if (auto packageName
          = inferPackageNameFromRobotUri(options.g1RobotUri)) {
        options.g1PackageName = *packageName;
      }
    } else if (g1PackageUriExplicit) {
      if (auto packageName
          = inferPackageNameFromPackageUri(options.g1PackageUri)) {
        options.g1PackageName = *packageName;
      }
    } else if (auto packageName
               = inferPackageNameFromRobotUri(options.g1RobotUri)) {
      options.g1PackageName = *packageName;
    }
  }

  normalizeRunOptions(options.run);
  if (options.guiScale != 1.0f) {
    options.run.width = std::max(
        1,
        static_cast<int>(
            std::lround(static_cast<float>(options.run.width) * options.guiScale)));
    options.run.height = std::max(
        1,
        static_cast<int>(
            std::lround(static_cast<float>(options.run.height) * options.guiScale)));
  }
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

void updateImGuiMouseInput(
    GLFWwindow* window,
    ImGuiIO& io,
    int framebufferWidth,
    int framebufferHeight)
{
  for (bool& mouseDown : io.MouseDown) {
    mouseDown = false;
  }

  if (window == nullptr) {
    const float offscreen = -std::numeric_limits<float>::max();
    io.MousePos = ImVec2(offscreen, offscreen);
    io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);
    return;
  }

  int windowWidth = framebufferWidth;
  int windowHeight = framebufferHeight;
  glfwGetWindowSize(window, &windowWidth, &windowHeight);
  const float xScale
      = windowWidth > 0
            ? static_cast<float>(framebufferWidth) / static_cast<float>(windowWidth)
            : 1.0f;
  const float yScale
      = windowHeight > 0
            ? static_cast<float>(framebufferHeight) / static_cast<float>(windowHeight)
            : 1.0f;
  io.DisplayFramebufferScale = ImVec2(xScale, yScale);

  double cursorX = 0.0;
  double cursorY = 0.0;
  glfwGetCursorPos(window, &cursorX, &cursorY);
  io.MousePos = ImVec2(
      static_cast<float>(cursorX) * xScale,
      static_cast<float>(cursorY) * yScale);
  io.MouseDown[0] = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  io.MouseDown[1]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  io.MouseDown[2]
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

bool isInsideStatusPanel(double cursorX, double cursorY, float guiScale)
{
  return cursorX >= 20.0 * guiScale && cursorX <= 360.0 * guiScale
         && cursorY >= 20.0 * guiScale && cursorY <= 360.0 * guiScale;
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

G1IkHandle* findG1IkHandle(DartScene& scene, RenderableId targetRenderableId)
{
  const auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [&](const G1IkHandle& candidate) {
        return candidate.targetRenderableId == targetRenderableId;
      });
  return handle == scene.ikHandles.end() ? nullptr : &*handle;
}

const G1IkHandle* findG1IkHandle(
    const DartScene& scene, RenderableId targetRenderableId)
{
  const auto handle = std::find_if(
      scene.ikHandles.begin(),
      scene.ikHandles.end(),
      [&](const G1IkHandle& candidate) {
        return candidate.targetRenderableId == targetRenderableId;
      });
  return handle == scene.ikHandles.end() ? nullptr : &*handle;
}

std::string selectionLabelForRenderable(
    const DartScene& scene, const RenderableDescriptor& descriptor)
{
  if (const auto* handle = findG1IkHandle(scene, descriptor.id)) {
    return handle->label + " IK target";
  }

  std::string label = descriptor.skeletonName.empty()
                          ? descriptor.shapeFrameName
                          : descriptor.skeletonName + "/" + descriptor.bodyName;
  if (!descriptor.shapeNodeName.empty()) {
    label += "/" + descriptor.shapeNodeName;
  }
  label += " (" + descriptor.geometry.shapeType + ")";
  return label;
}

bool translateRenderableAndApplyIk(
    DartScene& scene,
    const RenderableDescriptor& descriptor,
    const Eigen::Vector3d& worldTranslation)
{
  if (!translateFrameRenderable(descriptor, worldTranslation)) {
    return false;
  }

  if (auto* handle = findG1IkHandle(scene, descriptor.id)) {
    if (handle->ik) {
      handle->ik->getSolver()->setNumMaxIterations(30);
      handle->ik->solveAndApply(true);
    }
  }

  return true;
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

  auto createLineSegmentShape = [] {
    auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(
        Eigen::Vector3d(-0.35, 0.0, 0.0),
        Eigen::Vector3d(0.0, 0.25, 0.25),
        2.0f);
    shape->addVertex(Eigen::Vector3d(0.35, 0.0, 0.0), 1);
    shape->addVertex(Eigen::Vector3d(0.0, -0.25, 0.25), 0);
    return shape;
  };

  auto createPointCloudShape = [] {
    auto shape = std::make_shared<PointCloudShape>(0.14);
    shape->setPointShapeType(PointCloudShape::BOX);
    shape->setColorMode(PointCloudShape::BIND_OVERALL);
    shape->setOverallColor(Eigen::Vector4d(0.48, 0.86, 0.38, 1.0));
    shape->addPoint(Eigen::Vector3d(-0.24, -0.18, 0.0));
    shape->addPoint(Eigen::Vector3d(0.0, 0.16, 0.12));
    shape->addPoint(Eigen::Vector3d(0.24, -0.14, 0.02));
    shape->addPoint(Eigen::Vector3d(-0.02, -0.02, 0.3));
    return shape;
  };

  auto createHeightmapShape = [] {
    auto shape = std::make_shared<HeightmapShaped>();
    const std::array<double, 12> heights{
        0.02, 0.10, 0.06, 0.12, 0.08, 0.18,
        0.14, 0.05, 0.10, 0.24, 0.16, 0.08};
    shape->setHeightField(4u, 3u, heights);
    shape->setScale(Eigen::Vector3d(0.18, 0.18, 1.0));
    return shape;
  };

#if DART_HAVE_OCTOMAP
  auto createVoxelGridShape = [] {
    auto shape = std::make_shared<VoxelGridShape>(0.12);
    const std::array<Eigen::Vector3d, 7> occupiedVoxels{
        Eigen::Vector3d(-0.18, -0.06, 0.0),
        Eigen::Vector3d(-0.06, -0.06, 0.0),
        Eigen::Vector3d(0.06, -0.06, 0.0),
        Eigen::Vector3d(0.18, -0.06, 0.0),
        Eigen::Vector3d(-0.06, 0.06, 0.12),
        Eigen::Vector3d(0.06, 0.06, 0.12),
        Eigen::Vector3d(0.0, 0.18, 0.24)};
    for (const Eigen::Vector3d& voxel : occupiedVoxels) {
      shape->updateOccupancy(voxel);
    }
    return shape;
  };
#endif

  auto createSoftMeshSkeleton = [] {
    auto skeleton = Skeleton::create(kSoftMeshFixtureSkeletonName);
    auto [joint, softBody]
        = skeleton->createJointAndBodyNodePair<WeldJoint, SoftBodyNode>();
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(-0.05, 0.55, 0.9);
    joint->setTransformFromParentBodyNode(tf);

    SoftBodyNodeHelper::setBox(
        softBody,
        Eigen::Vector3d(0.42, 0.32, 0.26),
        Eigen::Isometry3d::Identity(),
        Eigen::Vector3i(3, 3, 3),
        1.0,
        10.0,
        10.0,
        0.1);

    for (std::size_t i = 0; i < softBody->getNumPointMasses(); ++i) {
      auto* pointMass = softBody->getPointMass(i);
      if (pointMass == nullptr) {
        continue;
      }
      const Eigen::Vector3d& resting = pointMass->getRestingPosition();
      pointMass->setPositions(Eigen::Vector3d(
          0.0,
          0.0,
          0.05
              * std::sin(7.0 * resting.x() + 5.0 * resting.y()
                         + static_cast<double>(i) * 0.3)));
    }

    for (std::size_t i = 0; i < softBody->getNumShapeNodes(); ++i) {
      auto* shapeNode = softBody->getShapeNode(i);
      if (shapeNode == nullptr || shapeNode->getVisualAspect() == nullptr) {
        continue;
      }
      shapeNode->getVisualAspect()->setRGBA(
          Eigen::Vector4d(0.88, 0.44, 0.72, 0.82));
    }

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
      kPyramidFixtureSkeletonName,
      std::make_shared<dart::dynamics::PyramidShape>(0.55, 0.45, 0.7),
      Eigen::Vector3d(-2.15, -0.15, 0.82),
      Eigen::Vector3d(0.9, 0.72, 0.24)));
  scene.world->addSkeleton(createStaticVisual(
      kMultiSphereFixtureSkeletonName,
      std::make_shared<dart::dynamics::MultiSphereConvexHullShape>(
          dart::dynamics::MultiSphereConvexHullShape::Spheres{
              {0.18, Eigen::Vector3d(-0.22, 0.0, 0.0)},
              {0.26, Eigen::Vector3d(0.16, 0.0, 0.04)},
              {0.14, Eigen::Vector3d(0.42, 0.0, -0.03)}}),
      Eigen::Vector3d(-2.85, -0.15, 0.72),
      Eigen::Vector3d(0.38, 0.74, 0.92)));
  scene.world->addSkeleton(createStaticVisual(
      kLineSegmentFixtureSkeletonName,
      createLineSegmentShape(),
      Eigen::Vector3d(-2.85, 0.55, 0.88),
      Eigen::Vector3d(0.96, 0.68, 0.22)));
  scene.world->addSkeleton(createStaticVisual(
      kConvexMeshFixtureSkeletonName,
      std::make_shared<ConvexMeshShape>(createTetraMesh()),
      Eigen::Vector3d(-2.15, 0.55, 0.95),
      Eigen::Vector3d(0.48, 0.86, 0.38)));
  scene.world->addSkeleton(createStaticVisual(
      kPointCloudFixtureSkeletonName,
      createPointCloudShape(),
      Eigen::Vector3d(-1.45, 0.55, 0.82),
      Eigen::Vector3d(0.48, 0.86, 0.38)));
  scene.world->addSkeleton(createStaticVisual(
      kHeightmapFixtureSkeletonName,
      createHeightmapShape(),
      Eigen::Vector3d(-0.75, 0.55, 0.65),
      Eigen::Vector3d(0.42, 0.74, 0.36)));
  scene.world->addSkeleton(createSoftMeshSkeleton());
#if DART_HAVE_OCTOMAP
  scene.world->addSkeleton(createStaticVisual(
      kVoxelGridFixtureSkeletonName,
      createVoxelGridShape(),
      Eigen::Vector3d(0.55, 0.55, 0.72),
      Eigen::Vector3d(0.94, 0.55, 0.24),
      0.78));
#endif
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

DartScene createG1DartScene(const AppOptions& options)
{
  DartScene scene;
  scene.world = World::create("filament_gui_g1");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->addSkeleton(createG1Ground());

  auto g1 = loadG1Skeleton(options);
  std::cout << "Loaded G1 robot from '" << options.g1RobotUri << "'.\n"
            << "Package root for '" << options.g1PackageName << "' set to '"
            << options.g1PackageUri << "'.\n";
  scene.world->addSkeleton(g1);
  addG1IkTargets(scene, g1);

  return scene;
}

DartScene createDartScene(const AppOptions& options)
{
  switch (options.scene) {
    case ExampleScene::Mvp:
      return createMvpDartScene();
    case ExampleScene::DragAndDrop:
      return createDragAndDropScene();
    case ExampleScene::G1:
      return createG1DartScene(options);
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

float4 toFloat4(const Eigen::Vector4d& vector)
{
  return {
      static_cast<float>(vector.x()),
      static_cast<float>(vector.y()),
      static_cast<float>(vector.z()),
      static_cast<float>(vector.w())};
}

float3 normalizeOr(const float3& vector, const float3& fallback)
{
  const float lengthSquared = vector.x * vector.x + vector.y * vector.y
                              + vector.z * vector.z;
  if (lengthSquared <= 1e-12f) {
    return fallback;
  }

  const float inverseLength = 1.0f / std::sqrt(lengthSquared);
  return {
      vector.x * inverseLength,
      vector.y * inverseLength,
      vector.z * inverseLength};
}

float3 crossProduct(const float3& lhs, const float3& rhs)
{
  return {
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x};
}

bool hasUsableTextureCoordinates(const std::vector<Vertex>& vertices)
{
  return std::any_of(vertices.begin(), vertices.end(), [](const Vertex& vertex) {
    return std::abs(vertex.uv.x) > 1e-7f || std::abs(vertex.uv.y) > 1e-7f;
  });
}

void generateTangentFrames(
    std::vector<Vertex>& vertices,
    const std::vector<filament::math::uint3>& triangles,
    const std::vector<float3>& normals = {})
{
  std::vector<filament::math::float2> uvs;
  std::vector<float3> positions;
  if (hasUsableTextureCoordinates(vertices)) {
    uvs.reserve(vertices.size());
    positions.reserve(vertices.size());
    std::transform(
        vertices.begin(),
        vertices.end(),
        std::back_inserter(uvs),
        [](const Vertex& vertex) { return vertex.uv; });
    std::transform(
        vertices.begin(),
        vertices.end(),
        std::back_inserter(positions),
        [](const Vertex& vertex) { return vertex.position; });
  }

  filament::geometry::SurfaceOrientation::Builder builder;
  builder.vertexCount(vertices.size())
      .positions(
          positions.empty() ? &vertices[0].position : positions.data(),
          positions.empty() ? sizeof(Vertex) : 0)
      .triangleCount(triangles.size())
      .triangles(triangles.data());
  if (normals.size() == vertices.size()) {
    builder.normals(normals.data());
  }
  if (!uvs.empty()) {
    builder.uvs(uvs.data());
  }

  std::unique_ptr<filament::geometry::SurfaceOrientation> orientation(
      builder.build());
  if (orientation == nullptr) {
    std::cerr << "Failed to generate Filament tangent frames\n";
    std::exit(1);
  }
  orientation->getQuats(&vertices[0].tangent, vertices.size(), sizeof(Vertex));
}

float3 orbitingKeyLightDirection(double elapsedSeconds, double orbitPeriodSeconds)
{
  constexpr double pi = 3.14159265358979323846;
  const double angle
      = elapsedSeconds * 2.0 * pi / std::max(orbitPeriodSeconds, 1.0);
  return normalizeOr(
      {static_cast<float>(0.68 * std::cos(angle)),
       static_cast<float>(0.68 * std::sin(angle)),
       -0.74f},
      {-0.30f, -0.42f, -1.0f});
}

double perspectiveNearPlane(const OrbitCamera& camera)
{
  return std::clamp(camera.distance * 0.004, 0.002, 0.025);
}

double perspectiveFarPlane(const OrbitCamera& camera)
{
  return std::max(30.0, camera.distance + 35.0);
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

std::optional<Renderable> createLineSegmentRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const RenderableDescriptor& descriptor)
{
  std::vector<DebugLineDescriptor> lines;
  lines.reserve(descriptor.geometry.lineConnections.size());
  for (const Eigen::Vector2i& connection :
       descriptor.geometry.lineConnections) {
    if (connection.x() < 0 || connection.y() < 0) {
      continue;
    }
    const auto first = static_cast<std::size_t>(connection.x());
    const auto second = static_cast<std::size_t>(connection.y());
    if (first >= descriptor.geometry.lineVertices.size()
        || second >= descriptor.geometry.lineVertices.size()) {
      continue;
    }

    DebugLineDescriptor line;
    line.from = descriptor.geometry.lineVertices[first];
    line.to = descriptor.geometry.lineVertices[second];
    line.rgba = descriptor.material.rgba;
    line.label = descriptor.shapeFrameName;
    lines.push_back(line);
  }

  return createDebugLineRenderable(engine, material, lines);
}

Renderable createTriangleMeshRenderable(
    filament::Engine& engine,
    filament::Material& material,
    std::vector<Vertex> vertices,
    std::vector<std::uint32_t> indices,
    std::vector<filament::math::uint3> triangles,
    std::vector<float3> normals,
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
  generateTangentFrames(vertices, triangles, normals);

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

void appendEllipsoidGeometry(
    std::vector<Vertex>& vertices,
    std::vector<float3>& normals,
    std::vector<std::uint32_t>& indices,
    std::vector<filament::math::uint3>& triangles,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& radii)
{
  static constexpr std::uint32_t segments = 32;
  static constexpr std::uint32_t rings = 16;
  static constexpr double pi = 3.14159265358979323846;
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const std::uint32_t start = static_cast<std::uint32_t>(vertices.size());

  for (std::uint32_t ring = 0; ring <= rings; ++ring) {
    const double theta = -pi / 2.0 + pi * ring / rings;
    const double z = std::sin(theta);
    const double radial = std::cos(theta);
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double phi = 2.0 * pi * segment / segments;
      vertices.push_back(
          Vertex{{static_cast<float>(center.x() + radii.x() * radial * std::cos(phi)),
                  static_cast<float>(center.y() + radii.y() * radial * std::sin(phi)),
                  static_cast<float>(center.z() + radii.z() * z)},
                 tangent});
      const Eigen::Vector3d normal{
          radii.x() > 1e-12 ? radial * std::cos(phi) / radii.x() : 0.0,
          radii.y() > 1e-12 ? radial * std::sin(phi) / radii.y() : 0.0,
          radii.z() > 1e-12 ? z / radii.z() : 0.0};
      normals.push_back(toFloat3(
          normal.squaredNorm() > 1e-12
              ? normal.normalized()
              : Eigen::Vector3d(0.0, 0.0, z >= 0.0 ? 1.0 : -1.0)));
    }
  }

  for (std::uint32_t ring = 0; ring < rings; ++ring) {
    for (std::uint32_t segment = 0; segment < segments; ++segment) {
      const std::uint32_t a = ring * (segments + 1) + segment;
      const std::uint32_t b = a + 1;
      const std::uint32_t c = (ring + 1) * (segments + 1) + segment;
      const std::uint32_t d = c + 1;
      appendTriangle(indices, triangles, start + a, start + b, start + c);
      appendTriangle(indices, triangles, start + b, start + d, start + c);
    }
  }
}

Renderable createEllipsoidRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const Eigen::Vector3d& radii,
    const float4& color)
{
  static constexpr std::uint32_t segments = 32;
  static constexpr std::uint32_t rings = 16;

  std::vector<Vertex> vertices;
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve((rings + 1) * (segments + 1));
  normals.reserve((rings + 1) * (segments + 1));
  indices.reserve(rings * segments * 6);
  triangles.reserve(rings * segments * 2);

  appendEllipsoidGeometry(
      vertices, normals, indices, triangles, Eigen::Vector3d::Zero(), radii);

  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      toFloat3(-radii),
      toFloat3(radii));
}

Renderable createMultiSphereRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const std::vector<Eigen::Vector3d>& centers,
    const std::vector<double>& radii,
    const float4& color)
{
  static constexpr std::uint32_t segments = 32;
  static constexpr std::uint32_t rings = 16;
  const std::size_t sphereCount = std::min(centers.size(), radii.size());
  const std::size_t verticesPerSphere = (rings + 1) * (segments + 1);
  const std::size_t trianglesPerSphere = rings * segments * 2;

  std::vector<Vertex> vertices;
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(sphereCount * verticesPerSphere);
  normals.reserve(sphereCount * verticesPerSphere);
  indices.reserve(sphereCount * trianglesPerSphere * 3);
  triangles.reserve(sphereCount * trianglesPerSphere);

  Eigen::Vector3d minBounds = Eigen::Vector3d::Zero();
  Eigen::Vector3d maxBounds = Eigen::Vector3d::Zero();
  bool hasBounds = false;
  for (std::size_t i = 0; i < sphereCount; ++i) {
    const double radius = radii[i];
    if (radius <= 0.0) {
      continue;
    }
    const Eigen::Vector3d extent = Eigen::Vector3d::Constant(radius);
    appendEllipsoidGeometry(
        vertices, normals, indices, triangles, centers[i], extent);
    if (!hasBounds) {
      minBounds = centers[i] - extent;
      maxBounds = centers[i] + extent;
      hasBounds = true;
    } else {
      minBounds = minBounds.cwiseMin(centers[i] - extent);
      maxBounds = maxBounds.cwiseMax(centers[i] + extent);
    }
  }

  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      toFloat3(minBounds),
      toFloat3(maxBounds));
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
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve((segments + 1) * 4 + 2);
  normals.reserve((segments + 1) * 4 + 2);

  for (const auto& [z, v] :
       {std::pair{-halfHeight, 0.0f}, std::pair{halfHeight, 1.0f}}) {
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double phi = 2.0 * pi * segment / segments;
      const float cosPhi = static_cast<float>(std::cos(phi));
      const float sinPhi = static_cast<float>(std::sin(phi));
      vertices.push_back(
          Vertex{{static_cast<float>(radius * cosPhi),
                  static_cast<float>(radius * sinPhi),
                  z},
                 tangent,
                 {static_cast<float>(segment) / static_cast<float>(segments),
                  v}});
      normals.push_back(normalizeOr({cosPhi, sinPhi, 0.0f}, {1.0f, 0.0f, 0.0f}));
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

  const std::uint32_t bottomCapStart
      = static_cast<std::uint32_t>(vertices.size());
  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * pi * segment / segments;
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    vertices.push_back(
        Vertex{{static_cast<float>(radius * cosPhi),
                static_cast<float>(radius * sinPhi),
                -halfHeight},
               tangent,
               {0.5f + 0.5f * cosPhi, 0.5f + 0.5f * sinPhi}});
    normals.push_back({0.0f, 0.0f, -1.0f});
  }
  const std::uint32_t topCapStart
      = static_cast<std::uint32_t>(vertices.size());
  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * pi * segment / segments;
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    vertices.push_back(
        Vertex{{static_cast<float>(radius * cosPhi),
                static_cast<float>(radius * sinPhi),
                halfHeight},
               tangent,
               {0.5f + 0.5f * cosPhi, 0.5f + 0.5f * sinPhi}});
    normals.push_back({0.0f, 0.0f, 1.0f});
  }
  const std::uint32_t bottomCenter = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, -halfHeight}, tangent, {0.5f, 0.5f}});
  normals.push_back({0.0f, 0.0f, -1.0f});
  const std::uint32_t topCenter = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, halfHeight}, tangent, {0.5f, 0.5f}});
  normals.push_back({0.0f, 0.0f, 1.0f});
  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    appendTriangle(
        indices,
        triangles,
        bottomCenter,
        bottomCapStart + segment + 1,
        bottomCapStart + segment);
    appendTriangle(
        indices,
        triangles,
        topCenter,
        topCapStart + segment,
        topCapStart + segment + 1);
  }

  const float r = static_cast<float>(radius);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
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
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(segments * 3 + segments + 2);
  normals.reserve(segments * 3 + segments + 2);

  const auto sideNormal = [&](double phi) {
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    const float z = height > 1e-12 ? static_cast<float>(radius / height) : 0.0f;
    return normalizeOr({cosPhi, sinPhi, z}, {cosPhi, sinPhi, 0.0f});
  };

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    const double phi0 = 2.0 * pi * segment / segments;
    const double phi1 = 2.0 * pi * (segment + 1) / segments;
    const double phiMid = 0.5 * (phi0 + phi1);
    const float u0 = static_cast<float>(segment) / static_cast<float>(segments);
    const float u1
        = static_cast<float>(segment + 1) / static_cast<float>(segments);
    const float cos0 = static_cast<float>(std::cos(phi0));
    const float sin0 = static_cast<float>(std::sin(phi0));
    const float cos1 = static_cast<float>(std::cos(phi1));
    const float sin1 = static_cast<float>(std::sin(phi1));
    const std::uint32_t base0 = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back(
        Vertex{{static_cast<float>(radius * cos0),
                static_cast<float>(radius * sin0),
                -halfHeight},
               tangent,
               {u0, 0.0f}});
    normals.push_back(sideNormal(phi0));
    const std::uint32_t base1 = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back(
        Vertex{{static_cast<float>(radius * cos1),
                static_cast<float>(radius * sin1),
                -halfHeight},
               tangent,
               {u1, 0.0f}});
    normals.push_back(sideNormal(phi1));
    const std::uint32_t tip = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back(Vertex{{0.0f, 0.0f, halfHeight}, tangent, {0.5f * (u0 + u1), 1.0f}});
    normals.push_back(sideNormal(phiMid));
    appendTriangle(indices, triangles, base0, base1, tip);
  }

  const std::uint32_t baseStart = static_cast<std::uint32_t>(vertices.size());
  for (std::uint32_t segment = 0; segment <= segments; ++segment) {
    const double phi = 2.0 * pi * segment / segments;
    const float cosPhi = static_cast<float>(std::cos(phi));
    const float sinPhi = static_cast<float>(std::sin(phi));
    vertices.push_back(
        Vertex{{static_cast<float>(radius * cosPhi),
                static_cast<float>(radius * sinPhi),
                -halfHeight},
               tangent,
               {0.5f + 0.5f * cosPhi, 0.5f + 0.5f * sinPhi}});
    normals.push_back({0.0f, 0.0f, -1.0f});
  }
  const std::uint32_t center = static_cast<std::uint32_t>(vertices.size());
  vertices.push_back(Vertex{{0.0f, 0.0f, -halfHeight}, tangent, {0.5f, 0.5f}});
  normals.push_back({0.0f, 0.0f, -1.0f});

  for (std::uint32_t segment = 0; segment < segments; ++segment) {
    appendTriangle(indices, triangles, center, baseStart + segment + 1, baseStart + segment);
  }

  const float r = static_cast<float>(radius);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      {-r, -r, -halfHeight},
      {r, r, halfHeight});
}

Renderable createPyramidRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const Eigen::Vector3d& size,
    const float4& color)
{
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const float halfWidth = static_cast<float>(size.x() * 0.5);
  const float halfDepth = static_cast<float>(size.y() * 0.5);
  const float halfHeight = static_cast<float>(size.z() * 0.5);

  const std::array<float3, 5> points = {
      float3{0.0f, 0.0f, halfHeight},
      float3{-halfWidth, -halfDepth, -halfHeight},
      float3{halfWidth, -halfDepth, -halfHeight},
      float3{halfWidth, halfDepth, -halfHeight},
      float3{-halfWidth, halfDepth, -halfHeight}};

  std::vector<Vertex> vertices;
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(18);
  normals.reserve(18);
  indices.reserve(18);
  triangles.reserve(6);

  const auto appendFace = [&](std::uint32_t a,
                              std::uint32_t b,
                              std::uint32_t c,
                              float2 uvA,
                              float2 uvB,
                              float2 uvC) {
    const float3 normal = normalizeOr(
        crossProduct(points[b] - points[a], points[c] - points[a]),
        {0.0f, 0.0f, 1.0f});
    const auto start = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back(Vertex{points[a], tangent, uvA});
    vertices.push_back(Vertex{points[b], tangent, uvB});
    vertices.push_back(Vertex{points[c], tangent, uvC});
    normals.push_back(normal);
    normals.push_back(normal);
    normals.push_back(normal);
    appendTriangle(indices, triangles, start, start + 1u, start + 2u);
  };

  appendFace(0, 1, 2, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendFace(0, 2, 3, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendFace(0, 3, 4, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendFace(0, 4, 1, {0.5f, 1.0f}, {0.0f, 0.0f}, {1.0f, 0.0f});
  appendFace(1, 4, 3, {0.0f, 0.0f}, {0.0f, 1.0f}, {1.0f, 1.0f});
  appendFace(1, 3, 2, {0.0f, 0.0f}, {1.0f, 1.0f}, {1.0f, 0.0f});

  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      {-halfWidth, -halfDepth, -halfHeight},
      {halfWidth, halfDepth, halfHeight});
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
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(rings.size() * (segments + 1));
  normals.reserve(rings.size() * (segments + 1));
  for (const Ring& ring : rings) {
    for (std::uint32_t segment = 0; segment <= segments; ++segment) {
      const double theta = 2.0 * pi * segment / segments;
      const float x = static_cast<float>(ring.radius * std::cos(theta));
      const float y = static_cast<float>(ring.radius * std::sin(theta));
      const float z = static_cast<float>(ring.z);
      vertices.push_back(
          Vertex{{x, y, z},
                 tangent,
                 {static_cast<float>(segment) / static_cast<float>(segments),
                  static_cast<float>(vertices.size() / (segments + 1))
                      / static_cast<float>(rings.size() - 1)}});
      const double capCenterZ = std::clamp(ring.z, -height * 0.5, height * 0.5);
      normals.push_back(normalizeOr(
          {x, y, static_cast<float>(ring.z - capCenterZ)},
          {0.0f, 0.0f, ring.z >= 0.0 ? 1.0f : -1.0f}));
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
      std::move(normals),
      color,
      {-r, -r, -halfTotalHeight},
      {r, r, halfTotalHeight});
}

std::optional<Renderable> createConvexMeshRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const ConvexMeshShape& convexMeshShape,
    const float4& color)
{
  const auto& triMesh = convexMeshShape.getMesh();
  if (!triMesh || triMesh->getVertices().empty()
      || triMesh->getTriangles().empty()) {
    return std::nullopt;
  }
  if (triMesh->getVertices().size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const filament::math::short4 tangent = {0, 0, 0, 32767};
  std::vector<Vertex> vertices;
  vertices.reserve(triMesh->getVertices().size());
  for (const Eigen::Vector3d& vertex : triMesh->getVertices()) {
    vertices.push_back(Vertex{toFloat3(vertex), tangent});
  }

  std::vector<float3> normals;
  if (triMesh->hasVertexNormals()
      && triMesh->getVertexNormals().size() == triMesh->getVertices().size()) {
    normals.reserve(triMesh->getVertexNormals().size());
    for (const Eigen::Vector3d& normal : triMesh->getVertexNormals()) {
      normals.push_back(toFloat3(
          normal.squaredNorm() > 1e-12 ? normal.normalized()
                                       : Eigen::Vector3d::UnitZ()));
    }
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

  const Bounds bounds = computeBounds(vertices);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      bounds.min,
      bounds.max);
}

void appendPointBoxGeometry(
    std::vector<Vertex>& vertices,
    std::vector<float3>& normals,
    std::vector<std::uint32_t>& indices,
    std::vector<filament::math::uint3>& triangles,
    const Eigen::Vector3d& center,
    double size)
{
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const float halfSize = static_cast<float>(size * 0.5);
  const float3 centerPoint = toFloat3(center);
  const std::array<float3, 8> points = {
      float3{
          centerPoint.x - halfSize,
          centerPoint.y - halfSize,
          centerPoint.z - halfSize},
      float3{
          centerPoint.x + halfSize,
          centerPoint.y - halfSize,
          centerPoint.z - halfSize},
      float3{
          centerPoint.x + halfSize,
          centerPoint.y + halfSize,
          centerPoint.z - halfSize},
      float3{
          centerPoint.x - halfSize,
          centerPoint.y + halfSize,
          centerPoint.z - halfSize},
      float3{
          centerPoint.x - halfSize,
          centerPoint.y - halfSize,
          centerPoint.z + halfSize},
      float3{
          centerPoint.x + halfSize,
          centerPoint.y - halfSize,
          centerPoint.z + halfSize},
      float3{
          centerPoint.x + halfSize,
          centerPoint.y + halfSize,
          centerPoint.z + halfSize},
      float3{
          centerPoint.x - halfSize,
          centerPoint.y + halfSize,
          centerPoint.z + halfSize}};

  const auto appendFace = [&](std::uint32_t a,
                              std::uint32_t b,
                              std::uint32_t cIndex,
                              std::uint32_t d,
                              const float3& normal) {
    const auto start = static_cast<std::uint32_t>(vertices.size());
    vertices.push_back(Vertex{points[a], tangent, {0.0f, 0.0f}});
    vertices.push_back(Vertex{points[b], tangent, {1.0f, 0.0f}});
    vertices.push_back(Vertex{points[cIndex], tangent, {1.0f, 1.0f}});
    vertices.push_back(Vertex{points[d], tangent, {0.0f, 1.0f}});
    normals.push_back(normal);
    normals.push_back(normal);
    normals.push_back(normal);
    normals.push_back(normal);
    appendTriangle(indices, triangles, start, start + 1u, start + 2u);
    appendTriangle(indices, triangles, start, start + 2u, start + 3u);
  };

  appendFace(4, 5, 6, 7, {0.0f, 0.0f, 1.0f});
  appendFace(0, 3, 2, 1, {0.0f, 0.0f, -1.0f});
  appendFace(3, 7, 6, 2, {0.0f, 1.0f, 0.0f});
  appendFace(0, 1, 5, 4, {0.0f, -1.0f, 0.0f});
  appendFace(1, 2, 6, 5, {1.0f, 0.0f, 0.0f});
  appendFace(0, 4, 7, 3, {-1.0f, 0.0f, 0.0f});
}

std::optional<Renderable> createPointCloudRenderable(
    filament::Engine& engine,
    const MaterialSet& materials,
    const RenderableDescriptor& descriptor)
{
  if (descriptor.geometry.pointCloudPoints.empty()) {
    return std::nullopt;
  }

  float4 color = toRgba(descriptor.material.rgba);
  if (descriptor.geometry.pointCloudColors.size() == 1u) {
    color = toRgba(descriptor.geometry.pointCloudColors.front());
  }

  const double pointSize = std::max(descriptor.geometry.pointSize, 1e-4);
  std::vector<Vertex> vertices;
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(descriptor.geometry.pointCloudPoints.size() * 24u);
  normals.reserve(descriptor.geometry.pointCloudPoints.size() * 24u);
  indices.reserve(descriptor.geometry.pointCloudPoints.size() * 36u);
  triangles.reserve(descriptor.geometry.pointCloudPoints.size() * 12u);

  for (const Eigen::Vector3d& point : descriptor.geometry.pointCloudPoints) {
    appendPointBoxGeometry(
        vertices, normals, indices, triangles, point, pointSize);
  }

  const Bounds bounds = computeBounds(vertices);
  return createTriangleMeshRenderable(
      engine,
      selectLitMaterial(materials, false, color),
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      bounds.min,
      bounds.max);
}

std::optional<Renderable> createVoxelGridRenderable(
    filament::Engine& engine,
    const MaterialSet& materials,
    const RenderableDescriptor& descriptor)
{
  if (descriptor.geometry.voxelCenters.empty()) {
    return std::nullopt;
  }

  const float4 color = toRgba(descriptor.material.rgba);
  const double voxelSize = std::max(descriptor.geometry.voxelSize, 1e-4);
  std::vector<Vertex> vertices;
  std::vector<float3> normals;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  vertices.reserve(descriptor.geometry.voxelCenters.size() * 24u);
  normals.reserve(descriptor.geometry.voxelCenters.size() * 24u);
  indices.reserve(descriptor.geometry.voxelCenters.size() * 36u);
  triangles.reserve(descriptor.geometry.voxelCenters.size() * 12u);

  for (const Eigen::Vector3d& center : descriptor.geometry.voxelCenters) {
    appendPointBoxGeometry(
        vertices, normals, indices, triangles, center, voxelSize);
  }

  const Bounds bounds = computeBounds(vertices);
  return createTriangleMeshRenderable(
      engine,
      selectLitMaterial(materials, false, color),
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      bounds.min,
      bounds.max);
}

template <typename S>
std::optional<Renderable> createHeightmapRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const dart::dynamics::HeightmapShape<S>& heightmapShape,
    const float4& color)
{
  const auto& heightmap = heightmapShape.getHeightField();
  const Eigen::Index rows = heightmap.rows();
  const Eigen::Index cols = heightmap.cols();
  if (rows < 2 || cols < 2) {
    return std::nullopt;
  }
  if (static_cast<std::size_t>(heightmap.size())
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const auto& scale = heightmapShape.getScale();
  const S spanX = static_cast<S>(cols - 1) * scale.x();
  const S spanY = static_cast<S>(rows - 1) * scale.y();
  const S xOffset = static_cast<S>(-0.5) * spanX;
  const S yOffset = static_cast<S>(0.5) * spanY;
  const filament::math::short4 tangent = {0, 0, 0, 32767};

  std::vector<Vertex> vertices;
  vertices.reserve(static_cast<std::size_t>(heightmap.size()));
  for (Eigen::Index row = 0; row < rows; ++row) {
    for (Eigen::Index col = 0; col < cols; ++col) {
      vertices.push_back(Vertex{
          {static_cast<float>(static_cast<S>(col) * scale.x() + xOffset),
           static_cast<float>(-static_cast<S>(row) * scale.y() + yOffset),
           static_cast<float>(heightmap(row, col) * scale.z())},
          tangent});
    }
  }

  const auto vertexIndex = [cols](Eigen::Index row, Eigen::Index col) {
    return static_cast<std::uint32_t>(row * cols + col);
  };

  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  indices.reserve(static_cast<std::size_t>((rows - 1) * (cols - 1) * 6));
  triangles.reserve(static_cast<std::size_t>((rows - 1) * (cols - 1) * 2));
  for (Eigen::Index row = 1; row < rows; ++row) {
    for (Eigen::Index col = 1; col < cols; ++col) {
      const auto p1 = vertexIndex(row - 1, col - 1);
      const auto p2 = vertexIndex(row - 1, col);
      const auto p3 = vertexIndex(row, col - 1);
      const auto curr = vertexIndex(row, col);
      appendTriangle(indices, triangles, p1, p3, p2);
      appendTriangle(indices, triangles, p2, p3, curr);
    }
  }

  std::vector<float3> normalSums(vertices.size(), {0.0f, 0.0f, 0.0f});
  for (const auto& triangle : triangles) {
    const float3 normal = normalizeOr(
        crossProduct(
            vertices[triangle.y].position - vertices[triangle.x].position,
            vertices[triangle.z].position - vertices[triangle.x].position),
        {0.0f, 0.0f, 1.0f});
    for (std::uint32_t index : {triangle.x, triangle.y, triangle.z}) {
      normalSums[index].x += normal.x;
      normalSums[index].y += normal.y;
      normalSums[index].z += normal.z;
    }
  }

  std::vector<float3> normals;
  normals.reserve(normalSums.size());
  for (const float3& normal : normalSums) {
    normals.push_back(normalizeOr(normal, {0.0f, 0.0f, 1.0f}));
  }

  const Bounds bounds = computeBounds(vertices);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      bounds.min,
      bounds.max);
}

std::optional<Renderable> createSoftMeshRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const SoftMeshShape& softMeshShape,
    const float4& color)
{
  const auto triMesh = softMeshShape.getTriMesh();
  if (!triMesh || triMesh->getTriangles().empty()) {
    return std::nullopt;
  }

  const auto* softBody = softMeshShape.getSoftBodyNode();
  const std::size_t vertexCount
      = softBody != nullptr ? softBody->getNumPointMasses()
                            : triMesh->getVertices().size();
  if (vertexCount == 0u
      || vertexCount
             > static_cast<std::size_t>(
                 std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const filament::math::short4 tangent = {0, 0, 0, 32767};
  std::vector<Vertex> vertices;
  vertices.reserve(vertexCount);
  for (std::size_t i = 0; i < vertexCount; ++i) {
    if (softBody != nullptr) {
      const auto* pointMass = softBody->getPointMass(i);
      if (pointMass == nullptr) {
        return std::nullopt;
      }
      vertices.push_back(
          Vertex{toFloat3(pointMass->getLocalPosition()), tangent});
    } else {
      vertices.push_back(Vertex{toFloat3(triMesh->getVertices()[i]), tangent});
    }
  }

  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  indices.reserve(triMesh->getTriangles().size() * 3u);
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

  std::vector<float3> normalSums(vertices.size(), {0.0f, 0.0f, 0.0f});
  for (const auto& triangle : triangles) {
    const float3 normal = normalizeOr(
        crossProduct(
            vertices[triangle.y].position - vertices[triangle.x].position,
            vertices[triangle.z].position - vertices[triangle.x].position),
        {0.0f, 0.0f, 1.0f});
    for (std::uint32_t index : {triangle.x, triangle.y, triangle.z}) {
      normalSums[index].x += normal.x;
      normalSums[index].y += normal.y;
      normalSums[index].z += normal.z;
    }
  }

  std::vector<float3> normals;
  normals.reserve(normalSums.size());
  for (const float3& normal : normalSums) {
    normals.push_back(normalizeOr(normal, {0.0f, 0.0f, 1.0f}));
  }

  const Bounds bounds = computeBounds(vertices);
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(vertices),
      std::move(indices),
      std::move(triangles),
      std::move(normals),
      color,
      bounds.min,
      bounds.max);
}

std::optional<Renderable> createMeshRenderable(
    filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const GeometryDescriptor& geometry,
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

  const Eigen::Vector3d& scale = geometry.scale;
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const auto textureCoords = meshShape.getTextureCoords();
  const bool hasTextureCoords
      = geometry.meshTextureCoordComponents >= 2
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

  std::vector<float3> normals;
  if (triMesh->hasVertexNormals()
      && triMesh->getVertexNormals().size() == meshVertices.size()) {
    normals.reserve(meshVertices.size());
    for (const Eigen::Vector3d& sourceNormal : triMesh->getVertexNormals()) {
      Eigen::Vector3d scaledNormal = sourceNormal;
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(scale[axis]) > 1e-12) {
          scaledNormal[axis] /= scale[axis];
        } else {
          scaledNormal[axis] = 0.0;
        }
      }
      if (scaledNormal.squaredNorm() <= 1e-12) {
        scaledNormal = sourceNormal;
      }
      normals.push_back(toFloat3(
          scaledNormal.squaredNorm() > 1e-12
              ? scaledNormal.normalized()
              : Eigen::Vector3d::UnitZ()));
    }
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

    if (!geometry.meshUsesMaterialColors || geometry.meshMaterials.empty()) {
      return state;
    }

    if (materialIndex < geometry.meshMaterials.size()) {
      const MeshMaterialDescriptor& meshMaterial
          = geometry.meshMaterials[materialIndex];
      state.followsDescriptorColor = false;
      state.baseColor = toFloat4(meshMaterial.diffuse);
      state.metallic = static_cast<float>(meshMaterial.metallicFactor);
      state.roughness = static_cast<float>(meshMaterial.roughnessFactor);
      state.emissiveColor = {
          static_cast<float>(meshMaterial.emissive.x()),
          static_cast<float>(meshMaterial.emissive.y()),
          static_cast<float>(meshMaterial.emissive.z())};

      if (hasTextureCoords) {
        const auto loadBinding
            = [&](const std::string& source,
                  TextureColorSpace colorSpace) -> const TextureBinding* {
          return getOrLoadTextureBinding(
              engine, textureCache, source, colorSpace);
        };

        const std::string& baseColorTexturePath
            = !meshMaterial.baseColorTexturePath.empty()
                  ? meshMaterial.baseColorTexturePath
                  : (!meshMaterial.textureImagePaths.empty()
                         ? meshMaterial.textureImagePaths[0]
                         : meshMaterial.baseColorTexturePath);
        state.textures.baseColor = loadBinding(
            baseColorTexturePath, TextureColorSpace::Srgb);
        state.textures.metallic = loadBinding(
            meshMaterial.metallicTexturePath, TextureColorSpace::Linear);
        state.textures.roughness = loadBinding(
            meshMaterial.roughnessTexturePath, TextureColorSpace::Linear);
        state.textures.metallicRoughness = loadBinding(
            meshMaterial.metallicRoughnessTexturePath,
            TextureColorSpace::Linear);
        state.textures.normal = loadBinding(
            meshMaterial.normalTexturePath, TextureColorSpace::Linear);
        state.textures.occlusion = loadBinding(
            meshMaterial.occlusionTexturePath, TextureColorSpace::Linear);
        state.textures.emissive = loadBinding(
            meshMaterial.emissiveTexturePath, TextureColorSpace::Srgb);
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
  std::vector<MeshPartDescriptor> parts;
  parts.reserve(geometry.meshParts.size());
  std::size_t coveredTriangleCount = 0;
  bool useSubMeshes = !geometry.meshParts.empty();
  for (const MeshPartDescriptor& range : geometry.meshParts) {
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
        std::move(normals),
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

  generateTangentFrames(vertices, triangles, normals);

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
  const std::vector<float3> normals(4, toFloat3(unitNormal));
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
      normals,
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
    case ShapeKind::Pyramid:
      renderable = createPyramidRenderable(
          engine, solidMaterial, descriptor.geometry.size, color);
      break;
    case ShapeKind::MultiSphere:
      if (descriptor.geometry.sphereCenters.size()
              == descriptor.geometry.sphereRadii.size()
          && std::any_of(
              descriptor.geometry.sphereRadii.begin(),
              descriptor.geometry.sphereRadii.end(),
              [](double radius) { return radius > 0.0; })) {
        renderable = createMultiSphereRenderable(
            engine,
            solidMaterial,
            descriptor.geometry.sphereCenters,
            descriptor.geometry.sphereRadii,
            color);
      }
      break;
    case ShapeKind::LineSegments:
      renderable
          = createLineSegmentRenderable(engine, materials.debugColor, descriptor);
      break;
    case ShapeKind::Capsule:
      renderable = createCapsuleRenderable(
          engine,
          solidMaterial,
          descriptor.geometry.radius,
          descriptor.geometry.height,
          color);
      break;
    case ShapeKind::ConvexMesh:
      if (const auto* convexMeshShape = dynamic_cast<const ConvexMeshShape*>(
              descriptor.shape)) {
        renderable = createConvexMeshRenderable(
            engine, solidMaterial, *convexMeshShape, color);
      }
      break;
    case ShapeKind::PointCloud:
      renderable = createPointCloudRenderable(engine, materials, descriptor);
      break;
    case ShapeKind::VoxelGrid:
      renderable = createVoxelGridRenderable(engine, materials, descriptor);
      break;
    case ShapeKind::Heightmap:
      if (const auto* heightmapShape = dynamic_cast<const HeightmapShapef*>(
              descriptor.shape)) {
        renderable
            = createHeightmapRenderable(engine, solidMaterial, *heightmapShape, color);
      } else if (const auto* heightmapShape
                 = dynamic_cast<const HeightmapShaped*>(descriptor.shape)) {
        renderable
            = createHeightmapRenderable(engine, solidMaterial, *heightmapShape, color);
      }
      break;
    case ShapeKind::SoftMesh:
      if (const auto* softMeshShape = dynamic_cast<const SoftMeshShape*>(
              descriptor.shape)) {
        renderable = createSoftMeshRenderable(
            engine, solidMaterial, *softMeshShape, color);
      }
      break;
    case ShapeKind::Mesh:
      if (const auto* meshShape = dynamic_cast<const MeshShape*>(
              descriptor.shape)) {
        renderable = createMeshRenderable(
            engine, materials, textureCache, descriptor.geometry, *meshShape, color);
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

void logUnsupportedRenderableDescriptor(const RenderableDescriptor& descriptor)
{
  std::cerr << "Unsupported DART shape";
  if (!descriptor.geometry.shapeType.empty()) {
    std::cerr << " '" << descriptor.geometry.shapeType << "'";
  }
  if (!descriptor.shapeFrameName.empty()) {
    std::cerr << " in shape frame '" << descriptor.shapeFrameName << "'";
  }
  if (!descriptor.skeletonName.empty()) {
    std::cerr << " on skeleton '" << descriptor.skeletonName << "'";
  }
  std::cerr << " will not be rendered";
  if (!descriptor.geometry.unsupportedReason.empty()) {
    std::cerr << ": " << descriptor.geometry.unsupportedReason;
  }
  std::cerr << "\n";
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

bool containsRenderableId(
    const std::vector<RenderableId>& ids, RenderableId id)
{
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

std::vector<RenderableId> collectRenderableIds(
    const std::vector<SceneRenderable>& sceneRenderables)
{
  std::vector<RenderableId> ids;
  ids.reserve(sceneRenderables.size());
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    ids.push_back(sceneRenderable.id);
  }
  return ids;
}

void logUnsupportedRenderableDescriptorOnce(
    std::vector<RenderableId>& loggedUnsupportedRenderableIds,
    const RenderableDescriptor& descriptor)
{
  if (descriptor.id != 0
      && containsRenderableId(loggedUnsupportedRenderableIds, descriptor.id)) {
    return;
  }

  logUnsupportedRenderableDescriptor(descriptor);
  if (descriptor.id != 0) {
    loggedUnsupportedRenderableIds.push_back(descriptor.id);
  }
}

void synchronizeSceneRenderables(
    filament::Engine& engine,
    filament::Scene& scene,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const std::vector<RenderableDescriptor>& descriptors,
    std::vector<SceneRenderable>& sceneRenderables,
    std::vector<RenderableId>& loggedUnsupportedRenderableIds)
{
  const auto plan = planRenderableSetUpdate(
      descriptors, collectRenderableIds(sceneRenderables));

  for (auto indexIt = plan.activeRenderableIndicesToRemove.rbegin();
       indexIt != plan.activeRenderableIndicesToRemove.rend();
       ++indexIt) {
    const std::size_t index = *indexIt;
    if (index >= sceneRenderables.size()) {
      continue;
    }

    SceneRenderable& sceneRenderable = sceneRenderables[index];
    scene.remove(sceneRenderable.renderable.entity);
    destroyRenderable(engine, sceneRenderable.renderable);
    sceneRenderables.erase(sceneRenderables.begin() + index);
  }

  for (const std::size_t descriptorIndex : plan.descriptorIndicesToAdd) {
    if (descriptorIndex >= descriptors.size()) {
      continue;
    }

    const RenderableDescriptor& descriptor = descriptors[descriptorIndex];
    auto renderable = createRenderableFromDescriptor(
        engine, materials, textureCache, descriptor);
    if (!renderable) {
      if (descriptor.geometry.kind == ShapeKind::Unsupported) {
        logUnsupportedRenderableDescriptorOnce(
            loggedUnsupportedRenderableIds, descriptor);
      }
      continue;
    }

    SceneRenderable sceneRenderable;
    sceneRenderable.id = descriptor.id;
    sceneRenderable.renderable = *renderable;
    scene.addEntity(sceneRenderable.renderable.entity);
    engine.getTransformManager().setTransform(
        engine.getTransformManager().getInstance(
            sceneRenderable.renderable.entity),
        toFilamentTransform(descriptor.worldTransform));
    sceneRenderables.push_back(sceneRenderable);
  }
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
  if (options.headless) {
    filament::Renderer::DisplayInfo displayInfo;
    displayInfo.refreshRate = 0.0f;
    renderer->setDisplayInfo(displayInfo);
  }
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
  view->setShadowType(filament::ShadowType::PCF);
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
    filament::MultiSampleAntiAliasingOptions multiSampleAntiAliasingOptions;
    multiSampleAntiAliasingOptions.enabled = true;
    multiSampleAntiAliasingOptions.sampleCount = 4;
    view->setMultiSampleAntiAliasingOptions(multiSampleAntiAliasingOptions);
    view->setAntiAliasing(filament::AntiAliasing::FXAA);
    view->setDithering(filament::Dithering::NONE);
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
      *debugMaterial,
      checkerBinding,
      fallbackBinding};
  TextureCache textureCache;

  DartScene dartScene = createDartScene(appOptions);
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
  const std::size_t pyramidDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPyramidFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Pyramid;
          }));
  const std::size_t multiSphereDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kMultiSphereFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::MultiSphere;
          }));
  const std::size_t lineSegmentDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kLineSegmentFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::LineSegments;
          }));
  const std::size_t convexMeshDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kConvexMeshFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::ConvexMesh;
          }));
  const std::size_t pointCloudDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPointCloudFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::PointCloud;
          }));
  const std::size_t heightmapDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kHeightmapFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Heightmap;
          }));
  const std::size_t softMeshDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kSoftMeshFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::SoftMesh;
          }));
#if DART_HAVE_OCTOMAP
  const std::size_t voxelGridDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kVoxelGridFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::VoxelGrid;
          }));
#endif
  const std::size_t pbrEnvironmentDescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
                   && descriptor.material.visible
                   && descriptor.geometry.kind == ShapeKind::Mesh;
          }));
  const std::size_t g1DescriptorCount = static_cast<std::size_t>(
      std::count_if(
          initialDescriptors.begin(),
          initialDescriptors.end(),
          [](const RenderableDescriptor& descriptor) {
            return descriptor.skeletonName == kG1FixtureSkeletonName
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
    if (pyramidDescriptorCount != 1) {
      std::cerr << "Expected the pyramid fixture to provide one visible pyramid "
                   "renderable descriptor, but extracted "
                << pyramidDescriptorCount << "\n";
      return 1;
    }
    if (multiSphereDescriptorCount != 1) {
      std::cerr
          << "Expected the multi-sphere fixture to provide one visible "
             "multi-sphere renderable descriptor, but extracted "
          << multiSphereDescriptorCount << "\n";
      return 1;
    }
    if (lineSegmentDescriptorCount != 1) {
      std::cerr
          << "Expected the line segment fixture to provide one visible line "
             "renderable descriptor, but extracted "
          << lineSegmentDescriptorCount << "\n";
      return 1;
    }
    if (convexMeshDescriptorCount != 1) {
      std::cerr
          << "Expected the convex mesh fixture to provide one visible convex "
             "mesh renderable descriptor, but extracted "
          << convexMeshDescriptorCount << "\n";
      return 1;
    }
    if (pointCloudDescriptorCount != 1) {
      std::cerr
          << "Expected the point cloud fixture to provide one visible point "
             "cloud renderable descriptor, but extracted "
          << pointCloudDescriptorCount << "\n";
      return 1;
    }
    if (heightmapDescriptorCount != 1) {
      std::cerr
          << "Expected the heightmap fixture to provide one visible heightmap "
             "renderable descriptor, but extracted "
          << heightmapDescriptorCount << "\n";
      return 1;
    }
    if (softMeshDescriptorCount != 1) {
      std::cerr
          << "Expected the soft mesh fixture to provide one visible soft mesh "
             "renderable descriptor, but extracted "
          << softMeshDescriptorCount << "\n";
      return 1;
    }
#if DART_HAVE_OCTOMAP
    if (voxelGridDescriptorCount != 1) {
      std::cerr
          << "Expected the voxel grid fixture to provide one visible voxel "
             "grid renderable descriptor, but extracted "
          << voxelGridDescriptorCount << "\n";
      return 1;
    }
#endif
    if (pbrEnvironmentDescriptorCount < kMinPbrEnvironmentRenderableCount) {
      std::cerr << "Expected the PBR environment fixture to provide at least "
                << kMinPbrEnvironmentRenderableCount
                << " visible mesh renderables, but extracted "
                << pbrEnvironmentDescriptorCount << "\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::G1) {
    if (g1DescriptorCount < kMinG1RenderableCount) {
      std::cerr << "Expected the G1 robot fixture to provide at least "
                << kMinG1RenderableCount
                << " visible mesh renderables, but extracted "
                << g1DescriptorCount << "\n";
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
  std::vector<RenderableId> loggedUnsupportedRenderableIds;
  std::size_t createdWamRenderableCount = 0;
  std::size_t createdAtlasRenderableCount = 0;
  std::size_t createdAtlasRobotRenderableCount = 0;
  std::size_t createdPyramidRenderableCount = 0;
  std::size_t createdMultiSphereRenderableCount = 0;
  std::size_t createdLineSegmentRenderableCount = 0;
  std::size_t createdConvexMeshRenderableCount = 0;
  std::size_t createdPointCloudRenderableCount = 0;
  std::size_t createdHeightmapRenderableCount = 0;
  std::size_t createdSoftMeshRenderableCount = 0;
#if DART_HAVE_OCTOMAP
  std::size_t createdVoxelGridRenderableCount = 0;
#endif
  std::size_t createdPbrEnvironmentRenderableCount = 0;
  std::size_t createdG1RenderableCount = 0;
  std::size_t createdDragAndDropFrameRenderableCount = 0;
  for (const RenderableDescriptor& descriptor : initialDescriptors) {
    if (!descriptor.material.visible) {
      continue;
    }

    auto renderable = createRenderableFromDescriptor(
        *engine, materials, textureCache, descriptor);
    if (!renderable) {
      if (descriptor.geometry.kind == ShapeKind::Unsupported) {
        logUnsupportedRenderableDescriptorOnce(
            loggedUnsupportedRenderableIds, descriptor);
      }
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
    if (descriptor.skeletonName == kPyramidFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Pyramid) {
      ++createdPyramidRenderableCount;
    }
    if (descriptor.skeletonName == kMultiSphereFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::MultiSphere) {
      ++createdMultiSphereRenderableCount;
    }
    if (descriptor.skeletonName == kLineSegmentFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::LineSegments) {
      ++createdLineSegmentRenderableCount;
    }
    if (descriptor.skeletonName == kConvexMeshFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::ConvexMesh) {
      ++createdConvexMeshRenderableCount;
    }
    if (descriptor.skeletonName == kPointCloudFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::PointCloud) {
      ++createdPointCloudRenderableCount;
    }
    if (descriptor.skeletonName == kHeightmapFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Heightmap) {
      ++createdHeightmapRenderableCount;
    }
    if (descriptor.skeletonName == kSoftMeshFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::SoftMesh) {
      ++createdSoftMeshRenderableCount;
    }
#if DART_HAVE_OCTOMAP
    if (descriptor.skeletonName == kVoxelGridFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::VoxelGrid) {
      ++createdVoxelGridRenderableCount;
    }
#endif
    if (descriptor.skeletonName == kPbrEnvironmentFixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdPbrEnvironmentRenderableCount;
    }
    if (descriptor.skeletonName == kG1FixtureSkeletonName
        && descriptor.geometry.kind == ShapeKind::Mesh) {
      ++createdG1RenderableCount;
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
    if (createdPyramidRenderableCount != pyramidDescriptorCount) {
      std::cerr << "Only " << createdPyramidRenderableCount << " of "
                << pyramidDescriptorCount
                << " pyramid renderables were created\n";
      return 1;
    }
    if (createdMultiSphereRenderableCount != multiSphereDescriptorCount) {
      std::cerr << "Only " << createdMultiSphereRenderableCount << " of "
                << multiSphereDescriptorCount
                << " multi-sphere renderables were created\n";
      return 1;
    }
    if (createdLineSegmentRenderableCount != lineSegmentDescriptorCount) {
      std::cerr << "Only " << createdLineSegmentRenderableCount << " of "
                << lineSegmentDescriptorCount
                << " line segment renderables were created\n";
      return 1;
    }
    if (createdConvexMeshRenderableCount != convexMeshDescriptorCount) {
      std::cerr << "Only " << createdConvexMeshRenderableCount << " of "
                << convexMeshDescriptorCount
                << " convex mesh renderables were created\n";
      return 1;
    }
    if (createdPointCloudRenderableCount != pointCloudDescriptorCount) {
      std::cerr << "Only " << createdPointCloudRenderableCount << " of "
                << pointCloudDescriptorCount
                << " point cloud renderables were created\n";
      return 1;
    }
    if (createdHeightmapRenderableCount != heightmapDescriptorCount) {
      std::cerr << "Only " << createdHeightmapRenderableCount << " of "
                << heightmapDescriptorCount
                << " heightmap renderables were created\n";
      return 1;
    }
    if (createdSoftMeshRenderableCount != softMeshDescriptorCount) {
      std::cerr << "Only " << createdSoftMeshRenderableCount << " of "
                << softMeshDescriptorCount
                << " soft mesh renderables were created\n";
      return 1;
    }
#if DART_HAVE_OCTOMAP
    if (createdVoxelGridRenderableCount != voxelGridDescriptorCount) {
      std::cerr << "Only " << createdVoxelGridRenderableCount << " of "
                << voxelGridDescriptorCount
                << " voxel grid renderables were created\n";
      return 1;
    }
#endif
    if (createdPbrEnvironmentRenderableCount < pbrEnvironmentDescriptorCount) {
      std::cerr << "Only " << createdPbrEnvironmentRenderableCount << " of "
                << pbrEnvironmentDescriptorCount
                << " PBR environment mesh renderables were created\n";
      return 1;
    }
  } else if (appOptions.scene == ExampleScene::G1) {
    if (createdG1RenderableCount < g1DescriptorCount) {
      std::cerr << "Only " << createdG1RenderableCount << " of "
                << g1DescriptorCount
                << " G1 robot mesh renderables were created\n";
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
  staticDebugOptions.drawInertiaBoxes = false;
  staticDebugOptions.drawCollisionShapeBounds = false;
  staticDebugOptions.drawSupportPolygons
      = appOptions.scene == ExampleScene::G1;
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
  shadowOptions.mapSize = options.headless ? 2048 : 4096;
  shadowOptions.shadowCascades = options.headless ? 3 : 4;
  shadowOptions.cascadeSplitPositions[0] = 0.10f;
  shadowOptions.cascadeSplitPositions[1] = 0.30f;
  shadowOptions.cascadeSplitPositions[2] = 0.62f;
  shadowOptions.shadowFar = options.headless ? 10.0f : 14.0f;
  shadowOptions.shadowFarHint = options.headless ? 4.5f : 5.5f;
  shadowOptions.screenSpaceContactShadows = false;
  bool orbitLight = appOptions.orbitLight;
  const float3 keyLightDirection
      = orbitLight ? orbitingKeyLightDirection(
                         0.0, appOptions.orbitLightPeriodSeconds)
                   : float3{-0.30f, -0.42f, -1.0f};
  filament::LightManager::Builder(filament::LightManager::Type::SUN)
      .color({1.0f, 0.96f, 0.88f})
      .intensity(82000.0f)
      .direction(keyLightDirection)
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
  imguiStyle.ScaleAllSizes(appOptions.guiScale);
  imguiStyle.WindowRounding = 4.0f * appOptions.guiScale;
  imguiStyle.Colors[ImGuiCol_WindowBg].w = 0.72f;
  auto& imguiIo = ImGui::GetIO();
  loadImGuiFont(imguiIo, appOptions.guiScale);
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
  ProfileAccumulator profile;
  auto lastSimulationClock = ProfileAccumulator::Clock::now();
  const auto orbitStartClock = ProfileAccumulator::Clock::now();
  double simulationAccumulator = 0.0;
  constexpr std::size_t kMaxSimulationStepsPerRenderedFrame = 64;

  while (options.headless || !glfwWindowShouldClose(window)) {
    const auto frameStart = ProfileAccumulator::Clock::now();
    auto phaseStart = ProfileAccumulator::Clock::now();
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

      for (const auto& handle : dartScene.ikHandles) {
        if (glfwGetKey(window, handle.hotkey) == GLFW_PRESS) {
          selectedRenderableId = handle.targetRenderableId;
          selectedLabel = handle.label + " IK target";
          lifecycle.paused = true;
        }
      }
    }
    profile.inputMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
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
        45.0,
        aspect,
        perspectiveNearPlane(cameraController.camera),
        perspectiveFarPlane(cameraController.camera),
        filament::Camera::Fov::VERTICAL);
    if (window != nullptr) {
      double cursorX = 0.0;
      double cursorY = 0.0;
      glfwGetCursorPos(window, &cursorX, &cursorY);
      const bool suppressCameraOrbit
          = leftMouseStartedDrag
            || (appOptions.showUi
                && isInsideStatusPanel(cursorX, cursorY, appOptions.guiScale));
      updateCameraController(window, cameraController, suppressCameraOrbit);
    }
    const Eigen::Vector3d eye = cameraEye(cameraController.camera);
    camera->lookAt(
        {eye.x(), eye.y(), eye.z()},
        {cameraController.camera.target.x(),
         cameraController.camera.target.y(),
         cameraController.camera.target.z()},
        {0.0, 0.0, 1.0});
    profile.viewportCameraMs += elapsedMs(phaseStart);

    std::size_t simulationStepsToRun = 0;
    if (shouldAdvanceSimulation(lifecycle)) {
      if (options.headless || lifecycle.stepOnce) {
        simulationStepsToRun = 1;
      } else {
        const auto now = ProfileAccumulator::Clock::now();
        simulationAccumulator += std::chrono::duration<double>(
                                     now - lastSimulationClock)
                                     .count();
        lastSimulationClock = now;
        const double timeStep = dartScene.world->getTimeStep();
        if (timeStep > 0.0) {
          simulationAccumulator = std::min(
              simulationAccumulator,
              timeStep
                  * static_cast<double>(kMaxSimulationStepsPerRenderedFrame));
          while (simulationStepsToRun < kMaxSimulationStepsPerRenderedFrame
                 && simulationAccumulator + 1e-12 >= timeStep) {
            ++simulationStepsToRun;
            simulationAccumulator -= timeStep;
          }
        }
      }
    } else {
      lastSimulationClock = ProfileAccumulator::Clock::now();
      simulationAccumulator = 0.0;
    }

    if (simulationStepsToRun > 0) {
      phaseStart = ProfileAccumulator::Clock::now();
      for (std::size_t i = 0; i < simulationStepsToRun; ++i) {
        const double timeStep = dartScene.world->getTimeStep();
        dartScene.world->step();
        profile.simulatedMs += timeStep * 1000.0;
      }
      markSimulationAdvanced(lifecycle);
      profile.simulationSteps += simulationStepsToRun;
      profile.simulationMs += elapsedMs(phaseStart);

      phaseStart = ProfileAccumulator::Clock::now();
      refreshContactDebugOverlay();
      profile.contactDebugMs += elapsedMs(phaseStart);
    }

    auto& transforms = engine->getTransformManager();
    phaseStart = ProfileAccumulator::Clock::now();
    auto descriptors = extractRenderables(*dartScene.world);
    profile.extractionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
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
            && translateRenderableAndApplyIk(
                dartScene, *selectedDescriptor, nudge)) {
          lifecycle.paused = true;
          descriptors = extractRenderables(*dartScene.world);
        }
      }
    }
    profile.interactionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    synchronizeSceneRenderables(
        *engine,
        *scene,
        materials,
        textureCache,
        descriptors,
        sceneRenderables,
        loggedUnsupportedRenderableIds);
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
    profile.syncMs += elapsedMs(phaseStart);

    if (orbitLight) {
      const double orbitElapsedSeconds = std::chrono::duration<double>(
                                             ProfileAccumulator::Clock::now()
                                             - orbitStartClock)
                                             .count();
      auto& lights = engine->getLightManager();
      lights.setDirection(
          lights.getInstance(lightEntity),
          orbitingKeyLightDirection(
              orbitElapsedSeconds, appOptions.orbitLightPeriodSeconds));
    }

    phaseStart = ProfileAccumulator::Clock::now();
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
            = appOptions.showUi
              && isInsideStatusPanel(cursorX, cursorY, appOptions.guiScale);

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
              && translateRenderableAndApplyIk(
                  dartScene, *selectedDescriptor, *translation)) {
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
            selectedLabel = selectionLabelForRenderable(dartScene, descriptor);
          } else {
            selectedRenderableId = 0;
            selectedLabel = "none";
          }
        }
        leftMouseStartedDrag = false;
      }
      wasLeftMousePressed = isLeftMousePressed;
    }
    profile.interactionMs += elapsedMs(phaseStart);

    phaseStart = ProfileAccumulator::Clock::now();
    refreshSelectionDebugOverlay(descriptors, selectedRenderableId);
    profile.selectionDebugMs += elapsedMs(phaseStart);

    if (appOptions.showUi) {
      phaseStart = ProfileAccumulator::Clock::now();
      updateImGuiMouseInput(window, imguiIo, width, height);
      ImGui::NewFrame();
      ImGui::SetNextWindowPos(
          {20.0f * appOptions.guiScale, 20.0f * appOptions.guiScale},
          ImGuiCond_Always);
      ImGui::SetNextWindowBgAlpha(0.72f);
      ImGui::Begin(
          "DART",
          nullptr,
          ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
      ImGui::PushTextWrapPos(
          ImGui::GetCursorPosX() + 300.0f * appOptions.guiScale);
      ImGui::TextWrapped(
          "DART scene viewer: inspect renderables, shadows, and debug overlays.");
      ImGui::TextWrapped(
          "Mouse: left orbit, right/middle pan, wheel zoom, click select.");
      ImGui::TextWrapped(
          "Keys: Space pause, N step, arrows/Pg or Ctrl-left drag selected, "
          "Esc exit.");
      if (!dartScene.ikHandles.empty()) {
        ImGui::TextWrapped(
            "G1 IK: press 1-4 or click a colored target, then move it.");
      }
      ImGui::PopTextWrapPos();
      ImGui::Separator();
      ImGui::Text("scene: %s", sceneName(appOptions.scene));
      ImGui::Text("time: %.3f", dartScene.world->getTime());
      ImGui::Text(
          "contacts: %zu",
          dartScene.world->getLastCollisionResult().getNumContacts());
      ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 300.0f * appOptions.guiScale);
      ImGui::Text("selected: %s", selectedLabel.c_str());
      ImGui::PopTextWrapPos();
      if (ImGui::Button(lifecycle.paused ? "Resume" : "Pause")) {
        togglePaused(lifecycle);
      }
      ImGui::SameLine();
      if (ImGui::Button("Step")) {
        requestSingleStep(lifecycle);
      }
      ImGui::SameLine();
      ImGui::Checkbox("Orbit light", &orbitLight);
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
      debugOptionsChanged |= ImGui::Checkbox(
          "Inertia", &staticDebugOptions.drawInertiaBoxes);
      ImGui::SameLine();
      debugOptionsChanged |= ImGui::Checkbox(
          "Collision", &staticDebugOptions.drawCollisionShapeBounds);
      ImGui::SameLine();
      debugOptionsChanged
          |= ImGui::Checkbox("Contacts", &contactDebugOptions.drawContacts);
      ImGui::SameLine();
      debugOptionsChanged |= ImGui::Checkbox(
          "Support", &staticDebugOptions.drawSupportPolygons);
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
      profile.uiMs += elapsedMs(phaseStart);
    }

    const bool shouldCaptureScreenshot
        = shouldRequestScreenshot(options, lifecycle);

    const auto renderFrameStart = ProfileAccumulator::Clock::now();
    const bool shouldRenderFrame = renderer->beginFrame(swapChain);
    profile.beginFrameMs += elapsedMs(renderFrameStart);
    if (!shouldRenderFrame) {
      markFrameSkipped(lifecycle);
      ++profile.skippedFrames;
      if (!options.headless || !renderer->shouldRenderFrame()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const double frameMs = elapsedMs(frameStart);
        profile.frameMs += frameMs;
        profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
        ++profile.frames;
        continue;
      }
      // Filament allows callers to ignore a pacing-only false return. Headless
      // software GL can report skips almost every frame, so keep deterministic
      // offscreen captures while still skipping backend failures above.
    }

    phaseStart = ProfileAccumulator::Clock::now();
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
    ++profile.renderedFrames;
    const double renderMs = elapsedMs(phaseStart);
    profile.renderMs += renderMs;
    profile.maxRenderMs = std::max(profile.maxRenderMs, renderMs);

    const double frameMs = elapsedMs(frameStart);
    profile.frameMs += frameMs;
    profile.maxFrameMs = std::max(profile.maxFrameMs, frameMs);
    ++profile.frames;
    markFrameRendered(lifecycle);
    if (shouldStopAfterFrame(options, lifecycle)) {
      break;
    }
  }

  if (!options.screenshotPath.empty() && !lifecycle.screenshotRequested) {
    std::cerr << "No rendered frame was available for screenshot capture\n";
  }
  if (lifecycle.screenshotRequested) {
    const auto screenshotWaitStart = ProfileAccumulator::Clock::now();
    screenshotSucceeded = waitForScreenshot(*engine, screenshotCapture);
    profile.screenshotWaitMs += elapsedMs(screenshotWaitStart);
    if (screenshotSucceeded) {
      const auto screenshotSaveStart = ProfileAccumulator::Clock::now();
      saveScreenshot(screenshotCapture, options.screenshotPath);
      profile.screenshotSaveMs += elapsedMs(screenshotSaveStart);
    } else {
      std::cerr << "Timed out waiting for Filament screenshot readback\n";
    }
  }
  if (options.maxFrames >= 0) {
    std::cout << "Final contacts: "
              << dartScene.world->getLastCollisionResult().getNumContacts()
              << "\n";
  }
  if (appOptions.profile) {
    printProfile(profile);
    DART_PROFILE_TEXT_DUMP();
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
  return screenshotSucceeded ? 0 : 1;
}
