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
#include "textured_lit_material.hpp"
#include "transparent_lit_material.hpp"
#include "transparent_textured_lit_material.hpp"

#include "scenes.hpp"

#include <dart/all.hpp>
#include <dart/common/local_resource_retriever.hpp>
#include <dart/common/profile.hpp>
#include <dart/config.hpp>
#include <dart/gui/experimental/geometry.hpp>
#include <dart/gui/experimental/scene.hpp>
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
using dart::dynamics::SphereShape;
using dart::dynamics::VisualAspect;
using dart::dynamics::WeldJoint;

#if DART_HAVE_OCTOMAP
using dart::dynamics::VoxelGridShape;
#endif

using dart::gui::experimental::ActiveRenderableState;
using dart::gui::experimental::DebugDrawOptions;
using dart::gui::experimental::DebugLineDescriptor;
using dart::gui::experimental::DirectionalNudgeInput;
using dart::gui::experimental::GeometryDescriptor;
using dart::gui::experimental::MaterialDescriptor;
using dart::gui::experimental::MeshGeometry;
using dart::gui::experimental::MeshIndexRange;
using dart::gui::experimental::MeshAlphaMode;
using dart::gui::experimental::MeshMaterialDescriptor;
using dart::gui::experimental::MeshPartDescriptor;
using dart::gui::experimental::OrbitCamera;
using dart::gui::experimental::OrbitCameraController;
using dart::gui::experimental::OrbitCameraControllerInput;
using dart::gui::experimental::PickRay;
using dart::gui::experimental::RenderableDescriptor;
using dart::gui::experimental::RenderableId;
using dart::gui::experimental::RunOptions;
using dart::gui::experimental::ShapeKind;
using dart::gui::experimental::ViewerLifecycleState;
using dart::gui::experimental::addOrbitCameraScroll;
using dart::gui::experimental::cameraEye;
using dart::gui::experimental::computeCameraRelativeNudge;
using dart::gui::experimental::computePlaneDragTranslation;
using dart::gui::experimental::extractContactDebugLines;
using dart::gui::experimental::extractDebugLines;
using dart::gui::experimental::extractRenderables;
using dart::gui::experimental::intersectPlane;
using dart::gui::experimental::markFrameRendered;
using dart::gui::experimental::markFrameSkipped;
using dart::gui::experimental::markScreenshotRequested;
using dart::gui::experimental::markSimulationAdvanced;
using dart::gui::experimental::appendBoxMeshGeometry;
using dart::gui::experimental::makeCapsuleMeshGeometry;
using dart::gui::experimental::makeConeMeshGeometry;
using dart::gui::experimental::makeCylinderMeshGeometry;
using dart::gui::experimental::makeEllipsoidMeshGeometry;
using dart::gui::experimental::makeMultiSphereMeshGeometry;
using dart::gui::experimental::makePyramidMeshGeometry;
using dart::gui::experimental::makeOrbitCameraBasis;
using dart::gui::experimental::makePerspectiveProjection;
using dart::gui::experimental::makePerspectivePickRay;
using dart::gui::experimental::makeRenderableId;
using dart::gui::experimental::makeSelectionDebugLines;
using dart::gui::experimental::normalizeRunOptions;
using dart::gui::experimental::pickNearestRenderable;
using dart::gui::experimental::planRenderableSetUpdate;
using dart::gui::experimental::requestSingleStep;
using dart::gui::experimental::shouldAdvanceSimulation;
using dart::gui::experimental::shouldRequestScreenshot;
using dart::gui::experimental::shouldStopAfterFrame;
using dart::gui::experimental::togglePaused;
using dart::gui::experimental::translateFrameRenderable;
using dart::gui::experimental::updateOrbitCameraController;
using dart::gui::experimental::writeRgbaPpm;
using dart::simulation::World;
using dart::examples::filament_gui::AppOptions;
using dart::examples::filament_gui::DartScene;
using dart::examples::filament_gui::ExampleScene;
using dart::examples::filament_gui::G1IkHandle;
using dart::examples::filament_gui::createDartScene;
using dart::examples::filament_gui::initialCameraForScene;
using dart::examples::filament_gui::kAtlasFixtureSkeletonName;
using dart::examples::filament_gui::kAtlasRobotFixtureSkeletonName;
using dart::examples::filament_gui::kConvexMeshFixtureSkeletonName;
using dart::examples::filament_gui::kG1FixtureSkeletonName;
using dart::examples::filament_gui::kHeightmapFixtureSkeletonName;
using dart::examples::filament_gui::kLineSegmentFixtureSkeletonName;
using dart::examples::filament_gui::kMinDragAndDropFrameRenderableCount;
using dart::examples::filament_gui::kMinG1RenderableCount;
using dart::examples::filament_gui::kMinPbrEnvironmentRenderableCount;
using dart::examples::filament_gui::kMultiSphereFixtureSkeletonName;
using dart::examples::filament_gui::kPbrEnvironmentFixtureSkeletonName;
using dart::examples::filament_gui::kPointCloudFixtureSkeletonName;
using dart::examples::filament_gui::kPyramidFixtureSkeletonName;
using dart::examples::filament_gui::kSoftMeshFixtureSkeletonName;
using dart::examples::filament_gui::kVoxelGridFixtureSkeletonName;
using dart::examples::filament_gui::kWamFixtureSkeletonName;
using dart::examples::filament_gui::parseOptions;
using dart::examples::filament_gui::sceneName;
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
  std::size_t shapeVersion = 0;
  std::size_t renderResourceVersion = 0;
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
      = static_cast<OrbitCameraController*>(glfwGetWindowUserPointer(window));
  if (controller != nullptr) {
    addOrbitCameraScroll(*controller, yOffset);
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

bool isInsideStatusPanel(double cursorX, double cursorY, double guiScale)
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
  if (window == nullptr) {
    return Eigen::Vector3d::Zero();
  }

  DirectionalNudgeInput input;
  input.left = isKeyDown(window, GLFW_KEY_LEFT);
  input.right = isKeyDown(window, GLFW_KEY_RIGHT);
  input.forward = isKeyDown(window, GLFW_KEY_UP);
  input.backward = isKeyDown(window, GLFW_KEY_DOWN);
  input.up = isKeyDown(window, GLFW_KEY_PAGE_UP);
  input.down = isKeyDown(window, GLFW_KEY_PAGE_DOWN);
  input.fast = isKeyDown(window, GLFW_KEY_LEFT_SHIFT)
               || isKeyDown(window, GLFW_KEY_RIGHT_SHIFT);
  input.stepSize = stepSize;
  return computeCameraRelativeNudge(camera, input);
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

float3 toFloat3(const Eigen::Vector3d& vector)
{
  return {
      static_cast<float>(vector.x()),
      static_cast<float>(vector.y()),
      static_cast<float>(vector.z())};
}

float3 toFloat3f(const Eigen::Vector3f& vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

float2 toFloat2f(const Eigen::Vector2f& vector)
{
  return {vector.x(), vector.y()};
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

float resolveMeshMaterialAlpha(
    float visualAlpha, float materialAlpha, MeshAlphaMode alphaMode)
{
  switch (alphaMode) {
    case MeshAlphaMode::Blend:
      return visualAlpha * materialAlpha;
    case MeshAlphaMode::Auto:
      return materialAlpha;
    case MeshAlphaMode::ShapeAlpha:
      return visualAlpha;
  }

  return visualAlpha * materialAlpha;
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
    OrbitCameraController& controller,
    bool suppressLeftMouseOrbit = false)
{
  if (window == nullptr) {
    return;
  }

  double x = 0.0;
  double y = 0.0;
  glfwGetCursorPos(window, &x, &y);

  OrbitCameraControllerInput input;
  input.cursorX = x;
  input.cursorY = y;
  input.orbit = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS
                && !suppressLeftMouseOrbit && !isDragModifierDown(window);
  input.pan
      = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS
        || glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  updateOrbitCameraController(controller, input);
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

void applyRenderableShadowSettings(
    filament::Engine& engine,
    const Renderable& renderable,
    const MaterialDescriptor& material)
{
  auto& renderables = engine.getRenderableManager();
  const auto instance = renderables.getInstance(renderable.entity);
  renderables.setCastShadows(instance, material.castsShadows);
  renderables.setReceiveShadows(instance, material.receivesShadows);
  renderables.setScreenSpaceContactShadows(
      instance, material.castsShadows || material.receivesShadows);
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

struct TriangleMeshBuffers
{
  std::vector<Vertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  std::vector<float3> normals;
  Bounds bounds{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
};

TriangleMeshBuffers makeTriangleMeshBuffers(MeshGeometry&& geometry)
{
  const filament::math::short4 tangent = {0, 0, 0, 32767};
  TriangleMeshBuffers buffers;
  buffers.vertices.reserve(geometry.vertices.size());
  buffers.normals.reserve(geometry.vertices.size());
  for (const auto& vertex : geometry.vertices) {
    buffers.vertices.push_back(
        Vertex{toFloat3f(vertex.position), tangent, toFloat2f(vertex.uv)});
    buffers.normals.push_back(toFloat3f(vertex.normal));
  }

  buffers.indices = std::move(geometry.indices);
  buffers.triangles.reserve(geometry.triangles.size());
  for (const auto& triangle : geometry.triangles) {
    buffers.triangles.push_back({triangle.a, triangle.b, triangle.c});
  }

  buffers.bounds = {
      toFloat3f(geometry.boundsMin), toFloat3f(geometry.boundsMax)};
  return buffers;
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

Renderable createGeometryMeshRenderable(
    filament::Engine& engine,
    filament::Material& material,
    MeshGeometry geometry,
    const float4& color)
{
  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  return createTriangleMeshRenderable(
      engine,
      material,
      std::move(buffers.vertices),
      std::move(buffers.indices),
      std::move(buffers.triangles),
      std::move(buffers.normals),
      color,
      buffers.bounds.min,
      buffers.bounds.max);
}

Renderable createEllipsoidRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const Eigen::Vector3d& radii,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeEllipsoidMeshGeometry(radii), color);
}

Renderable createMultiSphereRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const std::vector<Eigen::Vector3d>& centers,
    const std::vector<double>& radii,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeMultiSphereMeshGeometry(centers, radii), color);
}

Renderable createCylinderRenderable(
    filament::Engine& engine,
    filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeCylinderMeshGeometry(radius, height), color);
}

Renderable createConeRenderable(
    filament::Engine& engine,
    filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeConeMeshGeometry(radius, height), color);
}

Renderable createPyramidRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const Eigen::Vector3d& size,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makePyramidMeshGeometry(size), color);
}

Renderable createCapsuleRenderable(
    filament::Engine& engine,
    filament::Material& material,
    double radius,
    double height,
    const float4& color)
{
  return createGeometryMeshRenderable(
      engine, material, makeCapsuleMeshGeometry(radius, height), color);
}

std::optional<Renderable> createDescriptorTriangleMeshRenderable(
    filament::Engine& engine,
    filament::Material& material,
    const GeometryDescriptor& geometry,
    const float4& color)
{
  if (geometry.triangleVertices.empty() || geometry.triangleIndices.empty()) {
    return std::nullopt;
  }
  if (geometry.triangleVertices.size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const filament::math::short4 tangent = {0, 0, 0, 32767};
  std::vector<Vertex> vertices;
  vertices.reserve(geometry.triangleVertices.size());
  for (const Eigen::Vector3d& vertex : geometry.triangleVertices) {
    if (!vertex.allFinite()) {
      return std::nullopt;
    }
    vertices.push_back(Vertex{toFloat3(vertex), tangent});
  }

  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  indices.reserve(geometry.triangleIndices.size() * 3u);
  triangles.reserve(geometry.triangleIndices.size());
  for (const Eigen::Vector3i& triangle : geometry.triangleIndices) {
    if ((triangle.array() < 0).any()) {
      return std::nullopt;
    }
    const auto first = static_cast<std::size_t>(triangle.x());
    const auto second = static_cast<std::size_t>(triangle.y());
    const auto third = static_cast<std::size_t>(triangle.z());
    if (first >= vertices.size() || second >= vertices.size()
        || third >= vertices.size()) {
      return std::nullopt;
    }
    appendTriangle(
        indices,
        triangles,
        static_cast<std::uint32_t>(first),
        static_cast<std::uint32_t>(second),
        static_cast<std::uint32_t>(third));
  }
  if (indices.empty()) {
    return std::nullopt;
  }

  std::vector<float3> normals;
  if (geometry.triangleNormals.size() == vertices.size()) {
    normals.reserve(vertices.size());
    for (const Eigen::Vector3d& sourceNormal : geometry.triangleNormals) {
      if (!sourceNormal.allFinite() || sourceNormal.squaredNorm() <= 1e-12) {
        normals.clear();
        break;
      }
      normals.push_back(toFloat3(sourceNormal.normalized()));
    }
  }

  if (normals.size() != vertices.size()) {
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

    normals.clear();
    normals.reserve(normalSums.size());
    for (const float3& normal : normalSums) {
      normals.push_back(normalizeOr(normal, {0.0f, 0.0f, 1.0f}));
    }
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
  MeshGeometry geometry;
  std::vector<MeshIndexRange> pointRanges;
  geometry.vertices.reserve(descriptor.geometry.pointCloudPoints.size() * 24u);
  geometry.indices.reserve(descriptor.geometry.pointCloudPoints.size() * 36u);
  geometry.triangles.reserve(descriptor.geometry.pointCloudPoints.size() * 12u);
  pointRanges.reserve(descriptor.geometry.pointCloudPoints.size());

  for (const Eigen::Vector3d& point : descriptor.geometry.pointCloudPoints) {
    pointRanges.push_back(appendBoxMeshGeometry(
        geometry, point, Eigen::Vector3d::Constant(pointSize)));
  }

  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  const Bounds bounds = buffers.bounds;
  const bool usePerPointColors
      = descriptor.geometry.pointCloudColors.size()
        == descriptor.geometry.pointCloudPoints.size();
  if (usePerPointColors) {
    generateTangentFrames(buffers.vertices, buffers.triangles, buffers.normals);

    Renderable renderable;
    renderable.vertexBuffer
        = filament::VertexBuffer::Builder()
              .vertexCount(buffers.vertices.size())
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
        engine, 0, makeBufferDescriptor(std::move(buffers.vertices)));

    renderable.indexBuffer
        = filament::IndexBuffer::Builder()
              .indexCount(buffers.indices.size())
              .bufferType(filament::IndexBuffer::IndexType::UINT)
              .build(engine);
    renderable.indexBuffer->setBuffer(
        engine, makeBufferDescriptor(std::move(buffers.indices)));

    renderable.entity = EntityManager::get().create();
    auto builder = filament::RenderableManager::Builder(pointRanges.size());
    builder.boundingBox({bounds.min, bounds.max})
        .castShadows(true)
        .receiveShadows(true);
    for (std::size_t i = 0; i < pointRanges.size(); ++i) {
      const float4 pointColor = ensureReadableDisplayColor(
          toRgba(descriptor.geometry.pointCloudColors[i]));
      auto* materialInstance = addRenderableMaterial(
          renderable,
          selectLitMaterial(materials, false, pointColor),
          pointColor,
          false);
      configureLitMaterialInstance(
          *materialInstance, pointColor, 0.0f, 0.58f, {0.0f, 0.0f, 0.0f});
      builder.material(i, materialInstance)
          .geometry(
              i,
              filament::RenderableManager::PrimitiveType::TRIANGLES,
              renderable.vertexBuffer,
              renderable.indexBuffer,
              pointRanges[i].indexOffset,
              pointRanges[i].indexCount);
    }
    builder.build(engine, renderable.entity);

    auto& renderables = engine.getRenderableManager();
    renderables.setScreenSpaceContactShadows(
        renderables.getInstance(renderable.entity), true);
    return renderable;
  }

  return createTriangleMeshRenderable(
      engine,
      selectLitMaterial(materials, false, color),
      std::move(buffers.vertices),
      std::move(buffers.indices),
      std::move(buffers.triangles),
      std::move(buffers.normals),
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
  MeshGeometry geometry;
  geometry.vertices.reserve(descriptor.geometry.voxelCenters.size() * 24u);
  geometry.indices.reserve(descriptor.geometry.voxelCenters.size() * 36u);
  geometry.triangles.reserve(descriptor.geometry.voxelCenters.size() * 12u);

  for (const Eigen::Vector3d& center : descriptor.geometry.voxelCenters) {
    appendBoxMeshGeometry(
        geometry, center, Eigen::Vector3d::Constant(voxelSize));
  }

  auto buffers = makeTriangleMeshBuffers(std::move(geometry));
  const Bounds bounds = buffers.bounds;
  return createTriangleMeshRenderable(
      engine,
      selectLitMaterial(materials, false, color),
      std::move(buffers.vertices),
      std::move(buffers.indices),
      std::move(buffers.triangles),
      std::move(buffers.normals),
      color,
      bounds.min,
      bounds.max);
}

std::optional<Renderable> createMeshRenderable(
    filament::Engine& engine,
    const MaterialSet& materials,
    TextureCache& textureCache,
    const GeometryDescriptor& geometry,
    const float4& color)
{
  const auto& meshVertices = geometry.triangleVertices;
  const auto& meshTriangles = geometry.triangleIndices;
  if (meshVertices.empty() || meshTriangles.empty()) {
    return std::nullopt;
  }
  if (meshVertices.size()
      > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return std::nullopt;
  }

  const filament::math::short4 tangent = {0, 0, 0, 32767};
  const bool hasTextureCoords
      = geometry.meshTextureCoordComponents >= 2
        && geometry.meshTextureCoordinates.size() == meshVertices.size();

  std::vector<Vertex> vertices;
  vertices.reserve(meshVertices.size());
  for (std::size_t i = 0; i < meshVertices.size(); ++i) {
    if (!meshVertices[i].allFinite()) {
      return std::nullopt;
    }

    filament::math::float2 uv = {0.0f, 0.0f};
    if (hasTextureCoords) {
      const Eigen::Vector3d& textureCoordinate
          = geometry.meshTextureCoordinates[i];
      if (!textureCoordinate.allFinite()) {
        return std::nullopt;
      }
      uv = {
          static_cast<float>(textureCoordinate.x()),
          static_cast<float>(textureCoordinate.y())};
    }
    vertices.push_back(Vertex{toFloat3(meshVertices[i]), tangent, uv});
  }

  std::vector<float3> normals;
  if (geometry.triangleNormals.size() == meshVertices.size()) {
    normals.reserve(meshVertices.size());
    for (const Eigen::Vector3d& sourceNormal : geometry.triangleNormals) {
      if (!sourceNormal.allFinite() || sourceNormal.squaredNorm() <= 1e-12) {
        normals.clear();
        break;
      }
      normals.push_back(toFloat3(sourceNormal.normalized()));
    }
  }

  std::vector<std::uint32_t> indices;
  std::vector<filament::math::uint3> triangles;
  indices.reserve(meshTriangles.size() * 3);
  triangles.reserve(meshTriangles.size());
  for (const Eigen::Vector3i& triangle : meshTriangles) {
    if ((triangle.array() < 0).any()) {
      return std::nullopt;
    }

    const auto first = static_cast<std::size_t>(triangle.x());
    const auto second = static_cast<std::size_t>(triangle.y());
    const auto third = static_cast<std::size_t>(triangle.z());
    if (first >= vertices.size() || second >= vertices.size()
        || third >= vertices.size()) {
      return std::nullopt;
    }
    appendTriangle(
        indices,
        triangles,
        static_cast<std::uint32_t>(first),
        static_cast<std::uint32_t>(second),
        static_cast<std::uint32_t>(third));
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
      state.baseColor.w = resolveMeshMaterialAlpha(
          color.w, state.baseColor.w, geometry.meshAlphaMode);
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
    if (range.triangleOffset + range.triangleCount > meshTriangles.size()) {
      useSubMeshes = false;
      break;
    }
    coveredTriangleCount += range.triangleCount;
    parts.push_back(range);
  }
  useSubMeshes = useSubMeshes && !parts.empty()
                 && coveredTriangleCount == meshTriangles.size();
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
    case ShapeKind::Heightmap:
    case ShapeKind::SoftMesh:
      renderable = createDescriptorTriangleMeshRenderable(
          engine, solidMaterial, descriptor.geometry, color);
      break;
    case ShapeKind::PointCloud:
      renderable = createPointCloudRenderable(engine, materials, descriptor);
      break;
    case ShapeKind::VoxelGrid:
      renderable = createVoxelGridRenderable(engine, materials, descriptor);
      break;
    case ShapeKind::Mesh:
      renderable = createMeshRenderable(
          engine, materials, textureCache, descriptor.geometry, color);
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
    applyRenderableShadowSettings(engine, *renderable, descriptor.material);
  }

  return renderable;
}

void logUnsupportedRenderableDescriptor(const RenderableDescriptor& descriptor)
{
  std::cerr << "DART shape";
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

std::vector<ActiveRenderableState> collectActiveRenderableStates(
    const std::vector<SceneRenderable>& sceneRenderables)
{
  std::vector<ActiveRenderableState> states;
  states.reserve(sceneRenderables.size());
  for (const SceneRenderable& sceneRenderable : sceneRenderables) {
    ActiveRenderableState state;
    state.id = sceneRenderable.id;
    state.shapeVersion = sceneRenderable.shapeVersion;
    state.renderResourceVersion = sceneRenderable.renderResourceVersion;
    states.push_back(state);
  }
  return states;
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
      descriptors, collectActiveRenderableStates(sceneRenderables));

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
      if (descriptor.geometry.kind == ShapeKind::Unsupported
          || !descriptor.geometry.unsupportedReason.empty()) {
        logUnsupportedRenderableDescriptorOnce(
            loggedUnsupportedRenderableIds, descriptor);
      }
      continue;
    }

    SceneRenderable sceneRenderable;
    sceneRenderable.id = descriptor.id;
    sceneRenderable.shapeVersion = descriptor.shapeVersion;
    sceneRenderable.renderResourceVersion = descriptor.renderResourceVersion;
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

  OrbitCameraController cameraController;
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
  const float guiScale = static_cast<float>(options.guiScale);
  imguiStyle.ScaleAllSizes(guiScale);
  imguiStyle.WindowRounding = 4.0f * guiScale;
  imguiStyle.Colors[ImGuiCol_WindowBg].w = 0.72f;
  auto& imguiIo = ImGui::GetIO();
  loadImGuiFont(imguiIo, guiScale);
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
    const auto projection
        = makePerspectiveProjection(cameraController.camera, width, height);
    camera->setProjection(
        projection.verticalFovDegrees,
        projection.aspectRatio,
        projection.nearPlane,
        projection.farPlane,
        filament::Camera::Fov::VERTICAL);
    if (window != nullptr) {
      double cursorX = 0.0;
      double cursorY = 0.0;
      glfwGetCursorPos(window, &cursorX, &cursorY);
      const bool suppressCameraOrbit
          = leftMouseStartedDrag
            || (appOptions.showUi
                && isInsideStatusPanel(cursorX, cursorY, options.guiScale));
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
      applyRenderableShadowSettings(
          *engine, sceneRenderable.renderable, descriptor->material);
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
              && isInsideStatusPanel(cursorX, cursorY, options.guiScale);

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
          {20.0f * guiScale, 20.0f * guiScale},
          ImGuiCond_Always);
      ImGui::SetNextWindowBgAlpha(0.72f);
      ImGui::Begin(
          "DART",
          nullptr,
          ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
      ImGui::PushTextWrapPos(
          ImGui::GetCursorPosX() + 300.0f * guiScale);
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
      ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 300.0f * guiScale);
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
