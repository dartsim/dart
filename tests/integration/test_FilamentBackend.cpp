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

#include <dart/gui/DebugOverlay.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>
#include <dart/gui/detail/FilamentRenderer.hpp>
#include <dart/gui/detail/imgui_draw_data.hpp>
#include <dart/gui/detail/imgui_overlay.hpp>
#include <dart/gui/detail/key_input.hpp>
#include <dart/gui/detail/native_window.hpp>
#include <dart/gui/detail/render_context.hpp>
#include <dart/gui/detail/render_environment.hpp>
#include <dart/gui/detail/renderable_factory.hpp>
#include <dart/gui/detail/renderable_resources.hpp>
#define GLFW_INCLUDE_NONE
#include <dart/simulation/World.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>

#include <dart/common/Deprecated.hpp>

#include <GLFW/glfw3.h>
#include <assimp/material.h>
#include <assimp/scene.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Color.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/TextureSampler.h>
#include <filament/ToneMapper.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <gtest/gtest.h>
#include <jpeglib.h>
#include <png.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
// Lighting fixtures specify physical linear intensities. Encode their base
// colors through the SDK because DART's authored surface colors are display
// RGB.
Eigen::Vector4d displayColorForLinearFixture(const Eigen::Vector4d& linear)
{
  const auto encoded
      = ::filament::Color::toSRGB<::filament::ColorConversion::ACCURATE>(
          ::filament::LinearColorA{
              static_cast<float>(linear.x()),
              static_cast<float>(linear.y()),
              static_cast<float>(linear.z()),
              static_cast<float>(linear.w())});
  return {encoded.x, encoded.y, encoded.z, encoded.w};
}
struct CaptureFile
{
  std::filesystem::path path
      = std::filesystem::temp_directory_path()
        / ("dart-filament-"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count())
           + ".png");
  ~CaptureFile()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }
};
std::vector<unsigned char> readCapture(
    const CaptureFile& file, unsigned width, unsigned height)
{
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  if (!png_image_begin_read_from_file(&image, file.path.string().c_str()))
    throw std::runtime_error(image.message);
  if (image.width != width || image.height != height) {
    png_image_free(&image);
    throw std::runtime_error(
        "Capture dimensions differ from the requested physical pixels");
  }
  image.format = PNG_FORMAT_RGBA;
  std::vector<unsigned char> pixels(PNG_IMAGE_SIZE(image));
  if (!png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr)) {
    const std::string error = image.message;
    png_image_free(&image);
    throw std::runtime_error(error);
  }
  png_image_free(&image);
  return pixels;
}
bool preserveCapture(
    const char* name,
    const std::vector<unsigned char>& pixels,
    unsigned width,
    unsigned height)
{
  const auto* directory = std::getenv("DART_FILAMENT_PARITY_DIR");
  if (!directory)
    return true;
  std::filesystem::create_directories(directory);
  const auto file = std::filesystem::path(directory) / name;
  png_image png{};
  png.version = PNG_IMAGE_VERSION;
  png.width = width;
  png.height = height;
  png.format = PNG_FORMAT_RGBA;
  return png_image_write_to_file(
      &png, file.string().c_str(), 0, pixels.data(), 0, nullptr);
}
bool writeSolidJpeg(
    const std::filesystem::path& path,
    const std::array<unsigned char, 3>& color)
{
  auto* file = std::fopen(path.string().c_str(), "wb");
  if (!file)
    return false;
  jpeg_compress_struct info{};
  jpeg_error_mgr error{};
  info.err = jpeg_std_error(&error);
  jpeg_create_compress(&info);
  jpeg_stdio_dest(&info, file);
  info.image_width = info.image_height = 16;
  info.input_components = 3;
  info.in_color_space = JCS_RGB;
  jpeg_set_defaults(&info);
  jpeg_set_quality(&info, 100, TRUE);
  jpeg_start_compress(&info, TRUE);
  std::array<unsigned char, 16 * 3> row{};
  for (unsigned x = 0; x < 16; ++x)
    std::copy(color.begin(), color.end(), row.begin() + x * 3);
  while (info.next_scanline < info.image_height) {
    JSAMPROW data = row.data();
    jpeg_write_scanlines(&info, &data, 1);
  }
  jpeg_finish_compress(&info);
  jpeg_destroy_compress(&info);
  return std::fclose(file) == 0;
}
void expectRedAboveCenter(
    const std::vector<unsigned char>& pixels, unsigned width, unsigned height)
{
  double sum = 0;
  std::size_t count = 0;
  for (unsigned y = 0; y < height; ++y)
    for (unsigned x = 0; x < width; ++x) {
      const auto pixel = (static_cast<std::size_t>(y) * width + x) * 4;
      if (pixels[pixel] > pixels[pixel + 2] + 30) {
        sum += y;
        ++count;
      }
    }
  ASSERT_GT(count, 20u);
  EXPECT_LT(sum / count, height * 0.5 - 5.0)
      << "World +Z must appear above image center in top-left RGBA";
}
} // namespace

TEST(FilamentBackend, BackgroundPreservesDisplayColor)
{
  constexpr unsigned width = 64, height = 48;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setBackgroundColor({.58, .62, .65, 1});
  const auto image = renderer.readPixelsRGBA();
  const std::array<unsigned char, 4> expected{148, 158, 166, 255};
  for (std::size_t pixel = 0; pixel < image.size(); pixel += 4)
    for (unsigned channel = 0; channel < 4; ++channel)
      EXPECT_EQ(image[pixel + channel], expected[channel]);
  ASSERT_TRUE(
      preserveCapture("background-display-color.png", image, width, height));
  // A scene object must still receive lighting and the scene tone mapper.
  dart::gui::RenderableDescriptor box;
  box.id = 1;
  box.geometry.kind = dart::gui::ShapeKind::Box;
  box.geometry.size = Eigen::Vector3d::Ones();
  box.material.rgba = {.9, .05, .05, 1};
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  renderer.synchronize({box});
  const auto scene = renderer.readPixelsRGBA();
  const auto center = (height / 2 * width + width / 2) * 4;
  EXPECT_GT(scene[center], scene[center + 2] + 30);
  for (unsigned channel = 0; channel < 4; ++channel)
    EXPECT_EQ(scene[channel], expected[channel]);
}

TEST(FilamentBackend, QueuedCaptureKeepsCompletedFrameAndLatestUploads)
{
  struct ImGuiScope
  {
    ImGuiContext* previous = ImGui::GetCurrentContext();
    ImGuiContext* owned = ImGui::CreateContext();
    ~ImGuiScope()
    {
      ImGui::DestroyContext(owned);
      ImGui::SetCurrentContext(previous);
    }
  } scope;
  unsigned width = 64, height = 48;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 60, .01, 1000);
  // Initializing fonts creates an overlay View before any draw mesh exists.
  // It must not replace the main scene's result with an empty overlay render.
  renderer.renderImGui(nullptr);
  EXPECT_THROW(renderer.consumeCapturedPixelsRGBA(), std::runtime_error);
  dart::gui::RenderableDescriptor point;
  point.id = 1;
  point.geometry.kind = dart::gui::ShapeKind::PointCloud;
  point.geometry.pointShapeType = dart::gui::PointShapeType::Point;
  point.geometry.pointSize = 32;
  point.geometry.pointCloudPoints = {{0, 0, 0}};
  const std::array<Eigen::Vector4d, 4> colors{
      Eigen::Vector4d(1, 0, 0, 1),
      Eigen::Vector4d(0, 1, 0, 1),
      Eigen::Vector4d(0, 0, 1, 1),
      Eigen::Vector4d(1, 1, 1, 1)};
  std::vector<unsigned char> previous;
  for (unsigned frame = 0; frame < colors.size(); ++frame) {
    SCOPED_TRACE(frame);
    if (frame == 2) {
      width = 96;
      height = 64;
      renderer.resize(width, height);
    }
    // Existing tone mapping and gamut mapping can alter saturated primaries.
    // Compare the queued frame to an independent render of the same state.
    dart::gui::detail::FilamentRenderer reference(width, height, true);
    reference.setCamera({0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 60, .01, 1000);
    reference.setBackgroundColor({0, 0, 0, 1});
    point.material.rgba = colors[frame];
    reference.synchronize({point});
    const auto expected = reference.readPixelsRGBA();
    ++point.renderResourceVersion;
    renderer.synchronize({point});
    renderer.setBackgroundColor({0, 0, 0, 1});
    ASSERT_TRUE(renderer.frame(true));
    // Consumption must return the queued frame. Rendering again here would
    // instead capture this newly authored white background.
    renderer.setBackgroundColor({1, 1, 1, 1});
    const auto pixels = renderer.consumeCapturedPixelsRGBA();
    ASSERT_EQ(pixels.size(), static_cast<std::size_t>(width) * height * 4);
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_EQ(pixels[channel], 0);
    ASSERT_EQ(pixels.size(), expected.size());
    int maximumDifference = 0;
    for (std::size_t offset = 0; offset < pixels.size(); ++offset)
      maximumDifference = std::max(
          maximumDifference, std::abs(pixels[offset] - expected[offset]));
    // Each independently rendered frame can dither by one display unit.
    EXPECT_LE(maximumDifference, 2);
    for (std::size_t offset = 3; offset < pixels.size(); offset += 4)
      EXPECT_EQ(pixels[offset], 255);
    EXPECT_NE(pixels, previous);
    previous = pixels;
    EXPECT_THROW(renderer.consumeCapturedPixelsRGBA(), std::runtime_error);
  }
  ASSERT_TRUE(preserveCapture(
      "queued-single-frame-capture.png", previous, width, height));
  const auto standalone = renderer.readPixelsRGBA();
  for (unsigned channel = 0; channel < 4; ++channel)
    EXPECT_EQ(standalone[channel], 255);
}

TEST(FilamentBackend, DefaultBlueWireframePreservesAuthoredColor)
{
  constexpr unsigned width = 320, height = 240;
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>();
  frame->setShape(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(.3, .3, 1)));
  const auto authored = frame->createVisualAspect()->getRGBA();
  ASSERT_TRUE(authored.isApprox(Eigen::Vector4d(.5, .5, 1, 1)));
  std::vector<dart::gui::RenderableDescriptor> descriptors(3);
  for (unsigned i = 0; i < descriptors.size(); ++i) {
    auto& descriptor = descriptors[i];
    descriptor.id = i + 1;
    descriptor.geometry.kind = dart::gui::ShapeKind::Box;
    descriptor.geometry.size = {.3, .3, 1};
    descriptor.material.rgba = authored;
    descriptor.worldTransform.translation() = Eigen::Vector3d(0, 0, i);
  }
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({6.5, 6.5, 3}, {0, 0, 1}, {0, 0, 1}, 30, .01, 1000);
  renderer.setBackgroundColor({.58, .62, .65, 1});
  renderer.setShadowsEnabled(false);
  renderer.setWireframe(true);
  renderer.synchronize(descriptors);
  const auto pixels = renderer.readPixelsRGBA();
  std::array<std::vector<unsigned char>, 3> channels;
  constexpr std::array<int, 3> background{148, 158, 166};
  for (std::size_t offset = 0; offset < pixels.size(); offset += 4) {
    bool wirePixel = false;
    for (unsigned channel = 0; channel < 3; ++channel)
      wirePixel
          = wirePixel
            || std::abs(pixels[offset + channel] - background[channel]) > 5;
    if (wirePixel)
      for (unsigned channel = 0; channel < 3; ++channel)
        channels[channel].push_back(pixels[offset + channel]);
  }
  ASSERT_GT(channels[0].size(), 500u);
  // Actual retained OSG software capture with this exact scene/camera yields
  // median edge RGB=(96,96,192). Raw sRGB values used as linear inputs instead
  // gave (148,148,205), reducing the blue wireframe's contrast against gray.
  constexpr std::array<unsigned char, 3> osgMedian{96, 96, 192};
  for (unsigned channel = 0; channel < 3; ++channel) {
    std::sort(channels[channel].begin(), channels[channel].end());
    EXPECT_NEAR(
        channels[channel][channels[channel].size() / 2],
        osgMedian[channel],
        12);
  }
  ASSERT_TRUE(
      preserveCapture("authored-blue-wireframe.png", pixels, width, height));
}

TEST(FilamentBackend, MainScenePointAndMeshColorsPreserveAuthoredSrgb)
{
  constexpr unsigned width = 320, height = 240;
  constexpr double pi = 3.14159265358979323846;
  const double scale = height / (8 * std::tan(pi / 6));
  const auto position = [&](double x, double y) {
    return Eigen::Vector3d(
        (x - width / 2) / scale, (height / 2 - y) / scale, 0);
  };
  std::vector<dart::gui::RenderableDescriptor> descriptors(2);
  auto& point = descriptors[0];
  point.id = 1;
  point.geometry.kind = dart::gui::ShapeKind::PointCloud;
  point.geometry.pointShapeType = dart::gui::PointShapeType::Point;
  point.geometry.pointSize = 24;
  point.geometry.pointCloudPoints = {position(64.5, 120.5)};
  point.geometry.pointCloudColors = {{.5, .5, .5, 1}};
  auto& mesh = descriptors[1];
  mesh.id = 2;
  mesh.geometry.kind = dart::gui::ShapeKind::Mesh;
  mesh.geometry.triangleVertices
      = {position(160.5, 120.5), position(200.5, 120.5)};
  mesh.geometry.meshPointIndices = {0, 1};
  mesh.geometry.meshUsesVertexColors = true;
  mesh.geometry.meshVertexColors = {{.5, .5, .5, 1}, {.5, .5, 1, 1}};
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 60, .01, 1000);
  renderer.setBackgroundColor({0, 0, 0, 1});
  renderer.synchronize(descriptors);
  const auto pixels = renderer.readPixelsRGBA();
  // Pinned-SDK render reference fed explicit Color::toLinear sRGB values to
  // the unchanged shaders: gray=(116,116,116), blue=(118,118,238). These
  // include the main view's existing PBR-neutral tone mapping. HALF4 point
  // and FLOAT4 mesh color uploads must both preserve this same color meaning.
  for (unsigned x : {64u, 160u}) {
    const auto offset = (120 * width + x) * 4;
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(pixels[offset + channel], 116, 3);
    EXPECT_EQ(pixels[offset + 3], 255);
  }
  const auto blueOffset = (120 * width + 200) * 4;
  constexpr std::array<int, 3> expectedBlue{118, 118, 238};
  for (unsigned channel = 0; channel < 3; ++channel)
    EXPECT_NEAR(pixels[blueOffset + channel], expectedBlue[channel], 4);
  EXPECT_EQ(pixels[blueOffset + 3], 255);
  ASSERT_TRUE(
      preserveCapture("authored-point-mesh-colors.png", pixels, width, height));
}

TEST(FilamentBackend, DarkLinearVertexColorsPreservePrecision)
{
  constexpr unsigned width = 320, height = 240;
  constexpr double pi = 3.14159265358979323846;
  const double scale = height / (8 * std::tan(pi / 6));
  const auto position = [&](double x, double y) {
    return Eigen::Vector3d(
        (x - width / 2) / scale, (height / 2 - y) / scale, 0);
  };
  const Eigen::Vector4d gray(.025, .025, .025, 1);
  // Keep readback storage alive until context teardown drains callbacks, even
  // if a completion assertion fails.
  std::vector<unsigned char> pixels(width * height * 4);
  std::atomic<bool> done{false};
  auto context = dart::gui::detail::createFilamentRenderContext(
      width, height, true, nullptr);
  struct ContextScope
  {
    dart::gui::detail::FilamentRenderContext& context;
    ~ContextScope()
    {
      dart::gui::detail::destroyFilamentRenderContext(context);
    }
  } contextScope{context};
  auto materials = dart::gui::detail::createMaterialResources(*context.engine);
  struct SceneScope
  {
    dart::gui::detail::FilamentRenderContext& context;
    dart::gui::detail::MaterialResources& materials;
    ::filament::ColorGrading* grading;
    std::vector<dart::gui::detail::Renderable> renderables;
    ~SceneScope()
    {
      context.engine->flushAndWait();
      for (auto& renderable : renderables) {
        context.scene->remove(renderable.entity);
        dart::gui::detail::destroyRenderable(*context.engine, renderable);
      }
      context.view->setColorGrading(nullptr);
      context.engine->destroy(grading);
      dart::gui::detail::destroyMaterialResources(*context.engine, materials);
    }
  };
  // Isolate vertex precision from PBR-neutral's deliberate near-black toe.
  // Standard linear tone mapping plus sRGB output preserves authored gray
  // .025 (~6/255). Packing its linear value .001935 into UBYTE4 loses it.
  ::filament::LinearToneMapper mapper;
  SceneScope scene{
      context,
      materials,
      ::filament::ColorGrading::Builder().toneMapper(&mapper).build(
          *context.engine),
      {}};
  context.view->setColorGrading(scene.grading);
  context.view->setDithering(::filament::View::Dithering::NONE);
  context.view->setAntiAliasing(::filament::View::AntiAliasing::NONE);
  context.view->setBlendMode(::filament::View::BlendMode::TRANSLUCENT);
  context.view->setViewport({0, 0, width, height});
  context.camera->setProjection(
      60, static_cast<double>(width) / height, .01, 1000);
  context.camera->lookAt({0, 0, 4}, {0, 0, 0}, {0, 1, 0});
  ::filament::Renderer::ClearOptions clear;
  clear.clear = true;
  clear.clearColor = {0, 0, 0, 1};
  context.renderer->setClearOptions(clear);
  std::vector<dart::gui::RenderableDescriptor> descriptors(3);
  auto& point = descriptors[0];
  point.id = 1;
  point.material.rgba = gray;
  point.geometry.kind = dart::gui::ShapeKind::PointCloud;
  point.geometry.pointShapeType = dart::gui::PointShapeType::Point;
  point.geometry.pointSize = 16;
  point.geometry.pointCloudPoints = {position(50.5, 120.5)};
  auto& mesh = descriptors[1];
  mesh.id = 2;
  mesh.geometry.kind = dart::gui::ShapeKind::Mesh;
  mesh.geometry.triangleVertices = {position(160.5, 120.5)};
  mesh.geometry.meshPointIndices = {0};
  mesh.geometry.meshUsesVertexColors = true;
  mesh.geometry.meshVertexColors = {gray};
  auto& line = descriptors[2];
  line.id = 3;
  line.material.rgba = gray;
  line.geometry.kind = dart::gui::ShapeKind::LineSegments;
  line.geometry.lineVertices = {position(220.5, 120.5), position(260.5, 120.5)};
  line.geometry.lineConnections = {{0, 1}};
  line.geometry.lineThickness = 8;
  for (const auto& descriptor : descriptors) {
    auto renderable = dart::gui::detail::createRenderableFromDescriptor(
        *context.engine,
        materials.materialSet(),
        materials.textureCache,
        descriptor);
    ASSERT_TRUE(renderable);
    dart::gui::detail::updateScreenSpaceGeometry(
        *context.engine,
        *renderable,
        descriptor,
        {0, 0, 4},
        {0, 0, 0},
        {0, 1, 0},
        60,
        height);
    context.scene->addEntity(renderable->entity);
    scene.renderables.push_back(std::move(*renderable));
  }
  dart::gui::DebugTriangleDescriptor triangle;
  triangle.a = position(90.5, 130.5);
  triangle.b = position(110.5, 130.5);
  triangle.c = position(100.5, 100.5);
  triangle.rgba = gray;
  auto debug = dart::gui::detail::createDebugLineRenderable(
      *context.engine, *materials.debugColor, {}, {triangle});
  ASSERT_TRUE(debug);
  context.scene->addEntity(debug->entity);
  scene.renderables.push_back(std::move(*debug));
  context.renderer->beginFrame(context.swapChain);
  context.renderer->render(context.view);
  context.renderer->readPixels(
      0,
      0,
      width,
      height,
      ::filament::backend::PixelBufferDescriptor(
          pixels.data(),
          pixels.size(),
          ::filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
          ::filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
          [](void*, std::size_t, void* user) {
            static_cast<std::atomic<bool>*>(user)->store(true);
          },
          &done));
  context.renderer->endFrame();
  context.engine->flushAndWait();
  context.engine->pumpMessageQueues();
  ASSERT_TRUE(done.load());
  for (unsigned x : {50u, 100u, 160u, 240u}) {
    SCOPED_TRACE(x);
    const auto offset = (120 * width + x) * 4;
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(pixels[offset + channel], 6, 1);
    EXPECT_EQ(pixels[offset + 3], 255);
  }
  ASSERT_TRUE(
      preserveCapture("dark-linear-vertex-colors.png", pixels, width, height));
}

TEST(FilamentBackend, MeshWithoutUvsStillBindsAuthoredJpeg)
{
  constexpr unsigned width = 192, height = 128;
  CaptureFile redFile, blueFile;
  redFile.path.replace_extension(".jpg");
  blueFile.path.replace_extension(".jpg");
  ASSERT_TRUE(writeSolidJpeg(redFile.path, {255, 0, 0}));
  ASSERT_TRUE(writeSolidJpeg(blueFile.path, {0, 0, 255}));
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  std::vector<dart::gui::RenderableDescriptor> quads(2);
  for (unsigned index = 0; index < quads.size(); ++index) {
    auto& quad = quads[index];
    quad.id = index + 1;
    quad.material.rgba = displayColorForLinearFixture({.9, .9, .9, 1});
    auto& geometry = quad.geometry;
    geometry.kind = dart::gui::ShapeKind::Mesh;
    geometry.triangleVertices
        = {{-.45, 0, -.7}, {.45, 0, -.7}, {.45, 0, .7}, {-.45, 0, .7}};
    geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
    geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
    geometry.meshMaterials.resize(1);
    // Match URDF SHAPE_COLOR: material colors are disabled, textures remain.
    geometry.meshUsesMaterialColors = false;
    geometry.meshMaterials[0].baseColorTexturePath
        = (index == 0 ? redFile : blueFile).path.string();
    quad.worldTransform.translation().x() = index == 0 ? -.65 : .65;
  }
  renderer.synchronize(quads);
  const auto absent = renderer.readPixelsRGBA();
  const auto left = (height / 2 * width + 71) * 4;
  const auto right = (height / 2 * width + 121) * 4;
  EXPECT_GT(absent[left], absent[left + 2] + 30);
  EXPECT_GT(absent[right + 2], absent[right] + 30);
  ASSERT_TRUE(preserveCapture("mesh-no-uv-jpeg.png", absent, width, height));
  for (auto& quad : quads) {
    quad.geometry.meshTextureCoordComponents = 2;
    quad.geometry.meshTextureCoordinates.assign(4, Eigen::Vector3d::Zero());
    ++quad.renderResourceVersion;
  }
  renderer.synchronize(quads);
  const auto explicitZero = renderer.readPixelsRGBA();
  for (const auto pixel : {left, right})
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(absent[pixel + channel], explicitZero[pixel + channel], 2);
  for (auto& quad : quads) {
    quad.geometry.meshMaterials[0].baseColorTexturePath.clear();
    ++quad.renderResourceVersion;
  }
  renderer.synchronize(quads);
  const auto untextured = renderer.readPixelsRGBA();
  EXPECT_GT(untextured[left + 2], absent[left + 2] + 40);
  EXPECT_GT(untextured[right], absent[right] + 40);
}

TEST(FilamentBackend, TextureClampRetainsBlackBorderAtEveryMip)
{
  constexpr unsigned width = 160, height = 120;
  CaptureFile file;
  png_image png{};
  png.version = PNG_IMAGE_VERSION;
  png.width = png.height = 16;
  png.format = PNG_FORMAT_RGBA;
  const std::vector<unsigned char> white(16 * 16 * 4, 255);
  ASSERT_TRUE(png_image_write_to_file(
      &png, file.path.string().c_str(), 0, white.data(), 0, nullptr));
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  dart::gui::RenderableDescriptor quad;
  quad.id = 1;
  auto& geometry = quad.geometry;
  geometry.kind = dart::gui::ShapeKind::Mesh;
  geometry.triangleVertices = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
  geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
  geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
  geometry.meshTextureCoordComponents = 2;
  geometry.meshUsesMaterialColors = true;
  geometry.meshMaterials.resize(1);
  auto& material = geometry.meshMaterials[0];
  material.diffuse = material.ambient = material.specular = {0, 0, 0, 1};
  material.emissive = {1, 1, 1, 1};
  material.emissiveTexturePath = file.path.string();
  const auto center = (height / 2 * width + width / 2) * 4;
  const auto constantUv = [&](double u, double v) {
    geometry.meshTextureCoordinates.assign(4, Eigen::Vector3d(u, v, 0));
    ++quad.renderResourceVersion;
    renderer.synchronize({quad});
    return renderer.readPixelsRGBA();
  };
  const auto interior = constantUv(.5, .5);
  const auto edge = constantUv(0, .5);
  const auto corner = constantUv(0, 0);
  const auto beyondLeft = constantUv(-.25, .5);
  const auto beyondRight = constantUv(1.25, .5);
  EXPECT_GT(interior[center], edge[center] + 20);
  EXPECT_GT(edge[center], corner[center] + 20);
  // GL_CLAMP clamps UVs before filtering; it keeps a half border beyond an
  // edge, unlike REPEAT, CLAMP_TO_EDGE, and CLAMP_TO_BORDER.
  EXPECT_NEAR(beyondLeft[center], edge[center], 2);
  EXPECT_NEAR(beyondRight[center], edge[center], 2);
  geometry.meshTextureCoordinates
      = {{.1, -32, 0}, {.1, -32, 0}, {.1, 32, 0}, {.1, 32, 0}};
  ++quad.renderResourceVersion;
  renderer.synchronize({quad});
  const auto minified = renderer.readPixelsRGBA();
  // At the small mip, U=.1 also overlaps the black border. A base-level-only
  // border approximation would incorrectly retain the edge intensity.
  EXPECT_LT(minified[center], edge[center] - 20);
  EXPECT_GT(minified[center], corner[center] + 5);
  ASSERT_TRUE(
      preserveCapture("texture-clamp-interior.png", interior, width, height));
  ASSERT_TRUE(preserveCapture("texture-clamp-edge.png", edge, width, height));
  ASSERT_TRUE(
      preserveCapture("texture-clamp-corner.png", corner, width, height));
  ASSERT_TRUE(
      preserveCapture("texture-clamp-minified.png", minified, width, height));
}

TEST(FilamentBackend, RgbJpegBorderPreservesTransparentShapeOpacity)
{
  constexpr unsigned width = 160, height = 120;
  CaptureFile jpeg, rgba;
  jpeg.path.replace_extension(".jpg");
  ASSERT_TRUE(writeSolidJpeg(jpeg.path, {255, 255, 255}));
  png_image png{};
  png.version = PNG_IMAGE_VERSION;
  png.width = png.height = 16;
  png.format = PNG_FORMAT_RGBA;
  const std::vector<unsigned char> white(16 * 16 * 4, 255);
  ASSERT_TRUE(png_image_write_to_file(
      &png, rgba.path.string().c_str(), 0, white.data(), 0, nullptr));
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  renderer.setBackgroundColor({0, 0, 0, 1});
  dart::gui::RenderableDescriptor quad;
  quad.id = 1;
  auto& geometry = quad.geometry;
  geometry.kind = dart::gui::ShapeKind::Mesh;
  geometry.triangleVertices = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
  geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
  geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
  geometry.meshUsesMaterialColors = true;
  geometry.meshMaterials.resize(1);
  auto& material = geometry.meshMaterials[0];
  material.diffuse = {0, 0, 0, .5};
  material.ambient = material.specular = {0, 0, 0, 1};
  material.emissive = {1, 0, 0, 1};
  const auto capture = [&](const auto& path) {
    material.baseColorTexturePath = path.string();
    ++quad.renderResourceVersion;
    renderer.synchronize({quad});
    return renderer.readPixelsRGBA();
  };
  const auto center = (height / 2 * width + width / 2) * 4;
  for (const bool pbr : {false, true}) {
    SCOPED_TRACE(pbr ? "PBR" : "legacy");
    material.hasPbrRoughness = pbr;
    const auto rgb = capture(jpeg.path);
    const auto alpha = capture(rgba.path);
    // Both files contain white pixels. At constant UV=(0, 0), GL_CLAMP's
    // black border affects RGB color only, while RGBA also modulates alpha.
    EXPECT_GT(rgb[center], alpha[center] + 30);
    EXPECT_GT(rgb[center], rgb[center + 2] + 30);
    ASSERT_TRUE(preserveCapture(
        pbr ? "transparent-jpeg-pbr.png" : "transparent-jpeg-legacy.png",
        rgb,
        width,
        height));
  }
}

TEST(FilamentBackend, TextureChurnReleasesUnusedGpuResources)
{
  constexpr unsigned width = 64, height = 48;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  renderer.setShadowsEnabled(false);
  const auto original = renderer.getTextureCount();
  std::vector<dart::gui::RenderableDescriptor> quads(2);
  for (unsigned index = 0; index < quads.size(); ++index) {
    auto& quad = quads[index];
    quad.id = index + 1;
    auto& geometry = quad.geometry;
    geometry.kind = dart::gui::ShapeKind::Mesh;
    geometry.triangleVertices
        = {{-.4, 0, -.7}, {.4, 0, -.7}, {.4, 0, .7}, {-.4, 0, .7}};
    geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
    geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
    geometry.meshMaterials.resize(1);
    quad.worldTransform.translation().x() = index == 0 ? -.6 : .6;
  }
  for (unsigned cycle = 0; cycle < 16; ++cycle) {
    SCOPED_TRACE(cycle);
    CaptureFile texture;
    png_image png{};
    png.version = PNG_IMAGE_VERSION;
    png.width = png.height = 16;
    png.format = PNG_FORMAT_RGBA;
    std::vector<unsigned char> samples(16 * 16 * 4, 0);
    const unsigned dominant = cycle % 2 == 0 ? 0 : 2;
    for (std::size_t pixel = 0; pixel < samples.size(); pixel += 4) {
      samples[pixel + dominant] = 255;
      samples[pixel + 3] = 255;
    }
    ASSERT_TRUE(png_image_write_to_file(
        &png, texture.path.string().c_str(), 0, samples.data(), 0, nullptr));
    for (auto& quad : quads) {
      quad.geometry.meshMaterials[0].baseColorTexturePath
          = texture.path.string();
      ++quad.renderResourceVersion;
    }
    renderer.synchronize(quads);
    EXPECT_EQ(renderer.getTextureCount(), original + 1);
    const auto image = renderer.readPixelsRGBA();
    const auto left = (height / 2 * width + 23) * 4;
    EXPECT_GT(image[left + dominant], image[left + 2 - dominant] + 30);
    renderer.synchronize({quads[0]});
    EXPECT_EQ(renderer.getTextureCount(), original + 1)
        << "A texture shared by a remaining material must stay alive";
    renderer.synchronize({});
    EXPECT_EQ(renderer.getTextureCount(), original);
    // Also remove a freshly uploaded texture before its first frame; the SDK
    // must still drain its owned upload buffer and destruction commands.
    renderer.synchronize(quads);
    renderer.synchronize({});
    EXPECT_EQ(renderer.getTextureCount(), original);
    renderer.frame();
  }
}

TEST(FilamentBackend, AssimpPointLineTexturesRetainUvsAndVertexColors)
{
  constexpr unsigned width = 320, height = 240;
  constexpr double pi = 3.14159265358979323846;
  const double scale = height / (8 * std::tan(pi / 6));
  CaptureFile texture;
  png_image png{};
  png.version = PNG_IMAGE_VERSION;
  png.width = png.height = 16;
  png.format = PNG_FORMAT_RGBA;
  std::vector<unsigned char> samples(16 * 16 * 4, 0);
  for (unsigned y = 0; y < 16; ++y)
    for (unsigned x = 0; x < 16; ++x) {
      const auto pixel = (y * 16 + x) * 4;
      samples[pixel + (x < 8 ? 0 : 2)] = 255;
      samples[pixel + 3] = 255;
    }
  ASSERT_TRUE(png_image_write_to_file(
      &png, texture.path.string().c_str(), 0, samples.data(), 0, nullptr));
  auto scene = std::make_shared<aiScene>();
  scene->mNumMeshes = scene->mNumMaterials = 2;
  scene->mMeshes = new aiMesh* [2] {
    new aiMesh(), new aiMesh()
  };
  scene->mMaterials = new aiMaterial* [2] {
    new aiMaterial(), new aiMaterial()
  };
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 2;
  scene->mRootNode->mMeshes = new unsigned int[2]{0, 1};
  const auto position = [&](double x, double y) {
    return aiVector3D((x - width / 2) / scale, (height / 2 - y) / scale, 0);
  };
  auto& textured = *scene->mMeshes[0];
  textured.mMaterialIndex = 0;
  textured.mNumVertices = 6;
  textured.mVertices = new aiVector3D[6]{
      position(65.5, 65.5),
      position(125.5, 65.5),
      position(95.5, 25.5),
      position(64.5, 120.5),
      position(120.5, 120.5),
      position(220.5, 120.5)};
  textured.mNumFaces = 3;
  textured.mFaces = new aiFace[3];
  textured.mFaces[0].mNumIndices = 3;
  textured.mFaces[0].mIndices = new unsigned int[3]{0, 1, 2};
  textured.mFaces[1].mNumIndices = 1;
  textured.mFaces[1].mIndices = new unsigned int[1]{3};
  textured.mFaces[2].mNumIndices = 2;
  textured.mFaces[2].mIndices = new unsigned int[2]{4, 5};
  textured.mNumUVComponents[0] = 2;
  textured.mTextureCoords[0] = new aiVector3D[6]{
      {.25, .25, 0},
      {.75, .25, 0},
      {.5, .75, 0},
      {.25, .5, 0},
      {.25, .5, 0},
      {.75, .5, 0}};
  textured.mColors[0] = new aiColor4D[6];
  std::fill_n(textured.mColors[0], 6, aiColor4D(.6, 1, 1, 1));
  const aiString path(texture.path.string());
  ASSERT_EQ(
      scene->mMaterials[0]->AddProperty(&path, AI_MATKEY_TEXTURE_DIFFUSE(0)),
      AI_SUCCESS);
  auto& plain = *scene->mMeshes[1];
  plain.mMaterialIndex = 1;
  plain.mNumVertices = 3;
  plain.mVertices = new aiVector3D[3]{
      position(64.5, 160.5), position(120.5, 160.5), position(220.5, 160.5)};
  plain.mColors[0] = new aiColor4D[3]{{0, 1, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 1}};
  plain.mNumFaces = 2;
  plain.mFaces = new aiFace[2];
  plain.mFaces[0].mNumIndices = 1;
  plain.mFaces[0].mIndices = new unsigned int[1]{0};
  plain.mFaces[1].mNumIndices = 2;
  plain.mFaces[1].mIndices = new unsigned int[2]{1, 2};
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shape = std::make_shared<dart::dynamics::MeshShape>(
      Eigen::Vector3d::Ones(), scene);
  DART_SUPPRESS_DEPRECATED_END
  shape->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);
  dart::gui::RenderableDescriptor descriptor;
  descriptor.id = 1;
  const auto geometry = dart::gui::describeShape(*shape);
  ASSERT_TRUE(geometry);
  descriptor.geometry = *geometry;
  ASSERT_EQ(descriptor.geometry.meshPointIndices.size(), 2u);
  ASSERT_EQ(descriptor.geometry.meshLineIndices.size(), 2u);
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 60, .01, 1000);
  renderer.setBackgroundColor({.02, .03, .05, 1});
  const auto originalTextures = renderer.getTextureCount();
  renderer.synchronize({descriptor});
  EXPECT_EQ(renderer.getTextureCount(), originalTextures + 1);
  const auto image = renderer.readPixelsRGBA();
  const auto countColor = [&](const auto& pixels,
                              unsigned channel,
                              unsigned x0,
                              unsigned x1,
                              unsigned y) {
    unsigned count = 0;
    for (unsigned row = y - 1; row <= y + 1; ++row)
      for (unsigned x = x0; x < x1; ++x) {
        const auto pixel = (row * width + x) * 4;
        const auto other = (channel + 2) % 3;
        if (pixels[pixel + channel] > pixels[pixel + other] + 30)
          ++count;
      }
    return count;
  };
  EXPECT_GT(countColor(image, 0, 62, 67, 120), 0u);
  EXPECT_GT(countColor(image, 0, 126, 145, 120), 3u);
  EXPECT_GT(countColor(image, 2, 198, 217, 120), 3u);
  EXPECT_GT(countColor(image, 1, 62, 67, 160), 0u);
  EXPECT_GT(countColor(image, 1, 126, 217, 160), 30u);
  // Interpolating UVs across the line samples both texture halves at its
  // center; replacing the line's UVs with either endpoint loses this purple.
  const auto middle = (120 * width + 170) * 4;
  EXPECT_GT(image[middle], image[middle + 1] + 30);
  EXPECT_GT(image[middle + 2], image[middle + 1] + 30);
  ASSERT_TRUE(
      preserveCapture("mesh-textured-point-line.png", image, width, height));
  // Vertex color must multiply the authored texture, independently of material
  // diffuse. Changing only the source red channel removes the red samples.
  for (auto& color : descriptor.geometry.meshVertexColors)
    color.x() = 0;
  ++descriptor.renderResourceVersion;
  renderer.synchronize({descriptor});
  const auto filtered = renderer.readPixelsRGBA();
  EXPECT_EQ(countColor(filtered, 0, 62, 67, 120), 0u);
  EXPECT_EQ(countColor(filtered, 0, 126, 145, 120), 0u);
  EXPECT_GT(countColor(filtered, 2, 198, 217, 120), 3u);
  descriptor.geometry = *geometry;
  descriptor.geometry.triangleIndices.clear();
  for (auto& part : descriptor.geometry.meshParts) {
    part.triangleOffset = 0;
    part.triangleCount = 0;
  }
  ++descriptor.renderResourceVersion;
  renderer.synchronize({descriptor});
  EXPECT_EQ(renderer.getTextureCount(), originalTextures + 1)
      << "The point/line material must retain its texture after solids leave";
  renderer.setWireframe(true);
  const auto pixelsOnly = renderer.readPixelsRGBA();
  EXPECT_GT(countColor(pixelsOnly, 0, 126, 145, 120), 3u);
  EXPECT_GT(countColor(pixelsOnly, 2, 198, 217, 120), 3u);
  renderer.setCamera({0, 0, 8}, {0, 0, 0}, {0, 1, 0}, 60, .01, 1000);
  // Viewer::frame synchronizes after camera changes to refresh pixel geometry.
  renderer.synchronize({descriptor});
  const auto zoom = renderer.readPixelsRGBA();
  EXPECT_GT(countColor(zoom, 0, 110, 115, 120), 0u);
  EXPECT_GT(countColor(zoom, 0, 143, 152, 120), 2u);
  EXPECT_GT(countColor(zoom, 2, 180, 189, 120), 2u);
  EXPECT_EQ(countColor(zoom, 0, 62, 67, 120), 0u);
  ASSERT_TRUE(preserveCapture(
      "mesh-textured-point-line-zoom.png", zoom, width, height));
  renderer.synchronize({});
  EXPECT_EQ(renderer.getTextureCount(), originalTextures);

  // Retained OSG's opaque unlit bin ignores texture alpha: its actual point
  // and line captures are byte-identical for alpha 0, 128 and 255. Keep the
  // same RGB/UV inputs while retaining the qualified RGBA dither bound.
  std::array<CaptureFile, 3> alphaTextures;
  const std::array<unsigned char, 3> alphas{0, 128, 255};
  for (std::size_t i = 0; i < alphas.size(); ++i) {
    for (std::size_t pixel = 0; pixel < samples.size(); pixel += 4) {
      samples[pixel] = 230;
      samples[pixel + 1] = 64;
      samples[pixel + 2] = 26;
      samples[pixel + 3] = alphas[i];
    }
    ASSERT_TRUE(png_image_write_to_file(
        &png,
        alphaTextures[i].path.string().c_str(),
        0,
        samples.data(),
        0,
        nullptr));
  }
  const aiColor4D white(1, 1, 1, 1), black(0, 0, 0, 1);
  for (unsigned i = 0; i < scene->mNumMaterials; ++i) {
    ASSERT_EQ(
        scene->mMaterials[i]->AddProperty(&white, 1, AI_MATKEY_COLOR_AMBIENT),
        AI_SUCCESS);
    ASSERT_EQ(
        scene->mMaterials[i]->AddProperty(&white, 1, AI_MATKEY_COLOR_DIFFUSE),
        AI_SUCCESS);
    ASSERT_EQ(
        scene->mMaterials[i]->AddProperty(&black, 1, AI_MATKEY_COLOR_SPECULAR),
        AI_SUCCESS);
    ASSERT_EQ(
        scene->mMaterials[i]->AddProperty(&black, 1, AI_MATKEY_COLOR_EMISSIVE),
        AI_SUCCESS);
  }
  renderer.setWireframe(false);
  renderer.setShadowsEnabled(false);
  renderer.setCamera({0, 0, 4}, {0, 0, 0}, {0, 1, 0}, 60, .01, 1000);
  const auto empty = renderer.readPixelsRGBA();
  shape->setAlphaMode(dart::dynamics::MeshShape::AUTO);
  for (const unsigned mode : {0u, 1u, 2u}) {
    SCOPED_TRACE(mode);
    shape->setColorMode(
        mode == 0   ? dart::dynamics::MeshShape::MATERIAL_COLOR
        : mode == 1 ? dart::dynamics::MeshShape::SHAPE_COLOR
                    : dart::dynamics::MeshShape::COLOR_INDEX);
    const auto coloredGeometry = dart::gui::describeShape(*shape);
    ASSERT_TRUE(coloredGeometry);
    descriptor.geometry = *coloredGeometry;
    descriptor.geometry.triangleIndices.clear();
    for (auto& part : descriptor.geometry.meshParts) {
      part.triangleOffset = part.triangleCount = 0;
    }
    descriptor.material.rgba = {1, 1, 1, mode == 1 ? .5 : 1};
    std::array<std::vector<unsigned char>, 3> images;
    for (std::size_t i = 0; i < alphas.size(); ++i) {
      descriptor.geometry.meshMaterials[0].baseColorTexturePath
          = alphaTextures[i].path.string();
      ++descriptor.renderResourceVersion;
      renderer.synchronize({descriptor});
      images[i] = renderer.readPixelsRGBA();
      ASSERT_EQ(images[i].size(), empty.size());
      for (std::size_t pixel = 3; pixel < images[i].size(); pixel += 4)
        EXPECT_EQ(images[i][pixel], 255);
      const auto name = "mesh-point-line-bin" + std::to_string(mode)
                        + "-texture-alpha" + std::to_string(alphas[i]) + ".png";
      ASSERT_TRUE(preserveCapture(name.c_str(), images[i], width, height));
    }
    EXPECT_GT(countColor(images[2], 0, 62, 67, 120), 0u);
    EXPECT_GT(countColor(images[2], 0, 126, 145, 120), 3u);
    if (mode == 0) {
      for (std::size_t i = 0; i < 2; ++i) {
        int maximumDelta = 0;
        for (std::size_t byte = 0; byte < images[i].size(); ++byte)
          maximumDelta = std::max(
              maximumDelta,
              std::abs(int(images[i][byte]) - int(images[2][byte])));
        EXPECT_LE(maximumDelta, 2);
      }
    } else {
      // Explicitly transparent and COLOR_INDEX bins retain texture fading.
      for (const auto bounds :
           {std::array<unsigned, 2>{62, 67},
            std::array<unsigned, 2>{126, 145}}) {
        std::array<int, 3> maximumRed{};
        int clearDelta = 0;
        for (unsigned row = 119; row <= 121; ++row)
          for (unsigned x = bounds[0]; x < bounds[1]; ++x) {
            const auto pixel = (row * width + x) * 4;
            for (std::size_t i = 0; i < images.size(); ++i)
              maximumRed[i] = std::max(maximumRed[i], int(images[i][pixel]));
            for (unsigned channel = 0; channel < 4; ++channel)
              clearDelta = std::max(
                  clearDelta,
                  std::abs(
                      int(images[0][pixel + channel])
                      - int(empty[pixel + channel])));
          }
        EXPECT_LE(clearDelta, 2);
        EXPECT_GT(maximumRed[2], maximumRed[1] + 20);
        EXPECT_GT(maximumRed[2], maximumRed[0] + 30);
      }
    }
  }
  renderer.synchronize({});
  EXPECT_EQ(renderer.getTextureCount(), originalTextures);
}

TEST(FilamentBackend, HeadlessPixelsAndSceneReplacement)
{
  struct ImGuiRefreshNode : dart::gui::WorldNode
  {
    using dart::gui::WorldNode::WorldNode;
    unsigned refreshCount = 0;
    void customPreRefresh() override
    {
      ASSERT_NE(ImGui::GetCurrentContext(), nullptr);
      EXPECT_EQ(ImGui::GetCurrentContext(), getViewer()->getImGuiContext());
      EXPECT_GE(ImGui::GetTime(), 0.0);
      ++refreshCount;
    }
  };
  constexpr unsigned width = 192, height = 128;
  // Recreate the context so pending uploads/readbacks cannot escape teardown.
  for (int cycle = 0; cycle < 3; ++cycle) {
    auto world = dart::simulation::World::create();
    auto box = std::make_shared<dart::dynamics::SimpleFrame>();
    box->setShape(
        std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones()));
    box->createVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.05, 0.05));
    world->addSimpleFrame(box);
    dart::gui::Viewer viewer(Eigen::Vector4d(0.05, 0.1, 0.7, 1.0));
    auto node = std::make_shared<ImGuiRefreshNode>(world);
    viewer.addWorldNode(node);
    viewer.setCameraHome({0, -4, 0}, {0, 0, 0}, {0, 0, 1});
    viewer.setUpViewForCapture(width, height);
#if defined(__linux__)
    EXPECT_EQ(viewer.getRenderBackend(), "vulkan");
#elif defined(__APPLE__)
    EXPECT_EQ(viewer.getRenderBackend(), "metal");
#else
    EXPECT_EQ(viewer.getRenderBackend(), "opengl");
#endif
    EXPECT_FALSE(viewer.getRenderDevice().empty());
    CaptureFile file;
    viewer.captureScreen(file.path.string());
    viewer.frame();
    const auto pixels = readCapture(file, width, height);
    const auto center = (height / 2 * width + width / 2) * 4;
    EXPECT_GT(pixels[center], pixels[center + 1] + 30);
    EXPECT_GT(pixels[center], pixels[center + 2] + 30);
    EXPECT_EQ(pixels[center + 3], 255);
    box->setTranslation({0, 0, 0.6});
    viewer.captureScreen(file.path.string());
    viewer.frame();
    expectRedAboveCenter(readCapture(file, width, height), width, height);
    world->removeSimpleFrame(box);
    viewer.captureScreen(file.path.string());
    viewer.frame();
    const auto cleared = readCapture(file, width, height);
    EXPECT_GT(cleared[center + 2], cleared[center] + 30);
    EXPECT_NE(pixels, cleared);
    EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
    EXPECT_EQ(node->refreshCount, 3u);
  }
}

TEST(FilamentBackend, CaptureErrorsLeaveTheViewerUsable)
{
  dart::gui::Viewer viewer;
  viewer.setUpViewForCapture(32, 24);
  CaptureFile file;
  viewer.captureScreen((file.path / "missing-parent" / "capture.png").string());
  EXPECT_THROW(viewer.frame(), std::runtime_error);
  viewer.captureScreen(file.path.string());
  EXPECT_NO_THROW(viewer.frame());
  EXPECT_EQ(readCapture(file, 32, 24).size(), 32u * 24u * 4u);
}

TEST(FilamentBackend, ScrollZoomStopsBeforeTheCameraCollapses)
{
  using Event = dart::gui::GUIEventAdapter;
  dart::gui::Viewer viewer;
  viewer.setUpViewForCapture(32, 24);
  for (int i = 0; i < 300; ++i)
    viewer.handleEvent(Event(Event::SCROLL, 0, 16, 12, 0, 0, 0, 1));
  EXPECT_NO_THROW(viewer.frame());
  for (int i = 0; i < 300; ++i)
    viewer.handleEvent(Event(Event::SCROLL, 0, 16, 12, 0, 0, 0, -1));
  EXPECT_NO_THROW(viewer.frame());
}

TEST(FilamentBackend, ImGuiDrawOffsetsFramebufferScaleAndCallbacks)
{
  ImGuiContext* previous = ImGui::GetCurrentContext();
  ImGuiContext* context = ImGui::CreateContext();
  {
    ImDrawList list(ImGui::GetDrawListSharedData());
    list.VtxBuffer.resize(4);
    list.IdxBuffer.push_back(0);
    list.IdxBuffer.push_back(1);
    list.IdxBuffer.push_back(2);
    int callbackCount = 0;
    ImDrawCmd callback;
    callback.UserCallback = [](const ImDrawList*, const ImDrawCmd* command) {
      ++*static_cast<int*>(command->UserCallbackData);
    };
    callback.UserCallbackData = &callbackCount;
    list.CmdBuffer.push_back(callback);
    ImDrawCmd draw;
    draw.ElemCount = 3;
    draw.VtxOffset = 1;
    draw.ClipRect = {11, 21, 19, 27};
    draw.TexRef = ImTextureRef(static_cast<ImTextureID>(123));
    list.CmdBuffer.push_back(draw);
    ImDrawData data;
    data.DisplayPos = {10, 20};
    data.FramebufferScale = {2, 2};
    data.TotalVtxCount = 4;
    data.TotalIdxCount = 3;
    data.CmdLists.push_back(&list);
    const auto plan
        = dart::gui::detail::buildImGuiOverlayDrawPlan(data, 40, 30);
    ASSERT_EQ(plan.commands.size(), 1u);
    EXPECT_EQ(callbackCount, 1);
    EXPECT_EQ(plan.indices, (std::vector<std::uint32_t>{1, 2, 3}));
    EXPECT_EQ(plan.commands[0].textureId, 123u);
    EXPECT_EQ(plan.commands[0].scissorLeft, 2u);
    EXPECT_EQ(plan.commands[0].scissorBottom, 16u);
    EXPECT_EQ(plan.commands[0].scissorWidth, 16u);
    EXPECT_EQ(plan.commands[0].scissorHeight, 12u);
  }
  ImGui::DestroyContext(context);
  ImGui::SetCurrentContext(previous);
}

TEST(FilamentBackend, KeySymbolsFollowTypedCharactersAndRepeat)
{
  using dart::gui::GUIEventAdapter;
  dart::gui::detail::KeySymbolTranslator keys;
  const auto symbol = [](const auto& event) {
    return event ? (event->down ? event->symbol : -event->symbol) : 0;
  };
  // Shift+[ reports the typed '{', held keys repeat, and release matches.
  EXPECT_FALSE(keys.key(GLFW_KEY_LEFT_BRACKET, GLFW_PRESS, GLFW_MOD_SHIFT));
  EXPECT_EQ(symbol(keys.character('{')), '{');
  EXPECT_FALSE(keys.key(GLFW_KEY_LEFT_BRACKET, GLFW_REPEAT, GLFW_MOD_SHIFT));
  EXPECT_EQ(symbol(keys.character('{')), '{');
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_LEFT_BRACKET, GLFW_RELEASE, 0)), -'{');
  // The layout's character wins over the US physical token (AZERTY A on Q).
  EXPECT_FALSE(keys.key(GLFW_KEY_Q, GLFW_PRESS, 0));
  EXPECT_EQ(symbol(keys.character('a')), 'a');
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_Q, GLFW_RELEASE, 0)), -'a');
  // Non-printable keys and shortcuts are immediate and repeat.
  EXPECT_EQ(
      symbol(keys.key(GLFW_KEY_LEFT, GLFW_REPEAT, 0)),
      GUIEventAdapter::KEY_Left);
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_H, GLFW_PRESS, GLFW_MOD_CONTROL)), 'h');
  // Text without a pending press is not a viewer key; a press whose character
  // never arrives (e.g. a dead key) flushes with its fallback symbol.
  EXPECT_FALSE(keys.character('x'));
  EXPECT_FALSE(keys.key(GLFW_KEY_A, GLFW_PRESS, GLFW_MOD_SHIFT));
  EXPECT_EQ(symbol(keys.flush()), 'A');
  EXPECT_FALSE(keys.flush());
  // Windows AltGr arrives as Ctrl+Alt and still types its character, while a
  // Ctrl+Alt shortcut without text falls back to the letter.
  constexpr int ctrlAlt = GLFW_MOD_CONTROL | GLFW_MOD_ALT;
  EXPECT_FALSE(keys.key(GLFW_KEY_Q, GLFW_PRESS, ctrlAlt));
  EXPECT_EQ(symbol(keys.character('@')), '@');
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_Q, GLFW_RELEASE, ctrlAlt)), -'@');
  EXPECT_FALSE(keys.key(GLFW_KEY_W, GLFW_PRESS, ctrlAlt));
  EXPECT_EQ(symbol(keys.flush()), 'w');
  // Keypad digits deliver the typed character.
  EXPECT_FALSE(keys.key(GLFW_KEY_KP_1, GLFW_PRESS, 0));
  EXPECT_EQ(symbol(keys.character('1')), '1');
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_KP_1, GLFW_RELEASE, 0)), -'1');
  // A release matches its press even after the modifiers changed.
  constexpr int ctrlShift = GLFW_MOD_CONTROL | GLFW_MOD_SHIFT;
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_A, GLFW_PRESS, ctrlShift)), 'A');
  EXPECT_EQ(symbol(keys.key(GLFW_KEY_A, GLFW_RELEASE, GLFW_MOD_CONTROL)), -'A');
}

namespace {

// Filament caches a single-level texture's raw handle in each material
// instance, and shared UBO growth recommits every instance, including retained
// overlay clip instances that the current frame does not draw. Retire the font
// atlas on an empty frame and on a frame with fewer commands than retained
// instances, then force that recommit before any later frame can rebind.
void expectRetiredImGuiTextureLeavesNoCachedHandles(
    bool native, bool retireOnEmptyFrame)
{
  namespace detail = dart::gui::detail;
  GLFWwindow* window = nullptr;
  if (native) {
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = glfwCreateWindow(
        64, 64, "DART ImGui texture lifetime", nullptr, nullptr);
    ASSERT_NE(window, nullptr);
  }
  struct WindowScope
  {
    GLFWwindow* window;
    ~WindowScope()
    {
      if (window) {
        detail::releaseFilamentNativeWindow(window);
        glfwDestroyWindow(window);
      }
    }
  } windowScope{window};
  auto* previous = ImGui::GetCurrentContext();
  auto* imgui = ImGui::CreateContext();
  ImTextureData retired, replacement;
  retired.Create(ImTextureFormat_RGBA32, 4, 4);
  replacement.Create(ImTextureFormat_RGBA32, 4, 4);
  std::fill_n(retired.Pixels, 4 * 4 * 4, 255);
  std::fill_n(replacement.Pixels, 4 * 4 * 4, 255);
  auto context = detail::createFilamentRenderContext(
      64,
      64,
      !native,
      native ? detail::getFilamentNativeWindow(window) : nullptr);
  auto overlay = detail::createImGuiOverlay(*context.engine);
  std::vector<::filament::MaterialInstance*> instances;
  struct Scope
  {
    detail::FilamentRenderContext& context;
    detail::ImGuiOverlay& overlay;
    std::vector<::filament::MaterialInstance*>& instances;
    ImGuiContext* imgui;
    ImGuiContext* previous;
    ~Scope()
    {
      context.engine->flushAndWait();
      context.engine->pumpMessageQueues();
      for (auto* instance : instances)
        context.engine->destroy(instance);
      detail::destroyImGuiOverlay(*context.engine, overlay);
      detail::destroyFilamentRenderContext(context);
      ImGui::DestroyContext(imgui);
      ImGui::SetCurrentContext(previous);
    }
  } scope{context, overlay, instances, imgui, previous};
  ImVector<ImTextureData*> textures;
  textures.push_back(&retired);
  ImDrawList list(ImGui::GetDrawListSharedData());
  list.VtxBuffer.push_back({{0, 0}, {0.5f, 0.5f}, IM_COL32(0, 255, 0, 255)});
  list.VtxBuffer.push_back({{64, 0}, {0.5f, 0.5f}, IM_COL32(0, 255, 0, 255)});
  list.VtxBuffer.push_back({{0, 64}, {0.5f, 0.5f}, IM_COL32(0, 255, 0, 255)});
  // Three commands retain the base and two clip material instances.
  for (int command = 0; command < 3; ++command) {
    list.IdxBuffer.push_back(0);
    list.IdxBuffer.push_back(1);
    list.IdxBuffer.push_back(2);
    ImDrawCmd draw;
    draw.ElemCount = 3;
    draw.IdxOffset = command * 3;
    draw.ClipRect = {0, 0, 64, 64};
    draw.TexRef = retired.GetTexRef();
    list.CmdBuffer.push_back(draw);
  }
  ImDrawData data;
  data.DisplaySize = {64, 64};
  data.FramebufferScale = {1, 1};
  data.TotalVtxCount = 3;
  data.TotalIdxCount = 9;
  data.CmdLists.push_back(&list);
  data.Textures = &textures;
  context.renderer->setClearOptions({{0, 0, 1, 1}, true, true});
  const auto begin = [&] {
    // Display-free frames always render, as in the production renderer.
    if (!native) {
      context.renderer->beginFrame(context.swapChain);
      return;
    }
    const auto deadline
        = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!context.renderer->beginFrame(context.swapChain)) {
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("Native ImGui frame did not become ready");
      glfwWaitEventsTimeout(.01);
    }
  };
  const auto render = [&] {
    begin();
    context.renderer->render(overlay.view);
    context.renderer->endFrame();
    context.engine->flushAndWait();
    context.engine->pumpMessageQueues();
  };
  detail::updateImGuiOverlay(*context.engine, overlay, &data, 64, 64);
  render();
  ASSERT_EQ(overlay.mesh.clipMaterialInstances.size(), 2u);

  retired.UnusedFrames = 1;
  retired.WantDestroyNextFrame = true;
  retired.SetStatus(ImTextureStatus_WantDestroy);
  textures.push_back(&replacement);
  list.CmdBuffer.resize(1);
  list.CmdBuffer[0].TexRef = replacement.GetTexRef();
  data.TotalIdxCount = 3;
  ImDrawData empty;
  empty.Textures = &textures;
  detail::updateImGuiOverlay(
      *context.engine, overlay, retireOnEmptyFrame ? &empty : &data, 64, 64);
  EXPECT_EQ(retired.Status, ImTextureStatus_Destroyed);

  // Growing the shared UBO recommits every material instance's samplers.
  const auto count
      = context.engine->getConfig().sharedUboInitialSizeInBytes / 16 + 1;
  for (std::uint32_t i = 0; i < count; ++i) {
    auto* instance = overlay.material->createInstance();
    instance->setParameter(
        "fontTexture", overlay.fontTexture, ::filament::TextureSampler());
    instances.push_back(instance);
  }
  render();
  if (retireOnEmptyFrame)
    detail::updateImGuiOverlay(*context.engine, overlay, &data, 64, 64);

  std::vector<unsigned char> pixels(64 * 64 * 4);
  std::atomic<bool> complete{false};
  begin();
  context.renderer->render(overlay.view);
  context.renderer->readPixels(
      0,
      0,
      64,
      64,
      ::filament::backend::PixelBufferDescriptor(
          pixels.data(),
          pixels.size(),
          ::filament::backend::PixelDataFormat::RGBA,
          ::filament::backend::PixelDataType::UBYTE,
          [](void*, std::size_t, void* user) {
            static_cast<std::atomic<bool>*>(user)->store(true);
          },
          &complete));
  context.renderer->endFrame();
  context.engine->flushAndWait();
  context.engine->pumpMessageQueues();
  ASSERT_TRUE(complete.load());
  std::size_t green = 0;
  for (std::size_t pixel = 0; pixel < pixels.size(); pixel += 4) {
    if (pixels[pixel + 1] > pixels[pixel] + 100
        && pixels[pixel + 1] > pixels[pixel + 2] + 100)
      ++green;
    EXPECT_EQ(pixels[pixel + 3], 255);
  }
  EXPECT_GT(green, 1000u);
  const std::string name = std::string("imgui-texture-retirement-")
                           + (native ? "native" : "headless")
                           + (retireOnEmptyFrame ? "-empty.png" : ".png");
  EXPECT_TRUE(preserveCapture(name.c_str(), pixels, 64, 64));
}

} // namespace

TEST(FilamentBackend, RetiredImGuiTextureLeavesNoCachedSamplerReferences)
{
  expectRetiredImGuiTextureLeavesNoCachedHandles(false, true);
  expectRetiredImGuiTextureLeavesNoCachedHandles(false, false);
}

TEST(FilamentNative, RetiredImGuiTextureLeavesNoCachedSamplerReferences)
{
#if !defined(_WIN32) && !defined(__APPLE__)
  glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
  ASSERT_TRUE(glfwInit())
      << "Native Filament qualification requires a window system";
  struct GlfwScope
  {
    ~GlfwScope()
    {
      glfwTerminate();
    }
  } scope;
  expectRetiredImGuiTextureLeavesNoCachedHandles(true, true);
  expectRetiredImGuiTextureLeavesNoCachedHandles(true, false);
}

TEST(FilamentNative, ViewerSurvivesAtlasGrowthScaleChangesAndRepeatedWindows)
{
  struct TextWidget : dart::gui::ImGuiWidget
  {
    int frames = 0;
    void render() override
    {
      ++frames;
      // New baked sizes grow and replace ImGui's dynamic font atlas texture.
      auto* list = ImGui::GetForegroundDrawList();
      for (int k = 0; k < 4; ++k)
        list->AddText(
            ImGui::GetFont(),
            8.0f + static_cast<float>((frames * 7 + k * 13) % 120),
            ImVec2(4.0f, 4.0f + 20.0f * k),
            IM_COL32(255, 255, 0, 255),
            "DART Filament 0123456789");
      // Varying window counts leave retained clip material instances idle.
      for (int window = 0; window < 1 + frames % 4; ++window) {
        ImGui::SetNextWindowPos(ImVec2(10.0f + 20 * window, 60.0f));
        ImGui::Begin(("Window " + std::to_string(window)).c_str());
        ImGui::Text("frame %d", frames);
        ImGui::End();
      }
    }
  };
  auto world = dart::simulation::World::create();
  auto box = std::make_shared<dart::dynamics::SimpleFrame>();
  box->setShape(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  box->createVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.05, 0.05));
  world->addSimpleFrame(box);
  // A second viewer in the same process reuses the window system.
  for (int viewerIndex = 0; viewerIndex < 2; ++viewerIndex) {
    dart::gui::Viewer viewer;
    viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
    auto widget = std::make_shared<TextWidget>();
    viewer.addWidget(widget);
    // Recreating the window replaces the engine, overlay and ImGui context.
    for (int window = 0; window < 2; ++window) {
      viewer.setUpViewInWindow(0, 0, 160 + 40 * window, 120 + 30 * window);
      for (int frame = 0; frame < 40; ++frame) {
        if (frame % 10 == 0)
          viewer.setGuiScale(1.0 + 0.25 * (frame / 10));
        viewer.frame();
      }
    }
    EXPECT_GT(widget->frames, 0);
  }
}

TEST(FilamentNative, DefaultViewerWindowHasUsableDimensions)
{
  auto world = dart::simulation::World::create();
  auto box = std::make_shared<dart::dynamics::SimpleFrame>();
  box->setShape(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Ones()));
  box->createVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.05, 0.05));
  world->addSimpleFrame(box);
  dart::gui::Viewer viewer(Eigen::Vector4d(0.05, 0.1, 0.7, 1.0));
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
  viewer.setCameraHome({0, -4, 0}, {0, 0, 0}, {0, 0, 1});
  viewer.setUpViewInWindow();
  viewer.frame();
  auto* previous = ImGui::GetCurrentContext();
  ImGui::SetCurrentContext(viewer.getImGuiContext());
  const auto size = ImGui::GetIO().DisplaySize;
  const auto scale = ImGui::GetIO().DisplayFramebufferScale;
  ImGui::SetCurrentContext(previous);
  ASSERT_GT(size.x, 1);
  ASSERT_GT(size.y, 1);
  const auto width = static_cast<unsigned>(std::lround(size.x * scale.x));
  const auto height = static_cast<unsigned>(std::lround(size.y * scale.y));
  const auto pixels = dart::gui::detail::readViewerPixelsRGBA(viewer);
  ASSERT_EQ(pixels.size(), static_cast<std::size_t>(width) * height * 4);
  const auto center
      = (static_cast<std::size_t>(height) / 2 * width + width / 2) * 4;
  EXPECT_GT(pixels[center], pixels[center + 2] + 30);
  EXPECT_EQ(pixels[center + 3], 255);
  EXPECT_TRUE(
      preserveCapture("default-native-window.png", pixels, width, height));
}

TEST(FilamentNative, PresentationResizeAndContextTeardown)
{
#if !defined(_WIN32) && !defined(__APPLE__)
  glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
#endif
  ASSERT_TRUE(glfwInit())
      << "Native Filament qualification requires a window system";
  struct GlfwScope
  {
    ~GlfwScope()
    {
      glfwTerminate();
    }
  } scope;
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  for (int cycle = 0; cycle < 3; ++cycle) {
    GLFWwindow* window = glfwCreateWindow(
        192, 128, "DART Filament native qualification", nullptr, nullptr);
    ASSERT_NE(window, nullptr);
    struct WindowScope
    {
      GLFWwindow* window;
      ~WindowScope()
      {
        dart::gui::detail::releaseFilamentNativeWindow(window);
        glfwDestroyWindow(window);
      }
    } windowScope{window};
    int width = 0, height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    ASSERT_GT(width, 0);
    ASSERT_GT(height, 0);
    dart::gui::detail::FilamentRenderer renderer(
        width,
        height,
        false,
        dart::gui::detail::getFilamentNativeWindow(window));
#if defined(__APPLE__)
    EXPECT_EQ(renderer.getBackendName(), "metal");
#else
    EXPECT_EQ(renderer.getBackendName(), "opengl");
#endif
    EXPECT_FALSE(renderer.getDeviceName().empty());
    renderer.setBackgroundColor({0.05, 0.1, 0.7, 1.0});
    renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, 0.01, 1000);
    dart::gui::RenderableDescriptor box;
    box.id = 1;
    box.geometry.kind = dart::gui::ShapeKind::Box;
    box.geometry.size = Eigen::Vector3d::Ones();
    box.material.rgba = {0.9, 0.05, 0.05, 1.0};
    renderer.synchronize({box});
    for (int frame = 0; frame < 4; ++frame) {
      glfwPollEvents();
      renderer.frame();
    }
    const auto image = renderer.readPixelsRGBA();
    const auto center
        = (static_cast<std::size_t>(height) / 2 * width + width / 2) * 4;
    ASSERT_EQ(image.size(), static_cast<std::size_t>(width) * height * 4);
    EXPECT_GT(image[center], image[center + 2] + 30);
    glfwSetWindowSize(window, 256, 160);
    const auto resizeDeadline
        = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    int windowWidth = 0, windowHeight = 0;
    do {
      // Native configure events are asynchronous. Match Viewer::frame's
      // framebuffer-size observation before resizing the swap chain.
      glfwWaitEventsTimeout(0.01);
      glfwGetWindowSize(window, &windowWidth, &windowHeight);
      glfwGetFramebufferSize(window, &width, &height);
    } while (
        (windowWidth != 256 || windowHeight != 160 || width <= 0 || height <= 0)
        && std::chrono::steady_clock::now() < resizeDeadline);
    ASSERT_EQ(windowWidth, 256);
    ASSERT_EQ(windowHeight, 160);
    glfwGetFramebufferSize(window, &width, &height);
    ASSERT_GT(width, 0);
    ASSERT_GT(height, 0);
    dart::gui::detail::resizeFilamentNativeWindow(window, width, height);
    renderer.resize(width, height);
    renderer.synchronize({box});
    for (int frame = 0; frame < 4; ++frame)
      renderer.frame();
    const auto resized = renderer.readPixelsRGBA();
    ASSERT_EQ(resized.size(), static_cast<std::size_t>(width) * height * 4);
    const auto resizedCenter
        = (static_cast<std::size_t>(height) / 2 * width + width / 2) * 4;
    EXPECT_GT(resized[resizedCenter], resized[resizedCenter + 2] + 30);
    box.worldTransform.translation() = Eigen::Vector3d(0, 0, 0.6);
    renderer.synchronize({box});
    bool captured = false;
    const auto captureDeadline
        = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      glfwPollEvents();
      captured = renderer.frame(true);
      if (!captured)
        glfwWaitEventsTimeout(.01);
    } while (!captured && std::chrono::steady_clock::now() < captureDeadline);
    ASSERT_TRUE(captured);
    expectRedAboveCenter(renderer.consumeCapturedPixelsRGBA(), width, height);
  }
}

TEST(FilamentBackend, OriginalMaterialsRetainOsgDepthWriteAndBlendPolicies)
{
  namespace detail = dart::gui::detail;
  using dart::gui::MeshAlphaMode;
  using dart::gui::ShapeKind;
  using ::filament::BlendingMode;
  auto context = detail::createFilamentRenderContext(32, 24, true, nullptr);
  auto resources = detail::createMaterialResources(*context.engine);
  struct Scope
  {
    detail::FilamentRenderContext& context;
    detail::MaterialResources& resources;
    ~Scope()
    {
      context.engine->flushAndWait();
      detail::destroyMaterialResources(*context.engine, resources);
      detail::destroyFilamentRenderContext(context);
    }
  } scope{context, resources};
  const auto check
      = [&](const auto& descriptor, bool depthWrite, BlendingMode blending) {
          auto renderable = detail::createRenderableFromDescriptor(
              *context.engine,
              resources.materialSet(),
              resources.textureCache,
              descriptor);
          ASSERT_TRUE(renderable);
          EXPECT_FALSE(renderable->materials.empty());
          for (const auto& binding : renderable->materials) {
            EXPECT_EQ(binding.instance->isDepthWriteEnabled(), depthWrite);
            EXPECT_EQ(
                binding.instance->getMaterial()->getBlendingMode(), blending);
          }
          detail::destroyRenderable(*context.engine, *renderable);
        };

  dart::gui::RenderableDescriptor shape;
  shape.geometry.size = Eigen::Vector3d::Ones();
  shape.geometry.normal = Eigen::Vector3d::UnitZ();
  shape.geometry.triangleVertices = {{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}};
  shape.geometry.triangleIndices = {{0, 1, 2}};
  shape.geometry.triangleNormals.assign(3, Eigen::Vector3d::UnitZ());
  shape.geometry.voxelCenters = {{0, 0, 0}};
  for (const auto kind :
       {ShapeKind::Box,
        ShapeKind::Plane,
        ShapeKind::ConvexMesh,
        ShapeKind::Heightmap,
        ShapeKind::SoftMesh,
        ShapeKind::VoxelGrid}) {
    shape.geometry.kind = kind;
    SCOPED_TRACE(static_cast<int>(kind));
    const double threshold = 1.0 - 1e-9;
    for (const double alpha :
         {0.0,
          .5,
          .9995,
          1.0 - 5e-7,
          1.0 - 5e-9,
          threshold,
          std::nextafter(threshold, 1.0),
          1.0 - 5e-10,
          1.0}) {
      SCOPED_TRACE(alpha);
      shape.material.rgba = {.8, .2, .1, alpha};
      const bool opaque = std::abs(alpha) > threshold;
      check(shape, opaque, opaque ? BlendingMode::OPAQUE : BlendingMode::FADE);
    }
  }

  // Both overall and per-point BOX material paths always blend and write depth.
  shape.geometry.kind = ShapeKind::PointCloud;
  shape.geometry.pointShapeType = dart::gui::PointShapeType::Box;
  shape.geometry.pointCloudPoints = {{-.5, 0, 0}, {.5, 0, 0}};
  for (const double alpha : {0.0, .5, 1.0}) {
    shape.material.rgba = {.8, .2, .1, alpha};
    for (const std::size_t colorCount : {0u, 1u, 2u}) {
      SCOPED_TRACE(alpha);
      SCOPED_TRACE(colorCount);
      shape.geometry.pointCloudColors.assign(colorCount, shape.material.rgba);
      check(shape, true, BlendingMode::FADE);
    }
  }

  shape.geometry.kind = ShapeKind::Mesh;
  shape.geometry.meshMaterials.resize(1);
  shape.geometry.meshUsesMaterialColors = false;
  for (const double alpha : {.9995, 1.0 - 2e-6, 1.0 - 5e-7, 1.0}) {
    SCOPED_TRACE(alpha);
    shape.material.rgba.w() = alpha;
    const bool opaque = std::abs(static_cast<float>(alpha)) > 1.0f - 1e-6f;
    check(shape, opaque, opaque ? BlendingMode::OPAQUE : BlendingMode::FADE);
  }

  shape.geometry.meshUsesMaterialColors = true;
  shape.material.rgba.w() = 1;
  for (const auto mode :
       {MeshAlphaMode::ShapeAlpha, MeshAlphaMode::Blend, MeshAlphaMode::Auto}) {
    SCOPED_TRACE(static_cast<int>(mode));
    shape.geometry.meshAlphaMode = mode;
    for (unsigned coefficient = 0; coefficient < 4; ++coefficient) {
      SCOPED_TRACE(coefficient);
      auto& material = shape.geometry.meshMaterials.front();
      material = dart::gui::MeshMaterialDescriptor{};
      const std::array<Eigen::Vector4d*, 4> channels
          = {&material.ambient,
             &material.diffuse,
             &material.specular,
             &material.emissive};
      channels[coefficient]->w() = .5;
      const bool opaque = mode == MeshAlphaMode::ShapeAlpha;
      check(shape, opaque, opaque ? BlendingMode::OPAQUE : BlendingMode::FADE);
    }
    shape.geometry.meshMaterials.front() = dart::gui::MeshMaterialDescriptor{};
    shape.material.rgba.w() = .5;
    const bool opaque = mode == MeshAlphaMode::Auto;
    check(shape, opaque, opaque ? BlendingMode::OPAQUE : BlendingMode::FADE);
    shape.material.rgba.w() = 1;
  }

  // COLOR_INDEX retains its transparent bin even with fully opaque colors.
  shape.geometry.meshUsesMaterialColors = false;
  shape.geometry.meshUsesVertexColors = true;
  shape.geometry.meshVertexColors.assign(3, Eigen::Vector4d(.8, .2, .1, 1));
  check(shape, false, BlendingMode::FADE);

  // Unlit opaque lines and mesh pixels ignore texture alpha just as their
  // retained OSG blend-OFF bin does; transparent and vertex-color bins blend.
  auto pixels = shape;
  pixels.geometry.kind = ShapeKind::LineSegments;
  pixels.geometry.lineVertices = {{-.5, 0, 0}, {.5, 0, 0}};
  pixels.geometry.lineConnections = {{0, 1}};
  for (const double alpha : {0.0, .5, 1.0 - 5e-9, 1.0}) {
    SCOPED_TRACE(alpha);
    pixels.material.rgba.w() = alpha;
    const bool opaque = std::abs(alpha) > 1.0 - 1e-9;
    check(
        pixels,
        opaque,
        opaque ? BlendingMode::OPAQUE : BlendingMode::TRANSPARENT);
  }

  pixels.geometry.kind = ShapeKind::Mesh;
  pixels.geometry.triangleIndices.clear();
  pixels.geometry.meshPointIndices = {0};
  pixels.geometry.meshLineIndices = {{1, 2}};
  for (const bool vertexColors : {false, true}) {
    pixels.geometry.meshUsesVertexColors = vertexColors;
    for (const double alpha : {0.0, .5, 1.0}) {
      SCOPED_TRACE(vertexColors);
      SCOPED_TRACE(alpha);
      pixels.material.rgba.w() = alpha;
      const bool opaque = !vertexColors && alpha == 1;
      check(
          pixels,
          opaque,
          opaque ? BlendingMode::OPAQUE : BlendingMode::TRANSPARENT);
    }
  }
  pixels.geometry.meshUsesVertexColors = false;
  pixels.geometry.meshUsesMaterialColors = true;
  pixels.material.rgba.w() = 1;
  for (const auto mode :
       {MeshAlphaMode::ShapeAlpha, MeshAlphaMode::Blend, MeshAlphaMode::Auto}) {
    pixels.geometry.meshAlphaMode = mode;
    for (unsigned coefficient = 0; coefficient < 4; ++coefficient) {
      auto& material = pixels.geometry.meshMaterials.front();
      material = dart::gui::MeshMaterialDescriptor{};
      const std::array<Eigen::Vector4d*, 4> channels
          = {&material.ambient,
             &material.diffuse,
             &material.specular,
             &material.emissive};
      channels[coefficient]->w() = .5;
      const bool opaque = mode == MeshAlphaMode::ShapeAlpha;
      check(
          pixels,
          opaque,
          opaque ? BlendingMode::OPAQUE : BlendingMode::TRANSPARENT);
    }
  }

  pixels.geometry.kind = ShapeKind::PointCloud;
  pixels.geometry.pointShapeType = dart::gui::PointShapeType::Point;
  pixels.geometry.pointCloudColors.clear();
  for (const double alpha : {0.0, .5, 1.0}) {
    pixels.material.rgba.w() = alpha;
    check(pixels, true, BlendingMode::TRANSPARENT);
  }
}

TEST(FilamentBackend, DefaultDepthFitsVisibleGeometryAndRestoresColor)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setBackgroundColor({0, .2, .4, 1});
  renderer.setShadowsEnabled(false);
  renderer.setCamera({0, 0, 0}, {0, 0, -1}, {0, 1, 0}, 45, .01, 1000);
  const auto capture = [&](const auto& descriptors, const char* filename) {
    renderer.setDepthMode(false);
    renderer.synchronize(descriptors);
    const auto color = renderer.readPixelsRGBA();
    renderer.setDepthMode(true);
    renderer.synchronize(descriptors);
    const auto depth = renderer.readPixelsRGBA();
    EXPECT_TRUE(preserveCapture(filename, depth, width, height));
    renderer.setDepthMode(false);
    const auto restored = renderer.readPixelsRGBA();
    EXPECT_EQ(color.size(), width * height * 4);
    EXPECT_EQ(depth.size(), color.size());
    EXPECT_EQ(restored.size(), color.size());
    if (restored.size() != color.size())
      return depth;
    int maxDifference = 0;
    for (std::size_t i = 0; i < color.size(); ++i)
      maxDifference
          = std::max(maxDifference, std::abs(int(color[i]) - int(restored[i])));
    EXPECT_LE(maxDifference, 2);
    return depth;
  };
  const auto expectGray
      = [&](const auto& image, unsigned x, unsigned y, int gray) {
          ASSERT_EQ(image.size(), width * height * 4);
          const auto i = (y * width + x) * 4;
          for (unsigned channel = 0; channel < 3; ++channel)
            EXPECT_NEAR(image[i + channel], gray, 2);
          EXPECT_EQ(image[i + 3], 255);
        };
  dart::gui::RenderableDescriptor single;
  single.id = 1;
  single.renderResourceVersion = 1;
  single.geometry.kind = dart::gui::ShapeKind::Mesh;
  single.geometry.triangleVertices = {{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}};
  single.geometry.triangleIndices = {{0, 1, 2}};
  single.material.rgba = {.8, .1, .1, 1};
  single.worldTransform.translation().z() = -4;
  // Retained OSG's primitive-fit depth camera outputs 130 for distance 4,
  // and 14/246 for the two separate surfaces at distances 4 and 6.
  const auto singleImage = capture(
      std::vector<dart::gui::RenderableDescriptor>{single},
      "default-depth-single.png");
  expectGray(singleImage, 80, 60, 130);
  expectGray(singleImage, 0, 0, 255);
  auto left = single;
  // Manually authored geometry edits must invalidate the existing GPU buffers.
  left.renderResourceVersion = 2;
  left.geometry.triangleVertices = {{-.7, -.8, 0}, {.7, -.8, 0}, {0, .8, 0}};
  left.worldTransform.translation().x() = -.9;
  auto right = left;
  right.id = 2;
  right.worldTransform.translation() = Eigen::Vector3d(.9, 0, -6);
  const auto twoImage = capture(
      std::vector<dart::gui::RenderableDescriptor>{left, right},
      "default-depth-two.png");
  expectGray(twoImage, 47, 60, 14);
  expectGray(twoImage, 102, 60, 246);

  // Fully behind/off-frustum geometry, and a failed descriptor, cannot change
  // the range of the actual visible surfaces.
  auto behind = single;
  behind.id = 3;
  behind.worldTransform.translation().z() = 1;
  auto outside = single;
  outside.id = 4;
  outside.worldTransform.translation() = Eigen::Vector3d(100, 0, -100);
  auto invalid = single;
  invalid.id = 5;
  invalid.geometry.kind = dart::gui::ShapeKind::ConvexMesh;
  invalid.geometry.triangleIndices = {{0, 1, 99}};
  invalid.worldTransform.translation().z() = -1000;
  const auto excludedImage = capture(
      std::vector<dart::gui::RenderableDescriptor>{
          left, right, behind, outside, invalid},
      "default-depth-excluded.png");
  EXPECT_EQ(twoImage, excludedImage);

  // Every original corner is outside the sides, but the triangle crosses the
  // frustum. OSG clips it before finding its nearest/farthest visible points.
  auto edge = single;
  edge.renderResourceVersion = 3;
  edge.worldTransform.setIdentity();
  edge.geometry.triangleVertices = {{-10, -10, -4}, {10, -10, -6}, {0, 10, -4}};
  const auto edgeImage = capture(
      std::vector<dart::gui::RenderableDescriptor>{edge},
      "default-depth-frustum-edge.png");
  expectGray(edgeImage, 47, 60, 100);
  expectGray(edgeImage, 80, 60, 131);
  expectGray(edgeImage, 102, 60, 151);

  renderer.setCamera({0, -4, 2}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  auto floor = edge;
  floor.renderResourceVersion = 4;
  floor.geometry.triangleVertices
      = {{-100, -100, 0}, {100, -100, 0}, {100, 100, 0}, {-100, 100, 0}};
  floor.geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
  const auto floorImage = capture(
      std::vector<dart::gui::RenderableDescriptor>{floor},
      "default-depth-crossing-floor.png");
  expectGray(floorImage, 80, 60, 129);
  expectGray(floorImage, 80, 80, 87);
  expectGray(floorImage, 80, 105, 36);
}

TEST(FilamentBackend, DepthRespectsOriginalPerPrimitiveWriteMasks)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setShadowsEnabled(false);
  renderer.setCamera({0, 0, 0}, {0, 0, -1}, {0, 1, 0}, 45, .01, 1000);
  dart::gui::RenderableDescriptor far;
  far.id = 1;
  far.geometry.kind = dart::gui::ShapeKind::Mesh;
  far.geometry.triangleVertices = {{-2, -2, 0}, {2, -2, 0}, {0, 2, 0}};
  far.geometry.triangleIndices = {{0, 1, 2}};
  far.worldTransform.translation().z() = -6;
  auto near = far;
  near.id = 2;
  near.geometry.kind = dart::gui::ShapeKind::Box;
  near.geometry.size = Eigen::Vector3d::Ones();
  near.worldTransform.translation().z() = -4;
  const auto center = (height / 2 * width + width / 2) * 4;
  const auto capture = [&](int expectedGray) {
    renderer.setDepthMode(false);
    renderer.synchronize({far, near});
    const auto color = renderer.readPixelsRGBA();
    renderer.setDepthMode(true);
    renderer.synchronize({far, near});
    const auto image = renderer.readPixelsRGBA();
    ASSERT_EQ(image.size(), width * height * 4);
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(image[center + channel], expectedGray, 2);
    EXPECT_EQ(image[center + 3], 255);
    renderer.setDepthMode(false);
    const auto restored = renderer.readPixelsRGBA();
    ASSERT_EQ(restored.size(), color.size());
    int maxDifference = 0;
    for (std::size_t i = 0; i < color.size(); ++i)
      maxDifference
          = std::max(maxDifference, std::abs(int(color[i]) - int(restored[i])));
    EXPECT_LE(maxDifference, 2);
  };
  // The nonwriting near box still contributes its bounds, as in OSG. Its
  // far surface remains 248; making the near box opaque exposes depth 12.
  for (const double alpha : {0.0, .5, 1 - 5e-9, 1.0}) {
    SCOPED_TRACE(alpha);
    near.material.rgba = {1, 0, 0, alpha};
    ++near.renderResourceVersion;
    capture(alpha == 1 ? 12 : 248);
  }
  // OSG BOX points always blend and write depth, including alpha zero.
  near.geometry.kind = dart::gui::ShapeKind::PointCloud;
  near.geometry.pointShapeType = dart::gui::PointShapeType::Box;
  near.geometry.pointCloudPoints = {{0, 0, 0}};
  near.geometry.pointSize = 1;
  for (const double alpha : {0.0, .5, 1.0}) {
    SCOPED_TRACE(alpha);
    near.material.rgba.w() = alpha;
    ++near.renderResourceVersion;
    capture(12);
  }

  near.geometry.kind = dart::gui::ShapeKind::Mesh;
  near.geometry.triangleVertices = {{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}};
  near.geometry.triangleIndices = {{0, 1, 2}};
  near.geometry.meshUsesVertexColors = true;
  near.geometry.meshVertexColors.assign(3, Eigen::Vector4d(1, 0, 0, 1));
  ++near.renderResourceVersion;
  // COLOR_INDEX remains a nonwriter even when every vertex is opaque.
  capture(246);
  near.geometry.meshUsesVertexColors = false;
  near.geometry.meshUsesMaterialColors = true;
  near.geometry.meshMaterials.resize(1);
  near.geometry.meshAlphaMode = dart::gui::MeshAlphaMode::Blend;
  for (unsigned coefficient = 0; coefficient < 4; ++coefficient) {
    SCOPED_TRACE(coefficient);
    auto& material = near.geometry.meshMaterials.front();
    material = dart::gui::MeshMaterialDescriptor{};
    const std::array<Eigen::Vector4d*, 4> channels
        = {&material.ambient,
           &material.diffuse,
           &material.specular,
           &material.emissive};
    channels[coefficient]->w() = .5;
    ++near.renderResourceVersion;
    capture(246);
  }
}

TEST(FilamentBackend, WireframeAndFragmentDepth)
{
  constexpr int width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setBackgroundColor({0.05, 0.1, 0.7, 1.0});
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, 1.0, 10.0);
  dart::gui::RenderableDescriptor shape;
  shape.id = 1;
  shape.renderResourceVersion = 1;
  shape.geometry.kind = dart::gui::ShapeKind::Box;
  shape.geometry.size = Eigen::Vector3d::Ones();
  shape.material.rgba = {0.9, 0.05, 0.05, 1.0};
  renderer.synchronize({shape});
  const auto solid = renderer.readPixelsRGBA();
  renderer.setWireframe(true);
  const auto wireframe = renderer.readPixelsRGBA();
  const auto redPixels = [](const auto& pixels) {
    std::size_t count = 0;
    for (std::size_t i = 0; i < pixels.size(); i += 4)
      if (pixels[i] > pixels[i + 2] + 30)
        ++count;
    return count;
  };
  EXPECT_GT(redPixels(solid), 100u);
  EXPECT_GT(redPixels(wireframe), 10u);
  EXPECT_LT(redPixels(wireframe), redPixels(solid) / 2);
  renderer.setWireframe(false);
  renderer.setWireframe(shape.id, true);
  EXPECT_LT(redPixels(renderer.readPixelsRGBA()), redPixels(solid) / 2);
  renderer.setWireframe(shape.id, false);
  EXPECT_GT(redPixels(renderer.readPixelsRGBA()), redPixels(solid) * 3 / 4);
  shape.geometry.kind = dart::gui::ShapeKind::ConvexMesh;
  shape.geometry.triangleVertices = {{-1, -1, -1}, {1, 1, -1}, {0, 0, 1}};
  shape.geometry.triangleIndices = {{0, 1, 2}};
  ++shape.renderResourceVersion;
  renderer.synchronize({shape});
  renderer.setDepthMode(true);
  // Viewer applies its mode/camera before synchronizing the current scene.
  renderer.synchronize({shape});
  const auto depth = renderer.readPixelsRGBA();
  const auto left = (height / 2 * width + width / 2 - 15) * 4;
  const auto right = (height / 2 * width + width / 2 + 15) * 4;
  EXPECT_NEAR(depth[left], depth[left + 1], 2);
  EXPECT_NEAR(depth[left], depth[left + 2], 2);
  EXPECT_NEAR(depth[right], depth[right + 1], 2);
  EXPECT_NEAR(depth[right], depth[right + 2], 2);
  // A single sloped triangle must have fragment depth variation, rather than
  // one synthetic grayscale value for the whole shape.
  EXPECT_GT(depth[right], depth[left] + 5);
  renderer.resize(96, 64);
  renderer.synchronize({shape});
  const auto resized = renderer.readPixelsRGBA();
  EXPECT_EQ(resized.size(), 96u * 64u * 4u);
}

TEST(FilamentBackend, OverlayLabelsOutlinesAndOccludingLines)
{
  constexpr unsigned width = 320, height = 192;
  auto world = dart::simulation::World::create();
  auto box = std::make_shared<dart::dynamics::SimpleFrame>();
  box->setShape(
      std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d::Constant(2)));
  box->createVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.04, 0.04));
  world->addSimpleFrame(box);
  dart::gui::Viewer viewer({0.02, 0.02, 0.02, 1});
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
  viewer.setCameraHome({0, -4, 0}, {0, 0, 0}, {0, 0, 1});
  auto overlay = std::make_shared<dart::gui::DebugOverlay>();
  overlay->setLineWidth(6);
  overlay->addLine({-.5, 0, 0}, {.5, 0, 0}, {0, 1, 0, 1});
  overlay->setTextOutlineWidth(2);
  overlay->setTextOutlineColor({0, 0, 1, 1});
  overlay->addLabel({-.7, 0, .5}, "World", {1, 1, 1, 1}, .2);
  overlay->addLabelPixels({.7, 0, .5}, "Pixels", {1, 1, 1, 1}, 16);
  overlay->addScreenLabel({10, 10}, "Screen", {1, 1, 0, 1}, 16);
  viewer.addAttachment(overlay);
  viewer.setGuiScale(2.0);
  viewer.setUpViewForCapture(width, height);
  CaptureFile file;
  viewer.captureScreen(file.path.string());
  viewer.frame();
  const auto pixels = readCapture(file, width, height);
  const auto count = [&](unsigned x0,
                         unsigned y0,
                         unsigned x1,
                         unsigned y1,
                         const auto& matches) {
    std::size_t result = 0;
    for (unsigned y = y0; y < y1; ++y)
      for (unsigned x = x0; x < x1; ++x) {
        const auto i = (static_cast<std::size_t>(y) * width + x) * 4;
        if (matches(pixels[i], pixels[i + 1], pixels[i + 2]))
          ++result;
      }
    return result;
  };
  const auto white = [](int r, int g, int b) {
    return r > 180 && g > 180 && b > 180;
  };
  EXPECT_GT(
      count(
          8,
          8,
          90,
          32,
          [](int r, int g, int b) { return r > 150 && g > 150 && b < 100; }),
      20u);
  EXPECT_GT(count(80, 45, 155, 78, white), 15u);
  EXPECT_GT(count(170, 40, 250, 78, white), 20u);
  EXPECT_GT(
      count(
          120,
          90,
          200,
          102,
          [](int r, int g, int b) {
            return g > 150 && g > r + 60 && g > b + 60;
          }),
      150u);
  EXPECT_GT(
      count(
          0,
          0,
          width,
          height,
          [](int r, int g, int b) {
            return b > 150 && b > r + 60 && b > g + 60;
          }),
      20u);
  overlay->clear();
  viewer.captureScreen(file.path.string());
  viewer.frame();
  const auto cleared = readCapture(file, width, height);
  EXPECT_NE(pixels, cleared);
}

TEST(FilamentBackend, MaterialTeardownRestoresEngineCounts)
{
  auto context
      = dart::gui::detail::createFilamentRenderContext(32, 24, true, nullptr);
  struct ContextScope
  {
    dart::gui::detail::FilamentRenderContext& context;
    ~ContextScope()
    {
      dart::gui::detail::destroyFilamentRenderContext(context);
    }
  } scope{context};
  const auto original = context.engine->getMaterialCount();
  for (int cycle = 0; cycle < 16; ++cycle) {
    auto materials
        = dart::gui::detail::createMaterialResources(*context.engine);
    EXPECT_EQ(context.engine->getMaterialCount(), original + 11);
    dart::gui::detail::destroyMaterialResources(*context.engine, materials);
    EXPECT_EQ(context.engine->getMaterialCount(), original);
  }
}

TEST(FilamentBackend, BoxNormalsAndCameraFollowingHeadlight)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  dart::gui::RenderableDescriptor box;
  box.id = 1;
  box.geometry.kind = dart::gui::ShapeKind::Box;
  box.geometry.size = Eigen::Vector3d::Ones();
  box.material.rgba = {0.25, 0.25, 0.25, 1};
  renderer.synchronize({box});
  renderer.setHeadlightsEnabled(false);
  renderer.setCamera({0, -4, 2}, {0, 0, 0}, {0, 0, 1}, 45, 0.01, 1000);
  const auto image = renderer.readPixelsRGBA();
  const auto luminance = [&](const auto& pixels, unsigned x, unsigned y) {
    const auto i = (static_cast<std::size_t>(y) * width + x) * 4;
    return (pixels[i] + pixels[i + 1] + pixels[i + 2]) / 3.0;
  };
  // The fixed key light shines downward: the +Z face must be illuminated
  // more strongly than the front face. Interleaved flat-normal inputs broke it.
  EXPECT_GT(luminance(image, 80, 44), luminance(image, 80, 65) + 10);

  // Match OSG's unshadowed camera-light reference. OSG gray .25/.5/.75
  // gives RGB64/127/191 at both orbit angles. The sole 70k headlight gives
  // RGB73..75/125..129/182..188 on RTX Vulkan/GL and Lavapipe. The per-gray
  // bounds retain that measured PBR response plus one quantization step.
  renderer.setShadowsEnabled(false);
  const std::array<double, 3> grays{.25, .5, .75};
  const std::array<double, 3> reference{64, 127, 191};
  const std::array<double, 3> tolerance{12, 4, 10};
  const std::array<Eigen::Vector3d, 2> eyes{
      Eigen::Vector3d(0, -4, 0), Eigen::Vector3d(4, 0, 0)};
  for (std::size_t gray = 0; gray < grays.size(); ++gray) {
    SCOPED_TRACE(grays[gray]);
    box.material.rgba = {grays[gray], grays[gray], grays[gray], 1};
    renderer.synchronize({box});
    std::array<double, 2> offLuminance{};
    std::array<std::array<double, 3>, 2> onChannels{};
    for (std::size_t angle = 0; angle < eyes.size(); ++angle) {
      renderer.setCamera(eyes[angle], {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
      renderer.setHeadlightsEnabled(false);
      const auto off = renderer.readPixelsRGBA();
      offLuminance[angle] = luminance(off, 80, 60);
      renderer.setHeadlightsEnabled(true);
      const auto on = renderer.readPixelsRGBA();
      for (unsigned channel = 0; channel < 3; ++channel) {
        double sum = 0;
        for (unsigned y = 57; y <= 63; ++y)
          for (unsigned x = 77; x <= 83; ++x)
            sum += on[(static_cast<std::size_t>(y) * width + x) * 4 + channel];
        onChannels[angle][channel] = sum / 49;
        EXPECT_NEAR(
            onChannels[angle][channel], reference[gray], tolerance[gray]);
      }
    }
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(onChannels[0][channel], onChannels[1][channel], 2);
    if (gray == 1) {
      EXPECT_GT(offLuminance[1], offLuminance[0] + 10);
    }
  }

  auto context
      = dart::gui::detail::createFilamentRenderContext(32, 24, true, nullptr);
  const auto lights = dart::gui::detail::createSceneLights(
      *context.engine, false, false, 1.0);
  auto& manager = context.engine->getLightManager();
  const auto head = manager.getInstance(lights.headlight);
  const std::array<utils::Entity, 3> fixed{lights.key, lights.fill, lights.rim};
  const std::array<float, 3> fixedIntensity{82000, 26000, 18000};
  const auto expectMode = [&](bool enabled) {
    EXPECT_FLOAT_EQ(manager.getIntensity(head), enabled ? 70000 : 0);
    for (std::size_t i = 0; i < fixed.size(); ++i)
      EXPECT_FLOAT_EQ(
          manager.getIntensity(manager.getInstance(fixed[i])),
          enabled ? 0 : fixedIntensity[i]);
  };
  expectMode(true); // Construction must honor the default headlight mode.
  for (const bool enabled : {false, true, false, true}) {
    dart::gui::detail::setCameraHeadlightEnabled(
        *context.engine, lights, enabled);
    expectMode(enabled);
  }
  EXPECT_TRUE(manager.isShadowCaster(head));
  const auto key = manager.getInstance(lights.key);
  EXPECT_EQ(
      manager.getShadowOptions(head).mapSize,
      manager.getShadowOptions(key).mapSize);
  EXPECT_EQ(
      manager.getShadowOptions(head).shadowCascades,
      manager.getShadowOptions(key).shadowCascades);
  const auto expectShadowDirection = [&](const Eigen::Vector3d& expected) {
    for (const auto entity : {lights.key, lights.headlight}) {
      const auto instance = manager.getInstance(entity);
      const auto actual = manager.getShadowOptions(instance).transform
                          * manager.getDirection(instance);
      const Eigen::Vector3d direction(actual.x, actual.y, actual.z);
      EXPECT_LT((direction.normalized() - expected.normalized()).norm(), 1e-5);
    }
  };
  expectShadowDirection({0, -1, -1});
  for (const Eigen::Vector3d& direction :
       {Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(-1, 0, 0)}) {
    dart::gui::detail::updateCameraHeadlight(
        *context.engine,
        lights,
        {static_cast<float>(direction.x()),
         static_cast<float>(direction.y()),
         static_cast<float>(direction.z())});
    expectShadowDirection({0, -1, -1});
  }
  dart::gui::detail::rotateSceneLights(*context.engine, lights, {0, -1, 0});
  expectShadowDirection({0, 1, -1});
  // Changing the world up vector must not reorient the camera light.
  const auto headDirection = manager.getDirection(head);
  EXPECT_FLOAT_EQ(headDirection.x, -1);
  EXPECT_FLOAT_EQ(headDirection.y, 0);
  EXPECT_FLOAT_EQ(headDirection.z, 0);
  dart::gui::detail::rotateSceneLights(*context.engine, lights, {0, 0, 1});
  expectShadowDirection({0, -1, -1});
  dart::gui::detail::updateOrbitingKeyLight(*context.engine, lights, .25, 1);
  const auto orbitDirection = manager.getDirection(key);
  expectShadowDirection({orbitDirection.x, orbitDirection.y, orbitDirection.z});
  dart::gui::detail::destroySceneLights(*context.engine, lights);
  dart::gui::detail::destroyFilamentRenderContext(context);
}

TEST(FilamentBackend, HeadlightModesRetainFixedDirectionShadows)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({3, -4, 3}, {0, 0, .3}, {0, 0, 1}, 45, .01, 1000);
  dart::gui::RenderableDescriptor box, floor;
  box.id = 1;
  box.geometry.kind = dart::gui::ShapeKind::Box;
  box.geometry.size = Eigen::Vector3d::Ones();
  box.worldTransform.translation() = Eigen::Vector3d(0, 0, .6);
  box.material.rgba = {.75, .75, .75, 1};
  floor.id = 2;
  floor.geometry.kind = dart::gui::ShapeKind::Box;
  floor.geometry.size = {5, 5, .1};
  floor.worldTransform.translation() = Eigen::Vector3d(0, 0, -.05);
  floor.material.rgba = {.55, .55, .55, 1};
  renderer.synchronize({box, floor});
  for (const bool headlights : {false, true}) {
    SCOPED_TRACE(headlights);
    renderer.setHeadlightsEnabled(headlights);
    renderer.setShadowsEnabled(false);
    const auto lit = renderer.readPixelsRGBA();
    renderer.setShadowsEnabled(true);
    const auto shadowed = renderer.readPixelsRGBA();
    std::size_t darkFloorPixels = 0;
    for (unsigned y = 78; y < height; ++y)
      for (unsigned x = 0; x < width; ++x) {
        const auto i = (static_cast<std::size_t>(y) * width + x) * 4;
        if (lit[i] + lit[i + 1] + lit[i + 2]
            > shadowed[i] + shadowed[i + 1] + shadowed[i + 2] + 45)
          ++darkFloorPixels;
      }
    // This ROI lies below the box: it proves a real ground shadow rather than
    // darkening the object's own faces. Qualified RTX Vulkan/GL and Lavapipe
    // produce more than 300 dark floor pixels in each light mode.
    EXPECT_GT(darkFloorPixels, 300u);
    EXPECT_TRUE(preserveCapture(
        headlights ? "headlight-on-shadow.png" : "headlight-off-shadow.png",
        shadowed,
        width,
        height));
    renderer.setShadowsEnabled(false);
    const auto restored = renderer.readPixelsRGBA();
    for (std::size_t pixel = 0; pixel < lit.size(); ++pixel)
      EXPECT_NEAR(restored[pixel], lit[pixel], 2);
  }
}

TEST(FilamentBackend, ScalarUvTextureFormats)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, 0.01, 1000);
  dart::gui::RenderableDescriptor triangle;
  triangle.id = 1;
  auto& geometry = triangle.geometry;
  geometry.kind = dart::gui::ShapeKind::Mesh;
  geometry.triangleVertices = {{-1, 0, -1}, {1, 0, -1}, {0, 0, 1}};
  geometry.triangleIndices = {{0, 1, 2}};
  geometry.triangleNormals = {{0, -1, 0}, {0, -1, 0}, {0, -1, 0}};
  geometry.meshTextureCoordComponents = 1;
  geometry.meshTextureCoordinates = {{0, 23, 0}, {1, 42, 0}, {.5, 71, 0}};
  geometry.meshUsesMaterialColors = true;
  geometry.meshMaterials.resize(1);
  geometry.meshMaterials[0].diffuse = Eigen::Vector4d::Ones();
  std::vector<unsigned char> bmp(62, 0);
  bmp[0] = 'B';
  bmp[1] = 'M';
  bmp[2] = 62;
  bmp[10] = 54;
  bmp[14] = 40;
  bmp[18] = 2;
  bmp[22] = 1;
  bmp[26] = 1;
  bmp[28] = 24;
  bmp[56] = 255;
  bmp[57] = 255;
  std::vector<unsigned char> tga(18, 0);
  tga[2] = 2;
  tga[12] = 2;
  tga[14] = 1;
  tga[16] = 24;
  tga[17] = 0x20;
  tga.insert(tga.end(), {0, 0, 255, 255, 0, 0});
  auto tgaRle = tga;
  tgaRle[2] = 10;
  tgaRle.insert(tgaRle.begin() + 18, 1);
  const std::string p6 = "P6\n# Scalar U texture\n2 1\n255\n";
  std::vector<unsigned char> ppm(p6.begin(), p6.end());
  ppm.insert(ppm.end(), {255, 0, 0, 0, 0, 255});
  const std::string p3 = "P3\n2 1\n255\n255 0 0  0 0 255\n";
  const std::vector<std::pair<std::string, std::vector<unsigned char>>> formats
      = {{".png", {}},
         {".bmp", bmp},
         {".tga", tga},
         {".tga", tgaRle},
         {".ppm", ppm},
         {".ppm", {p3.begin(), p3.end()}}};
  for (const auto& format : formats) {
    SCOPED_TRACE(format.first);
    CaptureFile texture;
    texture.path.replace_extension(format.first);
    if (format.first == ".png") {
      png_image png{};
      png.version = PNG_IMAGE_VERSION;
      png.width = 2;
      png.height = 1;
      png.format = PNG_FORMAT_RGBA;
      const std::array<unsigned char, 8> colors{255, 0, 0, 255, 0, 0, 255, 255};
      ASSERT_TRUE(png_image_write_to_file(
          &png, texture.path.string().c_str(), 0, colors.data(), 0, nullptr));
    } else {
      std::ofstream out(texture.path, std::ios::binary);
      out.write(
          reinterpret_cast<const char*>(format.second.data()),
          format.second.size());
      ASSERT_TRUE(out.good());
    }
    geometry.meshMaterials[0].baseColorTexturePath = texture.path.string();
    ++triangle.renderResourceVersion;
    renderer.synchronize({triangle});
    const auto image = renderer.readPixelsRGBA();
    const auto left = (height / 2 * width + width / 2 - 12) * 4;
    const auto right = (height / 2 * width + width / 2 + 12) * 4;
    EXPECT_GT(image[left], image[left + 2] + 30);
    EXPECT_GT(image[right + 2], image[right] + 30);
  }
}

TEST(FilamentBackend, MeshTextureUsesBottomLeftUvOrigin)
{
  constexpr unsigned width = 160, height = 120;
  CaptureFile texture;
  png_image png{};
  png.version = PNG_IMAGE_VERSION;
  png.width = png.height = 2;
  png.format = PNG_FORMAT_RGBA;
  // File rows run top to bottom; mesh UVs run bottom to top.
  const std::array<unsigned char, 16> colors{
      255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 0, 255};
  ASSERT_TRUE(png_image_write_to_file(
      &png, texture.path.string().c_str(), 0, colors.data(), 0, nullptr));
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, 0.01, 1000);
  dart::gui::RenderableDescriptor quad;
  quad.id = 1;
  auto& geometry = quad.geometry;
  geometry.kind = dart::gui::ShapeKind::Mesh;
  geometry.triangleVertices = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
  geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
  geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
  geometry.meshTextureCoordComponents = 2;
  geometry.meshTextureCoordinates
      = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  geometry.meshUsesMaterialColors = true;
  geometry.meshMaterials.resize(1);
  geometry.meshMaterials[0].diffuse = Eigen::Vector4d::Ones();
  geometry.meshMaterials[0].baseColorTexturePath = texture.path.string();
  renderer.synchronize({quad});
  const auto image = renderer.readPixelsRGBA();
  const auto channel = [&](unsigned x, unsigned y, unsigned c) {
    return image[(static_cast<std::size_t>(y) * width + x) * 4 + c];
  };
  EXPECT_GT(channel(62, 42, 0), channel(62, 42, 1) + 30);
  EXPECT_GT(channel(62, 42, 0), channel(62, 42, 2) + 30);
  EXPECT_GT(channel(98, 42, 1), channel(98, 42, 0) + 30);
  EXPECT_GT(channel(98, 42, 1), channel(98, 42, 2) + 30);
  EXPECT_GT(channel(62, 78, 2), channel(62, 78, 0) + 30);
  EXPECT_GT(channel(62, 78, 2), channel(62, 78, 1) + 30);
  EXPECT_GT(channel(98, 78, 0), channel(98, 78, 2) + 30);
  EXPECT_GT(channel(98, 78, 1), channel(98, 78, 2) + 30);
}

TEST(FilamentBackend, LightingUpChangesFixedLightsWithoutMovingCamera)
{
  dart::gui::detail::FilamentRenderer renderer(160, 120, true);
  dart::gui::RenderableDescriptor box;
  box.id = 1;
  box.geometry.kind = dart::gui::ShapeKind::Box;
  box.geometry.size = Eigen::Vector3d::Ones();
  box.material.rgba = {.25, .25, .25, 1};
  renderer.synchronize({box});
  renderer.setHeadlightsEnabled(false);
  renderer.setCamera({0, -4, 2}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  const auto original = renderer.readPixelsRGBA();
  renderer.setLightingUp({0, -1, 0});
  const auto reoriented = renderer.readPixelsRGBA();
  const auto luminance = [](const auto& image, unsigned y) {
    const auto i = (y * 160 + 80) * 4;
    return (image[i] + image[i + 1] + image[i + 2]) / 3.0;
  };
  EXPECT_GT(luminance(reoriented, 65), luminance(original, 65) + 10);
  EXPECT_LT(luminance(reoriented, 44), luminance(original, 44) - 10);
  renderer.setLightingUp({0, 0, 1});
  const auto restored = renderer.readPixelsRGBA();
  EXPECT_NEAR(luminance(restored, 44), luminance(original, 44), 2);
  EXPECT_NEAR(luminance(restored, 65), luminance(original, 65), 2);
}

TEST(FilamentBackend, WorldShadowMasksAreIndependent)
{
  constexpr unsigned width = 256, height = 192;
  auto first = dart::simulation::World::create("shadowed-world");
  auto second = dart::simulation::World::create("other-world");
  const auto addBox = [](const auto& world,
                         const Eigen::Vector3d& size,
                         const Eigen::Vector3d& position,
                         const Eigen::Vector3d& color) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>();
    frame->setShape(std::make_shared<dart::dynamics::BoxShape>(size));
    frame->setTranslation(position);
    frame->createVisualAspect()->setColor(color);
    world->addSimpleFrame(frame);
  };
  addBox(first, {6, 6, .1}, {0, 0, -.05}, {.6, .6, .6});
  addBox(first, {1, 1, 1}, {-1, 0, .5}, {.8, .05, .05});
  addBox(second, {1, 1, 1}, {1, 0, .5}, {.05, .8, .05});
  auto nodeA = std::make_shared<dart::gui::WorldNode>(first);
  auto nodeB = std::make_shared<dart::gui::WorldNode>(second);
  nodeA->setShadowsEnabled(true);
  nodeB->setShadowsEnabled(false);
  dart::gui::Viewer viewer;
  viewer.addWorldNode(nodeA);
  viewer.addWorldNode(nodeB);
  viewer.setCameraHome({0, -5, 4}, {0, 0, .4}, {0, 0, 1});
  viewer.setUpViewForCapture(width, height);
  CaptureFile file;
  const auto capture = [&] {
    viewer.captureScreen(file.path.string());
    viewer.frame();
    return readCapture(file, width, height);
  };
  const auto onlyA = capture();
  nodeB->setShadowsEnabled(true);
  const auto both = capture();
  nodeA->setShadowsEnabled(false);
  const auto onlyB = capture();
  const auto darkenedFloorPixels = [](const auto& lit, const auto& shadowed) {
    std::size_t count = 0;
    for (std::size_t i = 0; i < lit.size(); i += 4) {
      if (std::abs(static_cast<int>(lit[i]) - lit[i + 1]) < 12
          && std::abs(static_cast<int>(lit[i]) - lit[i + 2]) < 12
          && shadowed[i] + shadowed[i + 1] + shadowed[i + 2] + 45
                 < lit[i] + lit[i + 1] + lit[i + 2])
        ++count;
    }
    return count;
  };
  // A's enabled flag must not make B cast a shadow, and vice versa.
  EXPECT_GT(darkenedFloorPixels(onlyA, both), 30u);
  EXPECT_GT(darkenedFloorPixels(onlyB, both), 30u);
}

TEST(FilamentBackend, AlphaFadesLitAndUnlitGeometry)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  // Match OSG WorldNode's default: isolate alpha from fixed-direction
  // self-shadows.
  renderer.setShadowsEnabled(false);
  renderer.setBackgroundColor({0, 0, .6, 1});
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  const auto blank = renderer.readPixelsRGBA();
  dart::gui::RenderableDescriptor descriptor;
  descriptor.id = 1;
  descriptor.geometry.kind = dart::gui::ShapeKind::Box;
  descriptor.geometry.size = Eigen::Vector3d::Ones();
  descriptor.material.rgba = {1, 0, 0, 1};
  const auto center = (height / 2 * width + width / 2) * 4;
  for (const auto kind :
       {dart::gui::ShapeKind::Box, dart::gui::ShapeKind::PointCloud}) {
    descriptor.geometry.kind = kind;
    descriptor.geometry.pointShapeType
        = dart::gui::PointShapeType::BillboardSquare;
    descriptor.geometry.pointCloudPoints = {{0, 0, 0}};
    descriptor.geometry.pointSize = 40;
    std::array<std::vector<unsigned char>, 3> images;
    unsigned sample = 0;
    for (const double alpha : {0.0, .5, 1.0}) {
      descriptor.material.rgba.w() = alpha;
      ++descriptor.renderResourceVersion;
      renderer.synchronize({descriptor});
      images[sample++] = renderer.readPixelsRGBA();
    }
    for (unsigned channel = 0; channel < 3; ++channel)
      EXPECT_NEAR(images[0][center + channel], blank[center + channel], 2);
    EXPECT_GT(images[1][center], images[0][center] + 20);
    EXPECT_GT(images[2][center], images[1][center] + 20);
    EXPECT_LT(images[2][center + 2], images[1][center + 2] - 20);
  }
  renderer.synchronize({});
  std::array<std::vector<unsigned char>, 3> images;
  unsigned sample = 0;
  for (const double alpha : {0.0, .5, 1.0}) {
    dart::gui::DebugScene debug;
    dart::gui::DebugTriangleDescriptor triangle;
    triangle.a = {-1, 0, -1};
    triangle.b = {1, 0, -1};
    triangle.c = {0, 0, 1};
    triangle.rgba = {1, 0, 0, alpha};
    debug.triangles.push_back(triangle);
    renderer.setDebugScene(debug);
    images[sample++] = renderer.readPixelsRGBA();
  }
  for (unsigned channel = 0; channel < 3; ++channel)
    EXPECT_NEAR(images[0][center + channel], blank[center + channel], 2);
  EXPECT_GT(images[1][center], images[0][center] + 20);
  EXPECT_GT(images[2][center], images[1][center] + 20);
}

TEST(FilamentBackend, TranslucentSelfShadowsHaveUniformPlanarFaces)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setBackgroundColor({0, 0, .6, 1});
  renderer.setShadowsEnabled(true);
  dart::gui::RenderableDescriptor box;
  box.id = 1;
  box.geometry.kind = dart::gui::ShapeKind::Box;
  box.geometry.size = Eigen::Vector3d::Ones();
  box.material.castsShadows = true;
  box.material.receivesShadows = true;
  for (const Eigen::Vector3d& eye :
       {Eigen::Vector3d(0, -4, 0), Eigen::Vector3d(4, 0, 0)}) {
    SCOPED_TRACE(eye.transpose());
    renderer.setCamera(eye, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
    renderer.synchronize({});
    const auto background = renderer.readPixelsRGBA();
    for (const double alpha : {0.0, .5, 1.0}) {
      SCOPED_TRACE(alpha);
      box.material.rgba = {1, 0, 0, alpha};
      ++box.renderResourceVersion;
      renderer.synchronize({box});
      const auto image = renderer.readPixelsRGBA();
      ASSERT_EQ(image.size(), width * height * 4);
      unsigned low = 255, high = 0;
      for (unsigned y = 48; y <= 72; ++y) {
        const auto i = (y * width + width / 2) * 4;
        low = std::min(low, unsigned(image[i]));
        high = std::max(high, unsigned(image[i]));
        EXPECT_EQ(image[i + 3], 255);
      }
      if (alpha == .5) {
        // Inward winding produced 26-level horizontal red bands on the
        // half-alpha self-shadowed face. Its corrected red strip varies by
        // at most two output levels on native GL, GPU and CPU Vulkan.
        EXPECT_LE(high - low, 2u);
        ASSERT_TRUE(preserveCapture(
            eye.x() == 0 ? "translucent-shadow-y-face.png"
                         : "translucent-shadow-x-face.png",
            image,
            width,
            height));
      }
      const auto center = (height / 2 * width + width / 2) * 4;
      if (alpha == 0) {
        for (unsigned channel = 0; channel < 4; ++channel)
          EXPECT_NEAR(image[center + channel], background[center + channel], 2);
      } else {
        EXPECT_GT(image[center], 20);
      }
    }
  }
}

TEST(FilamentBackend, OpenSurfaceCullingRetainsSoftMeshTwoSides)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setBackgroundColor({0, .2, .4, 1});
  renderer.setShadowsEnabled(false);
  dart::gui::RenderableDescriptor triangle;
  triangle.id = 1;
  triangle.geometry.triangleVertices = {{-1, -1, 0}, {1, -1, 0}, {0, 1, 0}};
  triangle.geometry.triangleIndices = {{0, 1, 2}};
  triangle.geometry.triangleNormals.assign(3, Eigen::Vector3d::UnitZ());
  triangle.material.rgba = {.8, .1, .1, 1};
  const auto center = (height / 2 * width + width / 2) * 4;
  for (const auto kind :
       {dart::gui::ShapeKind::ConvexMesh,
        dart::gui::ShapeKind::Heightmap,
        dart::gui::ShapeKind::Mesh,
        dart::gui::ShapeKind::SoftMesh}) {
    SCOPED_TRACE(static_cast<int>(kind));
    triangle.geometry.kind = kind;
    ++triangle.renderResourceVersion;
    for (const double cameraZ : {4.0, -4.0}) {
      SCOPED_TRACE(cameraZ);
      // Keep the plane inside explicit clips and visibly separated from the
      // white far-depth background; the color projection is unchanged.
      renderer.setCamera({0, 0, cameraZ}, {0, 0, 0}, {0, 1, 0}, 45, 1, 10);
      renderer.synchronize({});
      std::array<std::vector<unsigned char>, 2> backgrounds;
      for (unsigned mode = 0; mode < backgrounds.size(); ++mode) {
        renderer.setDepthMode(mode == 1);
        backgrounds[mode] = renderer.readPixelsRGBA();
        ASSERT_EQ(backgrounds[mode].size(), width * height * 4);
      }
      for (unsigned channel = 0; channel < 4; ++channel)
        EXPECT_EQ(backgrounds[1][center + channel], 255);
      renderer.setDepthMode(false);
      renderer.synchronize({triangle});
      std::vector<unsigned char> initialColor;
      for (const bool depth : {false, true, false}) {
        SCOPED_TRACE(depth);
        renderer.setDepthMode(depth);
        renderer.synchronize({triangle});
        const auto image = renderer.readPixelsRGBA();
        ASSERT_EQ(image.size(), width * height * 4);
        const auto& background = backgrounds[depth ? 1 : 0];
        // OSG culls the back of open asset/convex/heightmap surfaces in both
        // modes; SoftMesh explicitly retains two-sided rendering.
        if (cameraZ > 0 || kind == dart::gui::ShapeKind::SoftMesh) {
          if (depth) {
            int difference = 0;
            for (unsigned channel = 0; channel < 3; ++channel)
              difference += std::abs(
                  static_cast<int>(image[center + channel])
                  - background[center + channel]);
            EXPECT_GT(difference, 50);
            EXPECT_NEAR(image[center], image[center + 1], 1);
            EXPECT_NEAR(image[center], image[center + 2], 1);
          } else {
            EXPECT_GT(image[center], image[center + 2] + 50);
          }
        } else {
          for (unsigned channel = 0; channel < 4; ++channel)
            EXPECT_NEAR(
                image[center + channel], background[center + channel], 2);
        }
        EXPECT_EQ(image[center + 3], 255);
        if (!depth) {
          if (initialColor.empty())
            initialColor = image;
          else
            for (unsigned channel = 0; channel < 4; ++channel)
              EXPECT_NEAR(
                  image[center + channel], initialColor[center + channel], 2);
        }
      }
    }
  }

  // Opposite culling policies must coexist in one depth frame, including
  // replacement/removal of resources while the depth override is active.
  renderer.setCamera({0, 0, -4}, {0, 0, 0}, {0, 1, 0}, 45, 1, 10);
  renderer.synchronize({});
  renderer.setDepthMode(true);
  const auto emptyDepth = renderer.readPixelsRGBA();
  auto oneSided = triangle;
  oneSided.id = 1;
  oneSided.geometry.kind = dart::gui::ShapeKind::Mesh;
  auto twoSided = triangle;
  twoSided.id = 2;
  twoSided.geometry.kind = dart::gui::ShapeKind::SoftMesh;
  for (auto* descriptor : {&oneSided, &twoSided})
    for (auto& vertex : descriptor->geometry.triangleVertices)
      vertex *= .4;
  oneSided.worldTransform.translation().x() = -.6;
  twoSided.worldTransform.translation().x() = .6;
  const auto culledPixel = (height / 2 * width + width / 2 + 22) * 4;
  const auto visiblePixel = (height / 2 * width + width / 2 - 22) * 4;
  const auto checkMixed = [&](const auto& image, bool softVisible) {
    ASSERT_EQ(image.size(), width * height * 4);
    for (unsigned channel = 0; channel < 4; ++channel)
      EXPECT_NEAR(
          image[culledPixel + channel], emptyDepth[culledPixel + channel], 2);
    int difference = 0;
    for (unsigned channel = 0; channel < 3; ++channel)
      difference += std::abs(
          static_cast<int>(image[visiblePixel + channel])
          - emptyDepth[visiblePixel + channel]);
    if (softVisible)
      EXPECT_GT(difference, 50);
    else
      EXPECT_LE(difference, 6);
    EXPECT_EQ(image[visiblePixel + 3], 255);
  };
  renderer.synchronize({oneSided, twoSided});
  checkMixed(renderer.readPixelsRGBA(), true);
  ++oneSided.renderResourceVersion;
  ++twoSided.renderResourceVersion;
  renderer.synchronize({oneSided, twoSided});
  checkMixed(renderer.readPixelsRGBA(), true);
  renderer.synchronize({oneSided});
  checkMixed(renderer.readPixelsRGBA(), false);
  renderer.synchronize({});

  const Eigen::Vector4d latestBackground(.1, .3, .5, 1);
  renderer.setDepthMode(false);
  renderer.setBackgroundColor(latestBackground);
  const auto authoredBackground = renderer.readPixelsRGBA();
  renderer.setBackgroundColor({0, .2, .4, 1});
  renderer.setDepthMode(true);
  renderer.setBackgroundColor(latestBackground);
  const auto whileDepth = renderer.readPixelsRGBA();
  for (unsigned channel = 0; channel < 4; ++channel)
    EXPECT_EQ(whileDepth[center + channel], 255);
  renderer.setDepthMode(false);
  const auto restoredBackground = renderer.readPixelsRGBA();
  for (unsigned channel = 0; channel < 4; ++channel)
    EXPECT_NEAR(
        restoredBackground[center + channel],
        authoredBackground[center + channel],
        2);
}

TEST(FilamentBackend, ImportedWhiteSpecularRetainsShadowedDiffuseFill)
{
  constexpr unsigned width = 160, height = 120;
  auto scene = std::make_shared<aiScene>();
  scene->mNumMaterials = scene->mNumMeshes = 1;
  scene->mMaterials = new aiMaterial* [1] {
    new aiMaterial()
  };
  scene->mMeshes = new aiMesh* [1] {
    new aiMesh()
  };
  const aiColor4D white(1, 1, 1, 1), black(0, 0, 0, 1),
      ambient(.05, .05, .05, 1);
  auto& material = *scene->mMaterials[0];
  ASSERT_EQ(
      material.AddProperty(&white, 1, AI_MATKEY_COLOR_DIFFUSE), AI_SUCCESS);
  ASSERT_EQ(
      material.AddProperty(&white, 1, AI_MATKEY_COLOR_SPECULAR), AI_SUCCESS);
  ASSERT_EQ(
      material.AddProperty(&ambient, 1, AI_MATKEY_COLOR_AMBIENT), AI_SUCCESS);
  ASSERT_EQ(
      material.AddProperty(&black, 1, AI_MATKEY_COLOR_EMISSIVE), AI_SUCCESS);
  auto& mesh = *scene->mMeshes[0];
  mesh.mNumVertices = 4;
  mesh.mVertices
      = new aiVector3D[4]{{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
  mesh.mNormals
      = new aiVector3D[4]{{0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}};
  mesh.mNumFaces = 2;
  mesh.mFaces = new aiFace[2];
  mesh.mFaces[0].mNumIndices = mesh.mFaces[1].mNumIndices = 3;
  mesh.mFaces[0].mIndices = new unsigned[3]{0, 1, 2};
  mesh.mFaces[1].mIndices = new unsigned[3]{0, 2, 3};
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned[1]{0};
  DART_SUPPRESS_DEPRECATED_BEGIN
  dart::dynamics::MeshShape shape(Eigen::Vector3d::Ones(), scene);
  DART_SUPPRESS_DEPRECATED_END
  shape.setColorMode(dart::dynamics::MeshShape::MATERIAL_COLOR);
  shape.setAlphaMode(dart::dynamics::MeshShape::AUTO);
  dart::gui::RenderableDescriptor receiver, caster;
  receiver.id = 1;
  receiver.material.castsShadows = false;
  caster.id = 2;
  caster.geometry.kind = dart::gui::ShapeKind::Box;
  caster.geometry.size = Eigen::Vector3d::Constant(.5);
  // The camera headlight and retained fixed shadow direction are independent.
  // This box is hidden behind the receiver but casts a visible center shadow.
  caster.worldTransform.translation() = Eigen::Vector3d(0, .5, .5);
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  renderer.setShadowsEnabled(true);
  const auto capture = [&] {
    const auto geometry = dart::gui::describeShape(shape);
    EXPECT_TRUE(geometry);
    if (!geometry)
      return std::vector<unsigned char>{};
    receiver.geometry = *geometry;
    ++receiver.renderResourceVersion;
    renderer.synchronize({receiver, caster});
    return renderer.readPixelsRGBA();
  };
  const auto importedWhite = capture();
  ASSERT_EQ(
      material.AddProperty(&black, 1, AI_MATKEY_COLOR_SPECULAR), AI_SUCCESS);
  const auto importedBlack = capture();
  ASSERT_EQ(importedWhite.size(), width * height * 4);
  ASSERT_EQ(importedBlack.size(), importedWhite.size());
  int maximumDelta = 0;
  for (std::size_t i = 0; i < importedWhite.size(); ++i)
    maximumDelta = std::max(
        maximumDelta, std::abs(int(importedWhite[i]) - int(importedBlack[i])));
  EXPECT_LE(maximumDelta, 2);
  const auto center = (height / 2 * width + width / 2) * 4;
  EXPECT_GT(importedWhite[center], 20);
  renderer.setShadowsEnabled(false);
  const auto lit = renderer.readPixelsRGBA();
  ASSERT_EQ(lit.size(), importedWhite.size());
  EXPECT_GT(lit[center], importedWhite[center] + 20);
  ASSERT_TRUE(preserveCapture(
      "imported-white-specular-shadow.png", importedWhite, width, height));
  ASSERT_TRUE(preserveCapture(
      "imported-black-specular-shadow.png", importedBlack, width, height));
  ASSERT_TRUE(
      preserveCapture("imported-black-specular-lit.png", lit, width, height));
}

TEST(FilamentBackend, LegacyMeshSpecularGlossAmbientAndEmission)
{
  constexpr unsigned width = 160, height = 120;
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  // Isolate the material coefficients from the independent fixed-direction
  // scene shadow feature, which is covered by
  // HeadlightModesRetainFixedDirectionShadows.
  renderer.setShadowsEnabled(false);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  dart::gui::RenderableDescriptor quad;
  quad.id = 1;
  auto& geometry = quad.geometry;
  geometry.kind = dart::gui::ShapeKind::Mesh;
  geometry.triangleVertices = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
  geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
  geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
  geometry.meshUsesMaterialColors = true;
  geometry.meshMaterials.resize(1);
  auto& material = geometry.meshMaterials[0];
  material.diffuse = displayColorForLinearFixture({.015, .015, .015, 1});
  material.ambient = {0, 0, 0, 1};
  material.specular = {.08, .002, .002, 1};
  material.shininess = 2;
  const auto capture = [&] {
    ++quad.renderResourceVersion;
    renderer.synchronize({quad});
    return renderer.readPixelsRGBA();
  };
  const auto broad = capture();
  material.shininess = 128;
  const auto glossy = capture();
  const auto center = (height / 2 * width + width / 2) * 4;
  const auto edge = (height / 2 * width + width / 2 + 24) * 4;
  EXPECT_GT(glossy[center], glossy[center + 1] + 20);
  EXPECT_GT(glossy[center], broad[center] + 10);
  EXPECT_GT(glossy[center] - glossy[edge], broad[center] - broad[edge] + 10);
  material.shininessStrength = 1.0 / 64;
  const auto softened = capture();
  EXPECT_NEAR(softened[center], broad[center], 2);
  material.specular = {0, 0, 0, 1};
  material.ambient = {0, .8, 0, 1};
  const auto ambient = capture();
  EXPECT_GT(ambient[center + 1], ambient[center] + 20);
  material.ambient = {0, 0, 0, 1};
  // Retained OSG outputs blue=51 for the raw .2 emission coefficient.
  // Decoding it as an sRGB display color instead reduced this render to 2.
  material.emissive = {0, 0, .2, 1};
  const auto emissive = capture();
  EXPECT_GT(emissive[center + 2], emissive[center] + 20);
  EXPECT_NEAR(emissive[center + 2], 51, 4);
  material.diffuse.w() = 0;
  const auto invisible = capture();
  renderer.synchronize({});
  const auto blank = renderer.readPixelsRGBA();
  for (unsigned channel = 0; channel < 3; ++channel)
    EXPECT_NEAR(invisible[center + channel], blank[center + channel], 2);
}

TEST(FilamentBackend, NormalMapChangesLegacyAndPbrIllumination)
{
  constexpr unsigned width = 160, height = 120;
  CaptureFile flatFile, bentFile;
  const auto writeNormalMap = [](const auto& path, bool bent) {
    std::array<unsigned char, 32 * 4> samples{};
    for (unsigned x = 0; x < 32; ++x) {
      samples[x * 4] = 128;
      samples[x * 4 + 1] = bent ? (x < 16 ? 26 : 230) : 128;
      samples[x * 4 + 2] = bent ? 204 : 255;
      samples[x * 4 + 3] = 255;
    }
    png_image png{};
    png.version = PNG_IMAGE_VERSION;
    png.width = 32;
    png.height = 1;
    png.format = PNG_FORMAT_RGBA;
    return png_image_write_to_file(
        &png, path.string().c_str(), 0, samples.data(), 0, nullptr);
  };
  ASSERT_TRUE(writeNormalMap(flatFile.path, false));
  ASSERT_TRUE(writeNormalMap(bentFile.path, true));
  dart::gui::detail::FilamentRenderer renderer(width, height, true);
  renderer.setHeadlightsEnabled(false);
  renderer.setCamera({0, -4, 0}, {0, 0, 0}, {0, 0, 1}, 45, .01, 1000);
  dart::gui::RenderableDescriptor quad;
  quad.id = 1;
  auto& geometry = quad.geometry;
  geometry.kind = dart::gui::ShapeKind::Mesh;
  geometry.triangleVertices = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};
  geometry.triangleIndices = {{0, 1, 2}, {0, 2, 3}};
  geometry.triangleNormals.assign(4, Eigen::Vector3d(0, -1, 0));
  geometry.meshTextureCoordComponents = 2;
  geometry.meshTextureCoordinates
      = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  geometry.meshUsesMaterialColors = true;
  geometry.meshMaterials.resize(1);
  auto& material = geometry.meshMaterials[0];
  material.diffuse = displayColorForLinearFixture({.25, .25, .25, 1});
  const auto capture = [&](const auto& path) {
    material.normalTexturePath = path.string();
    ++quad.renderResourceVersion;
    renderer.synchronize({quad});
    return renderer.readPixelsRGBA();
  };
  const auto luminance = [&](const auto& image, unsigned x) {
    const auto i = (height / 2 * width + x) * 4;
    return (image[i] + image[i + 1] + image[i + 2]) / 3.0;
  };
  for (const bool pbr : {false, true}) {
    SCOPED_TRACE(pbr ? "PBR" : "Legacy specular/gloss");
    material.hasPbrRoughness = pbr;
    const auto flat = capture(flatFile.path);
    const auto bent = capture(bentFile.path);
    // The SDK encodes this quad's bitangent toward -Z. Low green tilts the
    // normal upward into the fixed lights; high green tilts it downward.
    EXPECT_GT(luminance(bent, 62), luminance(flat, 62) + 20);
    EXPECT_LT(luminance(bent, 98), luminance(flat, 98) - 20);
  }
}
