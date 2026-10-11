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

// A paused-world renderer comparison. GPU completion is part of every measured
// frame; readback, state guards, and PNG encoding are
// separate. Run serially with the repository comparison runner.
#if defined(_WIN32) && !defined(NOMINMAX)
  #define NOMINMAX
#endif

#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>
#include <dart/gui/detail/viewer_access.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/LineSegmentShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/PointCloudShape.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>

#include <dart/common/Deprecated.hpp>

#include <assimp/material.h>
#include <assimp/scene.h>
#include <png.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <cmath>
#include <cstdlib>
#include <cstring>
#if defined(_WIN32)
  #include <psapi.h>
  #include <windows.h>
#else
  #include <sys/resource.h>
#endif

namespace {
using Clock = std::chrono::steady_clock;
using dart::dynamics::VisualAspect;
struct Options
{
  std::string renderer = "filament", scene = "boxes", png;
  int width = 1280, height = 720, warmup = 60, resets = 0;
  std::size_t count = 0, frames = 0;
  double seconds = 30;
  bool shadows = false;
};
Options parse(int argc, char** argv)
{
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--shadows") {
      o.shadows = true;
      continue;
    }
    if (flag == "--help") {
      std::cout << "gui_render_benchmark --renderer filament --scene "
                   "boxes|mesh|pointline|soft\n"
                   "  --width 1280 --height 720 --warmup 60 --seconds 30 | "
                   "--frames N\n"
                   "  --count N --shadows --resets N --png PATH\n"
                   "Resets divide the total measured window into N+1 viewer "
                   "lifetimes.\n";
      std::exit(0);
    }
    if (i + 1 == argc)
      throw std::invalid_argument("Missing value for " + flag);
    const std::string value = argv[++i];
    if ((flag == "--frames" || flag == "--count") && !value.empty()
        && value.front() == '-')
      throw std::invalid_argument("Counts must be nonnegative");
    if (flag == "--renderer")
      o.renderer = value;
    else if (flag == "--scene")
      o.scene = value;
    else if (flag == "--png")
      o.png = value;
    else if (flag == "--width")
      o.width = std::stoi(value);
    else if (flag == "--height")
      o.height = std::stoi(value);
    else if (flag == "--warmup")
      o.warmup = std::stoi(value);
    else if (flag == "--resets")
      o.resets = std::stoi(value);
    else if (flag == "--count")
      o.count = std::stoull(value);
    else if (flag == "--frames")
      o.frames = std::stoull(value);
    else if (flag == "--seconds")
      o.seconds = std::stod(value);
    else
      throw std::invalid_argument("Unknown option " + flag);
  }
  if (o.renderer != "filament")
    throw std::invalid_argument("Unknown renderer");
  if (o.scene != "boxes" && o.scene != "mesh" && o.scene != "pointline"
      && o.scene != "soft")
    throw std::invalid_argument("Unknown scene");
  if (o.width <= 0 || o.height <= 0 || o.warmup < 0 || o.resets < 0
      || !std::isfinite(o.seconds) || (o.frames == 0 && o.seconds <= 0)
      || (o.frames > 0 && o.frames <= static_cast<std::size_t>(o.resets)))
    throw std::invalid_argument(
        "Dimensions and measured windows must be positive");
  if (o.count == 0)
    o.count = o.scene == "pointline" ? 10000
              : o.scene == "soft"    ? 64
              : o.scene == "mesh"    ? 128
                                     : 512;
  return o;
}
double milliseconds(Clock::duration elapsed)
{
  return std::chrono::duration<double, std::milli>(elapsed).count();
}
Eigen::Vector3d placement(std::size_t i, std::size_t count)
{
  const auto side = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(
          std::ceil(std::cbrt(static_cast<double>(count)))));
  const double step = 3.0 / side;
  return {
      -1.5 + step * (static_cast<int>(i % side) + 0.5),
      -1.5 + step * (static_cast<int>((i / side) % side) + 0.5),
      0.2 + step * static_cast<int>(i / (side * side))};
}
std::shared_ptr<const aiScene> meshScene()
{
  constexpr unsigned side = 33;
  auto scene = std::make_shared<aiScene>();
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial* [1] {
    new aiMaterial()
  };
  const aiColor4D color(0.7f, 0.5f, 0.2f, 1);
  scene->mMaterials[0]->AddProperty(&color, 1, AI_MATKEY_COLOR_DIFFUSE);
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh* [1] {
    new aiMesh()
  };
  auto& mesh = *scene->mMeshes[0];
  mesh.mNumVertices = side * side;
  mesh.mVertices = new aiVector3D[mesh.mNumVertices];
  mesh.mNormals = new aiVector3D[mesh.mNumVertices];
  for (unsigned y = 0; y < side; ++y)
    for (unsigned x = 0; x < side; ++x) {
      const float u = static_cast<float>(x) / (side - 1) - 0.5f,
                  v = static_cast<float>(y) / (side - 1) - 0.5f;
      mesh.mVertices[y * side + x]
          = {u, v, 0.05f * std::sin(6 * u) * std::cos(6 * v)};
      Eigen::Vector3f n(
          -0.3f * std::cos(6 * u) * std::cos(6 * v),
          0.3f * std::sin(6 * u) * std::sin(6 * v),
          1);
      n.normalize();
      mesh.mNormals[y * side + x] = {n.x(), n.y(), n.z()};
    }
  mesh.mNumFaces = 2 * (side - 1) * (side - 1);
  mesh.mFaces = new aiFace[mesh.mNumFaces];
  unsigned face = 0;
  for (unsigned y = 0; y + 1 < side; ++y)
    for (unsigned x = 0; x + 1 < side; ++x) {
      const unsigned a = y * side + x, b = a + 1, c = a + side, d = c + 1;
      for (const auto triangle :
           {std::array<unsigned, 3>{a, b, c}, {b, d, c}}) {
        mesh.mFaces[face].mNumIndices = 3;
        mesh.mFaces[face++].mIndices
            = new unsigned[3]{triangle[0], triangle[1], triangle[2]};
      }
    }
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned[1]{0};
  return scene;
}
dart::simulation::WorldPtr buildScene(const Options& o)
{
  using namespace dart::dynamics;
  auto world = dart::simulation::World::create("renderer-benchmark");
  world->setGravity({0, 0, -9.81});
  auto floor = std::make_shared<SimpleFrame>(Frame::World(), "floor");
  floor->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(4, 4, 0.05)));
  floor->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.35, 0.38, 0.4, 1));
  floor->getVisualAspect()->setShadowed(true);
  world->addSimpleFrame(floor);
  const double objectSize = std::min(
      0.35, 1.5 / std::ceil(std::cbrt(static_cast<double>(o.count))));
  if (o.scene == "pointline") {
    std::vector<Eigen::Vector3d> points;
    points.reserve(o.count);
    auto lines = std::make_shared<LineSegmentShape>(2.0f);
    for (std::size_t i = 0; i < o.count; ++i) {
      const double t
          = static_cast<double>(i) / std::max<std::size_t>(1, o.count - 1);
      const Eigen::Vector3d p(
          1.3 * std::cos(t * 24), 1.3 * std::sin(t * 24), 0.2 + t * 2);
      points.push_back(p);
      lines->addVertex(p + Eigen::Vector3d(0.03, 0, 0));
    }
    auto cloud = std::make_shared<PointCloudShape>(0.025);
    cloud->setPoint(points);
    cloud->setPointShapeType(PointCloudShape::BILLBOARD_SQUARE);
    for (const auto& shape : std::vector<ShapePtr>{cloud, lines}) {
      auto frame = std::make_shared<SimpleFrame>(Frame::World(), "pointline");
      frame->setShape(shape);
      frame->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.7, 0.5, 0.2, 1));
      frame->getVisualAspect()->setShadowed(false);
      world->addSimpleFrame(frame);
    }
    return world;
  }
  auto box = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(objectSize));
  ShapePtr mesh;
  if (o.scene == "mesh") {
    DART_SUPPRESS_DEPRECATED_BEGIN
    mesh = std::make_shared<MeshShape>(
        Eigen::Vector3d::Constant(objectSize),
        meshScene(),
        "renderer-benchmark.obj");
    DART_SUPPRESS_DEPRECATED_END
  }
  for (std::size_t i = 0; i < o.count; ++i) {
    if (o.scene == "soft") {
      auto skeleton = Skeleton::create("soft-" + std::to_string(i));
      const SoftBodyNode::Properties properties(
          BodyNode::Properties(),
          SoftBodyNodeHelper::makeBoxProperties(
              Eigen::Vector3d::Constant(objectSize),
              Eigen::Isometry3d::Identity(),
              1.0));
      auto* body = skeleton
                       ->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>(
                           nullptr, FreeJoint::Properties(), properties)
                       .second;
      auto* node = body->getShapeNode(0);
      node->getVisualAspect()->setRGBA(Eigen::Vector4d(0.7, 0.5, 0.2, 1));
      node->getVisualAspect()->setShadowed(true);
      Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
      transform.translation() = placement(i, o.count);
      FreeJoint::setTransformOf(body, transform);
      world->addSkeleton(skeleton);
    } else {
      auto frame = std::make_shared<SimpleFrame>(
          Frame::World(), "object-" + std::to_string(i));
      frame->setShape(o.scene == "mesh" ? mesh : box);
      frame->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.7, 0.5, 0.2, 1));
      frame->getVisualAspect()->setShadowed(true);
      frame->setTranslation(placement(i, o.count));
      world->addSimpleFrame(frame);
    }
  }
  return world;
}
struct Hash
{
  std::uint64_t value = 14695981039346656037ULL;
  void scalar(double v)
  {
    if (!std::isfinite(v))
      throw std::runtime_error("Nonfinite benchmark scene");
    std::uint64_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    for (int i = 0; i < 8; ++i) {
      value ^= (bits >> (i * 8)) & 255;
      value *= 1099511628211ULL;
    }
  }
  template <typename Derived>
  void vector(const Eigen::MatrixBase<Derived>& v)
  {
    for (Eigen::Index i = 0; i < v.size(); ++i)
      scalar(static_cast<double>(v.derived().coeff(i)));
  }
};
void hashMeshNode(Hash& hash, const aiNode& node)
{
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      hash.scalar(node.mTransformation[i][j]);
  for (unsigned i = 0; i < node.mNumMeshes; ++i)
    hash.scalar(node.mMeshes[i]);
  for (unsigned i = 0; i < node.mNumChildren; ++i)
    hashMeshNode(hash, *node.mChildren[i]);
}
std::uint64_t stateHash(const dart::simulation::World& world)
{
  using namespace dart::dynamics;
  Hash hash;
  hash.scalar(world.getTime());
  hash.scalar(world.getTimeStep());
  std::unordered_set<const Shape*> resources;
  auto shapeFrame = [&](const ShapeFrame& frame) {
    hash.vector(frame.getWorldTransform().matrix());
    const auto* visual = frame.getVisualAspect();
    if (!visual)
      throw std::runtime_error("Benchmark visual aspect missing");
    hash.vector(visual->getRGBA());
    hash.scalar(visual->isHidden() ? 0 : 1);
    hash.scalar(visual->getShadowed() ? 1 : 0);
    const auto shape = frame.getShape();
    if (!shape || !resources.insert(shape.get()).second)
      return;
    if (const auto* box = dynamic_cast<const BoxShape*>(shape.get())) {
      hash.scalar(1);
      hash.vector(box->getSize());
    } else if (const auto* mesh = dynamic_cast<const MeshShape*>(shape.get())) {
      hash.scalar(2);
      hash.vector(mesh->getScale());
      hash.scalar(mesh->getColorMode());
      hash.scalar(mesh->getAlphaMode());
      DART_SUPPRESS_DEPRECATED_BEGIN
      const auto* scene = mesh->getMesh();
      DART_SUPPRESS_DEPRECATED_END
      hashMeshNode(hash, *scene->mRootNode);
      for (unsigned i = 0; i < scene->mNumMeshes; ++i) {
        const auto& part = *scene->mMeshes[i];
        for (unsigned v = 0; v < part.mNumVertices; ++v) {
          hash.vector(Eigen::Vector3d(
              part.mVertices[v].x, part.mVertices[v].y, part.mVertices[v].z));
          if (part.mNormals)
            hash.vector(Eigen::Vector3d(
                part.mNormals[v].x, part.mNormals[v].y, part.mNormals[v].z));
        }
        for (unsigned f = 0; f < part.mNumFaces; ++f)
          for (unsigned v = 0; v < part.mFaces[f].mNumIndices; ++v)
            hash.scalar(part.mFaces[f].mIndices[v]);
      }
    } else if (
        const auto* cloud = dynamic_cast<const PointCloudShape*>(shape.get())) {
      hash.scalar(3);
      hash.scalar(cloud->getPointShapeType());
      hash.scalar(cloud->getVisualSize());
      for (const auto& p : cloud->getPoints())
        hash.vector(p);
    } else if (
        const auto* lines
        = dynamic_cast<const LineSegmentShape*>(shape.get())) {
      hash.scalar(4);
      hash.scalar(lines->getThickness());
      for (const auto& p : lines->getVertices())
        hash.vector(p);
      for (const auto& c : lines->getConnections())
        hash.vector(c);
    } else if (
        const auto* soft = dynamic_cast<const SoftMeshShape*>(shape.get())) {
      hash.scalar(5);
      const auto* body = soft->getSoftBodyNode();
      for (std::size_t i = 0; i < body->getNumPointMasses(); ++i)
        hash.vector(body->getPointMass(i)->getLocalPosition());
      for (std::size_t i = 0; i < body->getNumFaces(); ++i)
        hash.vector(body->getFace(i));
    } else
      throw std::runtime_error("Unexpected benchmark shape type");
  };
  hash.scalar(world.getNumSimpleFrames());
  hash.scalar(world.getNumSkeletons());
  for (std::size_t i = 0; i < world.getNumSimpleFrames(); ++i)
    shapeFrame(*world.getSimpleFrame(i));
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto skeleton = world.getSkeleton(i);
    hash.vector(skeleton->getPositions());
    hash.vector(skeleton->getVelocities());
    hash.vector(skeleton->getAccelerations());
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j)
      skeleton->getBodyNode(j)->eachShapeNode(
          [&](const ShapeNode* node) { shapeFrame(*node); });
  }
  return hash.value;
}
struct Result
{
  std::size_t frames = 0;
  double renderSum = 0, renderSquares = 0,
         renderMin = std::numeric_limits<double>::infinity(), renderMax = 0;
  double setup = 0, readback = 0, png = 0, worldTime = 0;
  std::uint64_t hash = 0;
  bool pixels = false;
  std::string backend, device;
};
bool guardPixels(const std::vector<std::uint8_t>& pixels, int width, int height)
{
  if (pixels.size() != static_cast<std::size_t>(width) * height * 4)
    return false;
  // Every workload is ochre; the blue-gray floor alone must fail the guard.
  std::size_t objectPixels = 0;
  for (std::size_t i = 0; i < pixels.size(); i += 4)
    if (pixels[i] > pixels[i + 1] + 12 && pixels[i + 1] > pixels[i + 2] + 12)
      ++objectPixels;
  return objectPixels >= 8;
}
void checkPixelGuard()
{
  std::vector<std::uint8_t> floorOnly(8 * 8 * 4, 255);
  for (std::size_t i = 0; i < floorOnly.size(); i += 4) {
    floorOnly[i] = 89;
    floorOnly[i + 1] = 97;
    floorOnly[i + 2] = 102;
  }
  floorOnly[0] = 31;
  if (guardPixels(floorOnly, 8, 8))
    throw std::logic_error("Pixel guard accepted the floor without objects");
  for (std::size_t i = 0; i < 8 * 4; i += 4) {
    floorOnly[i] = 179;
    floorOnly[i + 1] = 128;
    floorOnly[i + 2] = 51;
  }
  if (!guardPixels(floorOnly, 8, 8) || guardPixels(floorOnly, 7, 8))
    throw std::logic_error("Pixel guard failed its object/size self-check");
}
void writePNG(
    const std::string& path,
    const std::vector<std::uint8_t>& pixels,
    int width,
    int height)
{
  png_image image{};
  image.version = PNG_IMAGE_VERSION;
  image.width = static_cast<png_uint_32>(width);
  image.height = static_cast<png_uint_32>(height);
  image.format = PNG_FORMAT_RGBA;
  if (!png_image_write_to_file(
          &image, path.c_str(), 0, pixels.data(), 0, nullptr)) {
    const std::string error = image.message;
    png_image_free(&image);
    throw std::runtime_error("Benchmark PNG failed: " + error);
  }
  png_image_free(&image);
}
Result runOne(
    const Options& o, std::size_t frameBudget, double secondsBudget, int reset)
{
  Result result;
  auto begin = Clock::now();
  auto world = buildScene(o);
  const auto built = Clock::now();
  const auto before = stateHash(*world);
  const auto guarded = Clock::now();
  const Eigen::Vector3d eye(5, -6, 4.5), center(0, 0, 1), up(0, 0, 1);
  const Eigen::Vector4d background(0.12, 0.14, 0.18, 1);
  dart::gui::Viewer viewer;
  auto node = std::make_shared<dart::gui::WorldNode>(world);
  node->setShadowsEnabled(o.shadows);
  viewer.addWorldNode(node);
  viewer.simulate(false);
  viewer.allowSimulation(false);
  viewer.setCameraHome(eye, center, up);
  viewer.setVerticalFieldOfView(45);
  viewer.setBackgroundColor(background);
  viewer.setUpViewForCapture(o.width, o.height);
  auto frame = [&]() {
    viewer.frame();
  };
  frame();
  result.setup = milliseconds((built - begin) + (Clock::now() - guarded));
  for (int i = 0; i < o.warmup; ++i)
    frame();
  std::cout << "DART_GUI_BENCHMARK_PHASE measured_begin\n" << std::flush;
  const auto start = Clock::now();
  const auto deadline = start
                        + std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(secondsBudget));
  while (frameBudget > 0 ? result.frames < frameBudget
                         : Clock::now() < deadline) {
    const auto t = Clock::now();
    frame();
    const double elapsed = milliseconds(Clock::now() - t);
    result.renderSum += elapsed;
    result.renderSquares += elapsed * elapsed;
    result.renderMin = std::min(result.renderMin, elapsed);
    result.renderMax = std::max(result.renderMax, elapsed);
    ++result.frames;
  }
  std::cout << "DART_GUI_BENCHMARK_PHASE measured_end\n" << std::flush;
  if (result.frames == 0)
    throw std::runtime_error("No completed measured frames");
  const auto readStart = Clock::now();
  const auto pixels = dart::gui::detail::captureViewerPixelsRGBA(viewer);
  result.readback = milliseconds(Clock::now() - readStart);
  result.pixels = guardPixels(pixels, o.width, o.height);
  result.hash = stateHash(*world);
  result.worldTime = world->getTime();
  if (result.hash != before || result.worldTime != 0)
    throw std::runtime_error("Paused scene changed during rendering");
  result.backend = viewer.getRenderBackend();
  result.device = viewer.getRenderDevice();
  if (!o.png.empty()) {
    std::filesystem::path output = o.png;
    if (o.resets > 0)
      output = output.parent_path()
               / (output.stem().string() + "-" + std::to_string(reset)
                  + output.extension().string());
    const auto t = Clock::now();
    writePNG(output.string(), pixels, o.width, o.height);
    result.png = milliseconds(Clock::now() - t);
  }
  if (!result.pixels)
    throw std::runtime_error(
        "Completed frame has no workload-colored objects or incorrect "
        "dimensions");
  return result;
}
std::string quote(const std::string& input)
{
  std::string output = "\"";
  constexpr char digits[] = "0123456789abcdef";
  for (unsigned char c : input) {
    if (c == '"' || c == '\\') {
      output += '\\';
      output += c;
    } else if (c < 32) {
      output += "\\u00";
      output += digits[c >> 4];
      output += digits[c & 15];
    } else
      output += c;
  }
  return output + '"';
}
std::size_t peakRSS()
{
#if defined(_WIN32)
  PROCESS_MEMORY_COUNTERS memory{};
  if (K32GetProcessMemoryInfo(GetCurrentProcess(), &memory, sizeof(memory)))
    return memory.PeakWorkingSetSize / 1024;
  return 0;
#else
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0)
    return 0;
  #if defined(__APPLE__)
  return static_cast<std::size_t>(usage.ru_maxrss) / 1024;
  #else
  return static_cast<std::size_t>(usage.ru_maxrss);
  #endif
#endif
}
} // namespace

int main(int argc, char** argv)
{
  try {
    const Options o = parse(argc, argv);
    checkPixelGuard();
    Result total;
    for (int reset = 0; reset <= o.resets; ++reset) {
      const std::size_t lifetimes = static_cast<std::size_t>(o.resets) + 1;
      const std::size_t frames
          = o.frames / lifetimes
            + (static_cast<std::size_t>(reset) < o.frames % lifetimes ? 1 : 0);
      const auto r = runOne(o, frames, o.seconds / lifetimes, reset);
      if (reset
          && (r.hash != total.hash || r.backend != total.backend
              || r.device != total.device))
        throw std::runtime_error(
            "Scene or graphics device changed after viewer reset");
      total.frames += r.frames;
      total.renderSum += r.renderSum;
      total.renderSquares += r.renderSquares;
      total.renderMin = std::min(total.renderMin, r.renderMin);
      total.renderMax = std::max(total.renderMax, r.renderMax);
      total.setup += r.setup;
      total.readback += r.readback;
      total.png += r.png;
      total.hash = r.hash;
      total.worldTime = r.worldTime;
      total.pixels = r.pixels;
      total.backend = r.backend;
      total.device = r.device;
    }
    const double mean = total.renderSum / total.frames;
    const double deviation = std::sqrt(
        std::max(0.0, total.renderSquares / total.frames - mean * mean));
    std::cout << std::setprecision(12) << "{\"renderer\":" << quote(o.renderer)
              << ",\"scene\":" << quote(o.scene) << ",\"count\":" << o.count
              << ",\"frames\":" << total.frames << ",\"warmup\":" << o.warmup
              << ",\"resets\":" << o.resets
              << ",\"shadows\":" << (o.shadows ? "true" : "false")
              << ",\"render_ms\":" << mean
              << ",\"render_total_ms\":" << total.renderSum
              << ",\"render_min_ms\":" << total.renderMin
              << ",\"render_max_ms\":" << total.renderMax
              << ",\"render_stddev_ms\":" << deviation
              << ",\"setup_ms\":" << total.setup / (o.resets + 1)
              << ",\"readback_full_frame_ms\":"
              << total.readback / (o.resets + 1) << ",\"png_ms\":"
              << (o.png.empty() ? "null"
                                : std::to_string(total.png / (o.resets + 1)))
              << ",\"width\":" << o.width << ",\"height\":" << o.height
              << ",\"fovy_degrees\":45,\"near_plane\":0.01,\"far_plane\":1000"
              << ",\"backend\":" << quote(total.backend)
              << ",\"device\":" << quote(total.device)
              << ",\"world_time\":" << total.worldTime
              << ",\"state_hash\":" << quote(std::to_string(total.hash))
              << ",\"guard_pixels\":" << (total.pixels ? "true" : "false")
              << ",\"peak_rss_kib\":" << peakRSS()
              << ",\"completion_boundary\":"
              << quote("Filament 1.77.3 flushAndWait GPU finish") << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "gui_render_benchmark: " << error.what() << '\n';
    return 1;
  }
}
