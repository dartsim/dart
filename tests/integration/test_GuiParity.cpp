// Copyright (c) 2026, The DART development contributors.
// SPDX-License-Identifier: BSD-2-Clause

#include <dart/gui/DebugOverlay.hpp>
#include <dart/gui/GridVisual.hpp>
#include <dart/gui/ImGuiWidget.hpp>
#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/WorldNode.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/ArrowShape.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/HeightmapShape.hpp>
#include <dart/dynamics/LineSegmentShape.hpp>
#include <dart/dynamics/MultiSphereConvexHullShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PointCloudShape.hpp>
#include <dart/dynamics/PointMass.hpp>
#include <dart/dynamics/PyramidShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/VoxelGridShape.hpp>

#include <dart/common/Deprecated.hpp>

#include <dart/external/imgui/imgui.h>

#include <assimp/material.h>
#include <assimp/scene.h>
#include <gtest/gtest.h>
#include <png.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cstdlib>

namespace {
using namespace dart;
constexpr unsigned width = 320, height = 240;
const Eigen::Vector4d blueBackground(0.03, 0.05, 0.65, 1);
const Eigen::Vector4d red(0.95, 0.02, 0.02, 1);
const Eigen::Vector4d green(0.02, 0.95, 0.02, 1);

std::filesystem::path artifactDirectory()
{
  const auto* configured = std::getenv("DART_FILAMENT_PARITY_DIR");
  auto path = configured ? std::filesystem::path(configured)
                         : std::filesystem::temp_directory_path()
                               / "dart-filament-parity";
  std::filesystem::create_directories(path);
  return path;
}

struct Image
{
  std::vector<unsigned char> pixels;

  explicit Image(const std::filesystem::path& path)
  {
    png_image image{};
    image.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_file(&image, path.string().c_str()))
      throw std::runtime_error(image.message);
    if (image.width != width || image.height != height) {
      png_image_free(&image);
      throw std::runtime_error("Wrong physical capture dimensions");
    }
    image.format = PNG_FORMAT_RGBA;
    pixels.resize(PNG_IMAGE_SIZE(image));
    if (!png_image_finish_read(&image, nullptr, pixels.data(), 0, nullptr)) {
      const std::string error = image.message;
      png_image_free(&image);
      throw std::runtime_error(error);
    }
    png_image_free(&image);
  }

  template <class Predicate>
  std::size_t count(
      Predicate predicate,
      unsigned x0 = 0,
      unsigned y0 = 0,
      unsigned x1 = width,
      unsigned y1 = height) const
  {
    std::size_t result = 0;
    for (unsigned y = y0; y < y1; ++y)
      for (unsigned x = x0; x < x1; ++x)
        result += predicate(&pixels[(y * width + x) * 4]);
    return result;
  }

  template <class Predicate>
  double meanX(Predicate predicate) const
  {
    double sum = 0;
    std::size_t matches = 0;
    for (unsigned y = 0; y < height; ++y)
      for (unsigned x = 0; x < width; ++x)
        if (predicate(&pixels[(y * width + x) * 4])) {
          sum += x;
          ++matches;
        }
    if (!matches)
      throw std::runtime_error("Expected colored geometry is absent");
    return sum / matches;
  }
};

bool isRed(const unsigned char* p)
{
  return p[0] > p[1] + 30 && p[0] > p[2] + 30;
}
bool isGreen(const unsigned char* p)
{
  return p[1] > p[0] + 30 && p[1] > p[2] + 30;
}
bool isWhite(const unsigned char* p)
{
  return p[0] > 170 && p[1] > 170 && p[2] > 170;
}

Image capture(gui::Viewer& viewer, const std::string& name)
{
  const auto path = artifactDirectory() / (name + ".png");
  viewer.captureScreen(path.string());
  viewer.frame();
  return Image(path);
}

std::shared_ptr<dynamics::SimpleFrame> visualFrame(
    const dynamics::ShapePtr& shape, const Eigen::Vector4d& color = red)
{
  auto frame = std::make_shared<dynamics::SimpleFrame>();
  frame->setShape(shape);
  frame->createVisualAspect()->setColor(color);
  return frame;
}

void setUp(gui::Viewer& viewer, const simulation::WorldPtr& world)
{
  viewer.addWorldNode(std::make_shared<gui::WorldNode>(world));
  viewer.setCameraHome({0, -4, 2}, {0, 0, 0}, {0, 0, 1});
  viewer.setUpViewForCapture(width, height);
}

std::shared_ptr<dynamics::MeshShape> coloredMesh()
{
  auto scene = std::make_shared<aiScene>();
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial* [1] {
    new aiMaterial()
  };
  const aiColor4D materialColor(1, 0, 0, 0.25f);
  scene->mMaterials[0]->AddProperty(&materialColor, 1, AI_MATKEY_COLOR_DIFFUSE);
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh* [1] {
    new aiMesh()
  };
  auto& mesh = *scene->mMeshes[0];
  mesh.mNumVertices = 4;
  mesh.mVertices = new aiVector3D[4]{
      {-0.5f, 0, -0.5f}, {0.5f, 0, -0.5f}, {0.5f, 0, 0.5f}, {-0.5f, 0, 0.5f}};
  mesh.mNormals
      = new aiVector3D[4]{{0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}};
  mesh.mColors[0] = new aiColor4D[4]{
      {0, 1, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 1}};
  mesh.mNumFaces = 2;
  mesh.mFaces = new aiFace[2];
  mesh.mFaces[0].mNumIndices = mesh.mFaces[1].mNumIndices = 3;
  mesh.mFaces[0].mIndices = new unsigned[3]{0, 1, 2};
  mesh.mFaces[1].mIndices = new unsigned[3]{0, 2, 3};
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumChildren = 1;
  scene->mRootNode->mChildren = new aiNode* [1] {
    new aiNode()
  };
  auto& child = *scene->mRootNode->mChildren[0];
  child.mParent = scene->mRootNode;
  child.mTransformation.a4 = 0.45f;
  child.mNumMeshes = 1;
  child.mMeshes = new unsigned[1]{0};
  DART_SUPPRESS_DEPRECATED_BEGIN
  auto shape
      = std::make_shared<dynamics::MeshShape>(Eigen::Vector3d::Ones(), scene);
  DART_SUPPRESS_DEPRECATED_END
  return shape;
}

std::string customFont()
{
  if (const auto* configured = std::getenv("DART_FILAMENT_TEST_FONT"))
    return configured;
#if defined(_WIN32)
  return "C:/Windows/Fonts/arial.ttf";
#elif defined(__APPLE__)
  return "/System/Library/Fonts/Supplemental/Arial.ttf";
#else
  return "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";
#endif
}
} // namespace

TEST(GuiParity, ShapeGalleryHasColoredGeometryInTheExpectedRegion)
{
  using namespace dynamics;
  auto world = simulation::World::create();
  gui::Viewer viewer(blueBackground);
  setUp(viewer, world);
  auto heightmap = std::make_shared<HeightmapShapef>();
  HeightmapShapef::HeightField heights(3, 3);
  heights << 0, 0.1f, 0, 0.1f, 0.3f, 0.1f, 0, 0.1f, 0;
  heightmap->setHeightField(heights);
  heightmap->setScale(Eigen::Vector3f(0.4f, 0.4f, 1));
  std::vector<std::pair<std::string, ShapePtr>> shapes{
      {"box", std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.8))},
      {"sphere", std::make_shared<SphereShape>(0.4)},
      {"ellipsoid",
       std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.7, 0.5, 1))},
      {"cylinder", std::make_shared<CylinderShape>(0.3, 0.8)},
      {"capsule", std::make_shared<CapsuleShape>(0.25, 0.5)},
      {"cone", std::make_shared<ConeShape>(0.4, 0.8)},
      {"pyramid", std::make_shared<PyramidShape>(0.8, 0.8, 0.8)},
      {"multi-sphere",
       std::make_shared<MultiSphereConvexHullShape>(
           MultiSphereConvexHullShape::Spheres{
               {0.25, Eigen::Vector3d(-0.2, 0, 0)},
               {0.25, Eigen::Vector3d(0.2, 0, 0)}})},
      {"plane", std::make_shared<PlaneShape>(Eigen::Vector3d::UnitZ(), 0.0)},
      {"convex",
       std::make_shared<ConvexMeshShape>(
           ConvexMeshShape::Vertices{
               {-0.4, -0.3, -0.3},
               {0.4, -0.3, -0.3},
               {0, 0.4, -0.3},
               {0, 0, 0.5}},
           ConvexMeshShape::Triangles{
               {0, 1, 3}, {1, 2, 3}, {2, 0, 3}, {0, 2, 1}})},
      {"arrow",
       std::make_shared<ArrowShape>(
           Eigen::Vector3d(-0.4, 0, 0),
           Eigen::Vector3d(0.4, 0, 0),
           ArrowShape::Properties(0.08),
           red)},
      {"heightmap-float", heightmap},
      {"connected-lines",
       std::make_shared<LineSegmentShape>(
           Eigen::Vector3d(-0.4, 0, -0.3), Eigen::Vector3d(0, 0, 0.4), 4.0f)}};
  std::static_pointer_cast<LineSegmentShape>(shapes.back().second)
      ->addVertex({0.4, 0, -0.3});
  for (int mode = PointCloudShape::BOX; mode <= PointCloudShape::POINT;
       ++mode) {
    auto cloud = std::make_shared<PointCloudShape>(
        mode == PointCloudShape::POINT ? 8.0 : 0.25);
    cloud->setPointShapeType(
        static_cast<PointCloudShape::PointShapeType>(mode));
    cloud->addPoint({-0.2, 0, 0});
    cloud->addPoint({0.2, 0, 0});
    shapes.emplace_back("point-mode-" + std::to_string(mode), cloud);
  }
#if HAVE_OCTOMAP
  auto voxels = std::make_shared<VoxelGridShape>(0.3);
  voxels->updateOccupancy(Eigen::Vector3d(0.1, 0.1, 0.1));
  voxels->updateOccupancy(Eigen::Vector3d(-0.2, 0.1, 0.1));
  shapes.emplace_back("voxels", voxels);
#endif
  for (const auto& [name, shape] : shapes) {
    SCOPED_TRACE(name);
    auto frame = visualFrame(shape);
    if (shape == heightmap)
      frame->setTranslation({-0.4, 0.4, 0});
    world->addSimpleFrame(frame);
    const auto image = capture(viewer, "gallery-" + name);
    EXPECT_GT(image.count(isRed, 80, 55, 240, 185), 12u);
    EXPECT_EQ(image.count(isRed, 0, 0, 40, 40), 0u);
    EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
    world->removeSimpleFrame(frame);
  }
  const auto empty = capture(viewer, "gallery-removed");
  EXPECT_EQ(empty.count(isRed), 0u);
}

TEST(GuiParity, LiveColorsDimensionsAndConnectedLineEditsReachPixels)
{
  auto world = simulation::World::create();
  auto box
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.5));
  auto frame = visualFrame(box);
  world->addSimpleFrame(frame);
  gui::Viewer viewer(blueBackground);
  setUp(viewer, world);
  const auto before = capture(viewer, "live-box-before");
  ASSERT_GT(before.count(isRed), 100u);
  box->setSize(Eigen::Vector3d::Constant(0.9));
  frame->getVisualAspect()->setColor(green);
  const auto after = capture(viewer, "live-box-after");
  EXPECT_EQ(after.count(isRed), 0u);
  EXPECT_GT(after.count(isGreen), before.count(isRed) * 2);
  auto line = std::make_shared<dynamics::LineSegmentShape>(
      Eigen::Vector3d(-0.5, 0, 0), Eigen::Vector3d(0.5, 0, 0), 3.0f);
  frame->setShape(line);
  frame->getVisualAspect()->setColor(red);
  const auto lineBefore = capture(viewer, "live-line-before");
  ASSERT_GT(lineBefore.count(isRed), 30u);
  line->setVertex(0, {0.3, 0, 0});
  line->setVertex(1, {1.3, 0, 0});
  line->setThickness(7.0f);
  const auto lineAfter = capture(viewer, "live-line-after");
  EXPECT_GT(lineAfter.meanX(isRed), lineBefore.meanX(isRed) + 35);
  EXPECT_GT(lineAfter.count(isRed), lineBefore.count(isRed) * 3 / 2);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
}

TEST(GuiParity, MeshHierarchyColorsAndAlphaModesReachPixels)
{
  auto shape = coloredMesh();
  shape->setColorMode(dynamics::MeshShape::MATERIAL_COLOR);
  shape->setAlphaMode(dynamics::MeshShape::AUTO);
  auto frame = visualFrame(shape);
  auto world = simulation::World::create();
  world->addSimpleFrame(frame);
  gui::Viewer viewer(blueBackground);
  setUp(viewer, world);
  viewer.setCameraHome({0, -4, 0}, {0, 0, 0}, {0, 0, 1});
  const auto transparent = capture(viewer, "material-auto-alpha");
  shape->setAlphaMode(dynamics::MeshShape::SHAPE_ALPHA);
  const auto opaque = capture(viewer, "material-shape-alpha");
  ASSERT_GT(opaque.count(isRed), 500u);
  EXPECT_GT(opaque.meanX(isRed), width / 2.0 + 20);
  const unsigned sample = (height / 2 * width + 190) * 4;
  EXPECT_GT(opaque.pixels[sample], transparent.pixels[sample] + 30);
  EXPECT_LT(opaque.pixels[sample + 2], transparent.pixels[sample + 2] - 20);
  shape->setColorMode(dynamics::MeshShape::COLOR_INDEX);
  shape->setColorIndex(0);
  const auto vertex = capture(viewer, "material-vertex-color");
  EXPECT_EQ(vertex.count(isRed), 0u);
  EXPECT_GT(vertex.count(isGreen), 500u);
  shape->setColorMode(dynamics::MeshShape::SHAPE_COLOR);
  const auto aspect = capture(viewer, "material-shape-color");
  EXPECT_EQ(aspect.count(isGreen), 0u);
  EXPECT_GT(aspect.count(isRed), 500u);
}

TEST(GuiParity, SoftPointEditsMoveTheVisibleMeshWithoutStepping)
{
  auto skeleton = dynamics::Skeleton::create("visible-soft");
  const auto softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
      Eigen::Vector3d::Constant(0.8), Eigen::Isometry3d::Identity(), 1.0);
  const dynamics::SoftBodyNode::Properties properties(
      dynamics::BodyNode::Properties(), softProperties);
  auto* body = skeleton
                   ->createJointAndBodyNodePair<
                       dynamics::FreeJoint,
                       dynamics::SoftBodyNode>(
                       nullptr, dynamics::FreeJoint::Properties(), properties)
                   .second;
  ASSERT_EQ(body->getNumShapeNodes(), 1u);
  ASSERT_EQ(
      body->getShapeNode(0)->getShape()->getType(),
      dynamics::SoftMeshShape::getStaticType());
  body->getShapeNode(0)->getVisualAspect()->setColor(red);
  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  gui::Viewer viewer(blueBackground);
  setUp(viewer, world);
  const auto before = capture(viewer, "soft-before");
  ASSERT_GT(before.count(isRed), 100u);
  const Eigen::Vector3d displacement(0.7, 0, 0);
  for (std::size_t i = 0; i < body->getNumPointMasses(); ++i)
    body->getPointMass(i)->setPositions(displacement);
  EXPECT_TRUE(body->getPointMass(0)->getPositions().isApprox(displacement));
  const auto after = capture(viewer, "soft-after");
  EXPECT_GT(after.meanX(isRed), before.meanX(isRed) + 35);
  EXPECT_GT(after.count(isRed), before.count(isRed) / 2);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
}

TEST(GuiParity, RecordingPauseResumeAndRestartPreserveSequence)
{
  auto directory
      = artifactDirectory()
        / ("recording-"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  auto world = simulation::World::create();
  auto frame = visualFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.8)));
  world->addSimpleFrame(frame);
  gui::Viewer viewer(blueBackground);
  setUp(viewer, world);
  viewer.record(directory.string(), "frame-", true, 3);
  ASSERT_TRUE(viewer.isRecording());
  viewer.frame();
  frame->getVisualAspect()->setColor(green);
  viewer.frame();
  ASSERT_GT(Image(directory / "frame-000.png").count(isRed), 100u);
  ASSERT_GT(Image(directory / "frame-001.png").count(isGreen), 100u);
  viewer.pauseRecording();
  EXPECT_FALSE(viewer.isRecording());
  viewer.frame();
  EXPECT_FALSE(std::filesystem::exists(directory / "frame-002.png"));
  viewer.record(directory.string(), "frame-", false, 3);
  viewer.frame();
  EXPECT_GT(Image(directory / "frame-002.png").count(isGreen), 100u);
  viewer.record(directory.string(), "restarted-", true, 2);
  frame->getVisualAspect()->setColor(red);
  viewer.frame();
  EXPECT_GT(Image(directory / "restarted-00.png").count(isRed), 100u);
  EXPECT_FALSE(std::filesystem::exists(directory / "restarted-01.png"));
}

TEST(GuiParity, BodyWireframeAndSharedWorldStepOwnership)
{
  auto skeleton = dynamics::Skeleton::create();
  auto* left
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  auto* right
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  left->getParentJoint()->setTransformFromParentBodyNode(
      Eigen::Translation3d(-0.7, 0, 0) * Eigen::Isometry3d::Identity());
  right->getParentJoint()->setTransformFromParentBodyNode(
      Eigen::Translation3d(0.7, 0, 0) * Eigen::Isometry3d::Identity());
  for (auto* body : {left, right})
    body->createShapeNodeWith<dynamics::VisualAspect>(
            std::make_shared<dynamics::BoxShape>(
                Eigen::Vector3d::Constant(0.8)))
        ->getVisualAspect()
        ->setColor(body == left ? red : green);
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(skeleton);
  gui::Viewer first(blueBackground), second(blueBackground);
  setUp(first, world);
  setUp(second, world);
  const auto solid = capture(first, "body-solid");
  ASSERT_GT(solid.count(isRed), 100u);
  ASSERT_GT(solid.count(isGreen), 100u);
  first.setBodyWireframe(left, true);
  EXPECT_TRUE(first.getBodyWireframe(left));
  EXPECT_FALSE(first.getBodyWireframe(right));
  const auto wire = capture(first, "body-left-wireframe");
  EXPECT_GT(wire.count(isRed), 10u);
  EXPECT_LT(wire.count(isRed), solid.count(isRed) / 2);
  EXPECT_GT(wire.count(isGreen), solid.count(isGreen) * 9 / 10);
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
  first.simulate(true);
  second.simulate(true);
  capture(first, "shared-world-first");
  EXPECT_NEAR(world->getTime(), world->getTimeStep(), 1e-12);
  capture(second, "shared-world-second");
  EXPECT_NEAR(world->getTime(), world->getTimeStep(), 1e-12);
  first.simulate(false);
  capture(second, "shared-world-owner-transferred");
  EXPECT_NEAR(world->getTime(), world->getTimeStep() * 2, 1e-12);
  second.simulate(false);
  capture(first, "shared-world-paused-first");
  capture(second, "shared-world-paused-second");
  EXPECT_NEAR(world->getTime(), world->getTimeStep() * 2, 1e-12);
}

TEST(GuiParity, SharedWorldStepOwnershipSurvivesCloseAndRefreshMutations)
{
  auto world = simulation::World::create();
  world->setTimeStep(0.001);
  gui::Viewer first, second;
  first.addWorldNode(std::make_shared<gui::WorldNode>(world));
  second.addWorldNode(std::make_shared<gui::WorldNode>(world));
  first.simulate(true);
  second.simulate(true);
  first.setDone();
  second.setUpViewForCapture(16, 16);
  second.frame();
  EXPECT_DOUBLE_EQ(world->getTime(), 0.001);

  class MutatingNode : public gui::WorldNode
  {
  public:
    using WorldNode::WorldNode;
    std::shared_ptr<simulation::World> replacement;
    std::weak_ptr<MutatingNode> self;
    bool remove = false;
    void customPreRefresh() override
    {
      if (replacement)
        setWorld(std::exchange(replacement, {}));
      if (remove)
        getViewer()->removeWorldNode(self.lock());
    }
  };
  auto original = simulation::World::create();
  auto destination = simulation::World::create();
  original->setTimeStep(0.001);
  destination->setTimeStep(0.001);
  auto mutating = std::make_shared<MutatingNode>(original);
  mutating->self = mutating;
  mutating->replacement = destination;
  gui::Viewer viewer;
  viewer.addWorldNode(mutating);
  viewer.addWorldNode(std::make_shared<gui::WorldNode>(destination));
  viewer.simulate(true);
  viewer.setUpViewForCapture(16, 16);
  viewer.frame();
  EXPECT_DOUBLE_EQ(original->getTime(), 0);
  EXPECT_DOUBLE_EQ(destination->getTime(), 0.001);
  mutating->remove = true;
  viewer.frame();
  EXPECT_EQ(viewer.getNumWorldNodes(), 1u);
  EXPECT_DOUBLE_EQ(destination->getTime(), 0.002);
}

TEST(GuiParity, GridSupportPolygonAndCustomFontAreVisibleAndCanBeHidden)
{
  auto world = simulation::World::create();
  world->setGravity({0, 0, -9.81});
  auto skeleton = dynamics::Skeleton::create();
  auto* body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  skeleton->setGravity({0, 0, -9.81});
  auto* support = body->createEndEffector()->getSupport(true);
  support->setGeometry(
      {{-0.5, -0.5, 0}, {0.5, -0.5, 0}, {0.5, 0.5, 0}, {-0.5, 0.5, 0}});
  support->setActive(true);
  world->addSkeleton(skeleton);
  gui::Viewer viewer(Eigen::Vector4d(0.02, 0.02, 0.02, 1));
  setUp(viewer, world);
  auto grid = std::make_shared<gui::GridVisual>();
  grid->setNumCells(4);
  grid->setMinorLineStepSize(0.3);
  grid->setMajorLineColor({1, 1, 1, 1});
  grid->setMinorLineColor({1, 1, 1, 1});
  grid->setMinorLineWidth(2);
  grid->setMajorLineWidth(2);
  auto polygon = std::make_shared<gui::SupportPolygonVisual>(skeleton, 0.02);
  polygon->setPolygonColor(green);
  polygon->displayCentroid(false);
  polygon->displayCenterOfMass(false);
  auto labels = std::make_shared<gui::DebugOverlay>();
  ASSERT_TRUE(std::filesystem::is_regular_file(customFont())) << customFont();
  labels->setFont(customFont());
  labels->addScreenLabel({12, 12}, "CUSTOM FONT", {1, 1, 1, 1}, 24);
  viewer.addAttachment(grid);
  viewer.addAttachment(polygon);
  viewer.addAttachment(labels);
  const auto visible = capture(viewer, "grid-support-custom-font");
  EXPECT_GT(visible.count(isWhite, 12, 12, 235, 48), 100u);
  EXPECT_GT(visible.count(isWhite, 40, 65, 280, 215), 200u);
  EXPECT_GT(visible.count(isGreen, 120, 95, 200, 160), 200u);
  EXPECT_EQ(labels->getNumLabels(), 1u);
  grid->display(false);
  polygon->displayPolygon(false);
  labels->clear();
  const auto hidden = capture(viewer, "grid-support-font-hidden");
  EXPECT_EQ(hidden.count(isWhite), 0u);
  EXPECT_EQ(hidden.count(isGreen), 0u);
}

TEST(GuiParity, WorldLabelsCenterAndStayBehindWidgetsWhileHudStaysVisible)
{
  class Panel : public gui::ImGuiWidget
  {
  public:
    void render() override
    {
      ImGui::SetNextWindowPos({90, 70});
      ImGui::SetNextWindowSize({140, 100});
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 1));
      ImGui::Begin(
          "opaque-panel",
          nullptr,
          ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoSavedSettings
              | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
      ImGui::End();
      ImGui::PopStyleColor();
    }
  };
  gui::Viewer viewer(blueBackground);
  setUp(viewer, simulation::World::create());
  auto labels = std::make_shared<gui::DebugOverlay>();
  labels->setTextOutlineWidth(0);
  labels->addLabelPixels({0, 0, 0}, "MM\nMM", red, 24);
  viewer.addAttachment(labels);
  const auto centered = capture(viewer, "world-label-centered");
  EXPECT_GT(centered.count(isRed, 120, 85, 200, 120), 10u);
  EXPECT_GT(centered.count(isRed, 120, 120, 200, 155), 10u);
  viewer.addWidget(std::make_shared<Panel>());
  labels->addScreenLabel({100, 80}, "HUD", green, 20);
  const auto covered = capture(viewer, "world-label-covered-hud-visible");
  EXPECT_EQ(covered.count(isRed, 90, 70, 230, 170), 0u);
  EXPECT_GT(covered.count(isGreen, 95, 75, 150, 110), 20u);
}

TEST(GuiParity, CombinedCapturePreservesRefreshRecordingAndOverlayTransitions)
{
  class CountingNode : public gui::WorldNode
  {
  public:
    explicit CountingNode(const simulation::WorldPtr& world) : WorldNode(world)
    {
    }
    void customPreRefresh() override
    {
      ++refreshes;
    }
    unsigned refreshes = 0;
  };
  class Marker : public gui::ImGuiWidget
  {
  public:
    void render() override
    {
      ImGui::GetForegroundDrawList()->AddRectFilled(
          {10, 10}, {50, 40}, IM_COL32(0, 255, 0, 255));
    }
  };
  auto world = simulation::World::create();
  auto box = visualFrame(
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.8)));
  world->addSimpleFrame(box);
  auto node = std::make_shared<CountingNode>(world);
  gui::Viewer viewer(blueBackground);
  viewer.addWorldNode(node);
  viewer.setCameraHome({0, -4, 2}, {0, 0, 0}, {0, 0, 1});
  viewer.setUpViewForCapture(width, height);
  unsigned captures = 0;
  auto combined = [&](const std::string& name) {
    const auto path = artifactDirectory() / (name + ".png");
    viewer.captureScreen(path.string());
    const auto pixels = gui::detail::captureViewerPixelsRGBA(viewer);
    EXPECT_EQ(pixels.size(), width * height * 4u);
    EXPECT_EQ(node->refreshes, ++captures);
    EXPECT_EQ(world->getTime(), 0.0);
    Image image(path);
    EXPECT_EQ(image.pixels, pixels);
    return image;
  };
  const auto empty = combined("combined-overlay-empty");
  EXPECT_GT(empty.count(isRed), 500u);
  EXPECT_EQ(empty.count(isGreen, 10, 10, 50, 40), 0u);

  const auto directory = artifactDirectory() / "combined-recording";
  std::filesystem::create_directories(directory);
  viewer.record(directory.string(), "frame-", true, 2);
  auto marker = std::make_shared<Marker>();
  viewer.addWidget(marker);
  const auto visible = combined("combined-overlay-visible");
  EXPECT_GT(visible.count(isGreen, 10, 10, 50, 40), 1000u);
  EXPECT_EQ(Image(directory / "frame-00.png").pixels, visible.pixels);

  marker->hide();
  box->getVisualAspect()->setColor(green);
  const auto hidden = combined("combined-overlay-hidden");
  EXPECT_EQ(hidden.count(isGreen, 10, 10, 50, 40), 0u);
  EXPECT_GT(hidden.count(isGreen, 110, 70, 210, 180), 500u);
  EXPECT_EQ(Image(directory / "frame-01.png").pixels, hidden.pixels);
  viewer.pauseRecording();
  viewer.setDone(true);
  EXPECT_THROW(
      gui::detail::captureViewerPixelsRGBA(viewer), std::runtime_error);
  EXPECT_EQ(node->refreshes, captures);
}
