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
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

// Covers the renderer-neutral debug-visual and per-shape PBR seams of
// dart::gui: the joint-axis/velocity debug helpers, the per-shape PBR
// material flow from dynamics::VisualAspect into RenderableDescriptor, the
// lit-material override gating by shape kind, and the application
// debugProvider propagation into the runtime scene.

#include <dart/gui/debug.hpp>
#include <dart/gui/detail/scenes.hpp>
#include <dart/gui/offscreen.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/view_quality.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/contact_force.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace dart::gui::detail {
// Declared locally because detail/renderable_factory.hpp pulls Filament math
// headers whose include directories are private to dart-gui-core; the symbol
// itself is exported, and a signature drift fails at link time.
DART_GUI_API bool shapeUsesLitMaterialOverride(dart::gui::ShapeKind kind);
} // namespace dart::gui::detail

namespace {

using dart::dynamics::BoxShape;
using dart::dynamics::FreeJoint;
using dart::dynamics::Skeleton;

void expectFiniteNonZeroLines(
    const std::vector<dart::gui::DebugLineDescriptor>& lines)
{
  for (const auto& line : lines) {
    EXPECT_TRUE(line.from.allFinite()) << line.label;
    EXPECT_TRUE(line.to.allFinite()) << line.label;
    EXPECT_GT((line.to - line.from).norm(), 1e-12) << line.label;
  }
}

bool hasLabel(
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    const std::string& label)
{
  return std::any_of(lines.begin(), lines.end(), [&](const auto& line) {
    return line.label == label;
  });
}

std::size_t countLabel(
    const std::vector<dart::gui::DebugLineDescriptor>& lines,
    const std::string& label)
{
  return static_cast<std::size_t>(
      std::count_if(lines.begin(), lines.end(), [&](const auto& line) {
        return line.label == label;
      }));
}

dart::gui::RenderableDescriptor makeBoundedBoxDescriptor(
    dart::gui::RenderableId id,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& halfExtents)
{
  dart::gui::RenderableDescriptor descriptor;
  descriptor.id = id;
  descriptor.shapeFrameName = "box";
  descriptor.shapeNodeName = "box";
  descriptor.geometry.kind = dart::gui::ShapeKind::Box;
  descriptor.geometry.size = 2.0 * halfExtents;
  descriptor.geometry.localBoundsMin = -halfExtents;
  descriptor.geometry.localBoundsMax = halfExtents;
  descriptor.geometry.hasLocalBounds = true;
  descriptor.material.visible = true;
  descriptor.worldTransform = Eigen::Isometry3d::Identity();
  descriptor.worldTransform.translation() = center;
  return descriptor;
}

TEST(GuiDebugVisuals, JointAxisDebugLinesFollowRevoluteAxis)
{
  auto skeleton = Skeleton::create("joint_axis");
  auto pair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());
  const dart::dynamics::BodyNode* body = pair.second;

  dart::gui::DebugDrawOptions options;
  options.drawJointAxes = false;
  EXPECT_TRUE(dart::gui::makeJointAxisDebugLines(*body, options).empty());

  options.drawJointAxes = true;
  options.jointAxisLength = 0.5;
  const auto lines = dart::gui::makeJointAxisDebugLines(*body, options);
  ASSERT_FALSE(lines.empty());
  const Eigen::Vector3d direction
      = (lines.front().to - lines.front().from).normalized();
  EXPECT_NEAR(std::abs(direction.dot(Eigen::Vector3d::UnitZ())), 1.0, 1e-9);
}

TEST(GuiDebugVisuals, VelocityDebugLinesRespectOptions)
{
  auto skeleton = Skeleton::create("velocity");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  dart::dynamics::BodyNode* body = pair.second;

  dart::gui::DebugDrawOptions options;
  // Both off by default -> nothing drawn even with motion.
  Eigen::VectorXd velocities = Eigen::VectorXd::Zero(6);
  velocities[3] = 2.0; // body-frame linear x velocity
  pair.first->setVelocities(velocities);
  EXPECT_TRUE(dart::gui::makeVelocityDebugLines(*body, options).empty());

  options.drawLinearVelocities = true;
  EXPECT_FALSE(dart::gui::makeVelocityDebugLines(*body, options).empty());

  // Zero velocity produces no arrow even when the option is on.
  pair.first->setVelocities(Eigen::VectorXd::Zero(6));
  EXPECT_TRUE(dart::gui::makeVelocityDebugLines(*body, options).empty());
}

TEST(GuiDebugVisuals, SelectionDebugLinesSkipFlatBoundsDegenerateEdges)
{
  dart::gui::RenderableDescriptor descriptor;
  descriptor.material.visible = true;
  descriptor.geometry.hasLocalBounds = true;
  descriptor.geometry.localBoundsMin = Eigen::Vector3d(-1.0, -2.0, 0.0);
  descriptor.geometry.localBoundsMax = Eigen::Vector3d(1.0, 2.0, 0.0);

  const auto lines = dart::gui::makeSelectionDebugLines(descriptor);
  ASSERT_FALSE(lines.empty());
  expectFiniteNonZeroLines(lines);
}

TEST(GuiDebugVisuals, SelectionDebugLinesSkipZeroVolumeBounds)
{
  dart::gui::RenderableDescriptor descriptor;
  descriptor.material.visible = true;
  descriptor.geometry.hasLocalBounds = true;
  descriptor.geometry.localBoundsMin = Eigen::Vector3d::Ones();
  descriptor.geometry.localBoundsMax = Eigen::Vector3d::Ones();

  EXPECT_TRUE(dart::gui::makeSelectionDebugLines(descriptor).empty());
}

TEST(GuiDebugVisuals, PerShapePbrOverridesFlowThroughDescriptor)
{
  auto skeleton = Skeleton::create("pbr");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(
      std::make_shared<BoxShape>(Eigen::Vector3d::Ones()));
  auto* visual = shapeNode->getVisualAspect();
  EXPECT_LT(visual->getMetallic(), 0.0);
  visual->setMetallic(0.8);
  visual->setRoughness(0.25);

  const auto descriptor = dart::gui::describeShapeFrame(*shapeNode);
  ASSERT_TRUE(descriptor.has_value());
  const auto& material = descriptor->material;
  ASSERT_TRUE(material.metallic.has_value());
  ASSERT_TRUE(material.roughness.has_value());
  EXPECT_NEAR(*material.metallic, 0.8, 1e-9);
  EXPECT_NEAR(*material.roughness, 0.25, 1e-9);
  EXPECT_FALSE(material.reflectance.has_value());

  // The new aspect properties must survive a skeleton clone.
  const auto clone = skeleton->cloneSkeleton();
  auto* cloneVisual = clone->getBodyNode(0)->getShapeNode(0)->getVisualAspect();
  ASSERT_NE(cloneVisual, nullptr);
  EXPECT_NEAR(cloneVisual->getMetallic(), 0.8, 1e-9);
  EXPECT_NEAR(cloneVisual->getRoughness(), 0.25, 1e-9);
  EXPECT_LT(cloneVisual->getReflectance(), 0.0);
}

TEST(GuiDebugVisuals, ShapeUsesLitMaterialOverrideGatesByKind)
{
  using dart::gui::ShapeKind;
  using dart::gui::detail::shapeUsesLitMaterialOverride;

  // Lit primitives accept the per-shape PBR override.
  EXPECT_TRUE(shapeUsesLitMaterialOverride(ShapeKind::Box));
  EXPECT_TRUE(shapeUsesLitMaterialOverride(ShapeKind::Sphere));
  EXPECT_TRUE(shapeUsesLitMaterialOverride(ShapeKind::Capsule));
  EXPECT_TRUE(shapeUsesLitMaterialOverride(ShapeKind::Plane));
  EXPECT_TRUE(shapeUsesLitMaterialOverride(ShapeKind::Heightmap));

  // Asset meshes keep their own materials; unlit renderables have no PBR
  // parameters.
  EXPECT_FALSE(shapeUsesLitMaterialOverride(ShapeKind::Mesh));
  EXPECT_FALSE(shapeUsesLitMaterialOverride(ShapeKind::LineSegments));
  EXPECT_FALSE(shapeUsesLitMaterialOverride(ShapeKind::PointCloud));
  EXPECT_FALSE(shapeUsesLitMaterialOverride(ShapeKind::VoxelGrid));
  EXPECT_FALSE(shapeUsesLitMaterialOverride(ShapeKind::Unsupported));
}

TEST(GuiDebugVisuals, DebugProviderPropagatesToDartScene)
{
  dart::gui::detail::AppOptions appOptions;
  appOptions.debugProvider = [] {
    dart::gui::DebugScene debugScene;
    dart::gui::DebugLineDescriptor line;
    line.to = Eigen::Vector3d::UnitX();
    line.label = "provider.line";
    debugScene.lines.push_back(line);
    dart::gui::DebugTriangleDescriptor triangle;
    triangle.b = Eigen::Vector3d::UnitX();
    triangle.c = Eigen::Vector3d::UnitY();
    triangle.label = "provider.triangle";
    debugScene.triangles.push_back(triangle);
    dart::gui::DebugLabelDescriptor label;
    label.position = Eigen::Vector3d::UnitZ();
    label.text = "provider label";
    debugScene.labels.push_back(label);
    return debugScene;
  };

  const dart::gui::detail::DartScene scene
      = dart::gui::detail::createDartScene(appOptions);
  ASSERT_TRUE(static_cast<bool>(scene.debugProvider));
  const dart::gui::DebugScene provided = scene.debugProvider();
  ASSERT_EQ(provided.lines.size(), 1u);
  EXPECT_EQ(provided.lines.front().label, "provider.line");
  ASSERT_EQ(provided.triangles.size(), 1u);
  EXPECT_EQ(provided.triangles.front().label, "provider.triangle");
  ASSERT_EQ(provided.labels.size(), 1u);
  EXPECT_EQ(provided.labels.front().text, "provider label");
}

TEST(GuiDebugVisuals, ExtractWorldDebugLinesEmitsPerBodyLayers)
{
  dart::simulation::World world;

  dart::simulation::RigidBodyOptions groundOptions;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.05);
  groundOptions.isStatic = true;
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      dart::simulation::CollisionShape::makeBox(
          Eigen::Vector3d(0.8, 0.8, 0.05)));

  dart::simulation::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.3);
  boxOptions.linearVelocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  auto box = world.addRigidBody("box", boxOptions);
  box.setCollisionShape(
      dart::simulation::CollisionShape::makeBox(
          Eigen::Vector3d(0.1, 0.1, 0.1)));

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawBodyFrames = true;
  options.drawCentersOfMass = true;
  options.drawLinearVelocities = true;

  const auto lines = dart::gui::extractDebugLines(world, options);
  ASSERT_FALSE(lines.empty());
  expectFiniteNonZeroLines(lines);

  // Body frames: three axes per body.
  for (const char* axis : {"box.x", "box.y", "box.z", "ground.x"}) {
    EXPECT_TRUE(hasLabel(lines, axis)) << axis;
  }
  // Center-of-mass markers.
  EXPECT_TRUE(hasLabel(lines, "box.com.x"));
  EXPECT_TRUE(hasLabel(lines, "ground.com.x"));
  // Only the moving body draws a velocity arrow.
  EXPECT_TRUE(hasLabel(lines, "box.vel_linear"));
  EXPECT_FALSE(hasLabel(lines, "ground.vel_linear"));
}

TEST(GuiDebugVisuals, ExtractWorldDebugLinesAppliesThicknessExceptGrid)
{
  dart::simulation::World world;

  dart::simulation::RigidBodyOptions boxOptions;
  boxOptions.position = Eigen::Vector3d(0.0, 0.0, 0.3);
  auto box = world.addRigidBody("box", boxOptions);
  box.setCollisionShape(
      dart::simulation::CollisionShape::makeBox(
          Eigen::Vector3d(0.1, 0.1, 0.1)));

  dart::gui::DebugDrawOptions options;
  options.drawGrid = true;
  options.drawWorldFrame = true;
  options.drawBodyFrames = true;
  options.drawCentersOfMass = true;
  options.lineThickness = 0.01;

  const auto lines = dart::gui::extractDebugLines(world, options);
  ASSERT_FALSE(lines.empty());

  bool sawGrid = false;
  bool sawBody = false;
  for (const auto& line : lines) {
    if (line.label == "grid") {
      sawGrid = true;
      EXPECT_DOUBLE_EQ(line.thickness, 0.0) << "grid stays a hairline";
    } else {
      sawBody = true;
      EXPECT_DOUBLE_EQ(line.thickness, 0.01) << line.label;
    }
  }
  EXPECT_TRUE(sawGrid);
  EXPECT_TRUE(sawBody);

  // A zero thickness keeps every line a hairline.
  options.lineThickness = 0.0;
  const auto thinLines = dart::gui::extractDebugLines(world, options);
  for (const auto& line : thinLines) {
    EXPECT_DOUBLE_EQ(line.thickness, 0.0) << line.label;
  }
}

TEST(GuiDebugVisuals, ExtractSimulationContactDebugLinesApplyThickness)
{
  std::vector<dart::simulation::Contact> contacts;
  dart::simulation::Contact contact;
  contact.point = Eigen::Vector3d(0.1, 0.2, 0.3);
  contact.normal = Eigen::Vector3d::UnitZ();
  contacts.push_back(contact);

  dart::gui::DebugDrawOptions options;
  options.lineThickness = 0.008;
  const auto lines = dart::gui::extractContactDebugLines(contacts, options);
  ASSERT_FALSE(lines.empty());
  for (const auto& line : lines) {
    EXPECT_DOUBLE_EQ(line.thickness, 0.008) << line.label;
  }
}

TEST(GuiDebugVisuals, ExtractContactForceDebugLinesDrawArrows)
{
  std::vector<dart::simulation::ContactForce> forces;
  dart::simulation::ContactForce contactForce;
  contactForce.point = Eigen::Vector3d(0.1, 0.2, 0.3);
  contactForce.force = Eigen::Vector3d(0.0, 0.0, 20.0); // 20 N upward
  forces.push_back(contactForce);

  dart::gui::DebugDrawOptions options;
  options.lineThickness = 0.006;
  const auto lines = dart::gui::extractContactForceDebugLines(forces, options);
  ASSERT_FALSE(lines.empty());
  expectFiniteNonZeroLines(lines);

  // Every emitted line is a force arrow drawn at the DART 6 force color and the
  // requested world-space thickness (assert on labels/colors, not counts).
  const Eigen::Vector4d forceColor(0.93, 0.31, 0.67, 1.0);
  for (const auto& line : lines) {
    EXPECT_EQ(line.label, "contact.force");
    EXPECT_TRUE(line.rgba.isApprox(forceColor, 1e-9));
    EXPECT_DOUBLE_EQ(line.thickness, 0.006);
  }

  // The arrow shaft starts at the contact point and points along the force.
  bool sawShaftFromPoint = false;
  for (const auto& line : lines) {
    if (line.from.isApprox(contactForce.point, 1e-9)
        && (line.to - line.from)
               .normalized()
               .isApprox(Eigen::Vector3d::UnitZ(), 1e-6)) {
      sawShaftFromPoint = true;
    }
  }
  EXPECT_TRUE(sawShaftFromPoint);

  // Gated by drawContactForces.
  options.drawContactForces = false;
  EXPECT_TRUE(
      dart::gui::extractContactForceDebugLines(forces, options).empty());
}

TEST(GuiDebugVisuals, ExtractSimulationContactDebugLinesMarkContactPoints)
{
  std::vector<dart::simulation::Contact> contacts;
  dart::simulation::Contact contact;
  contact.point = Eigen::Vector3d(0.1, 0.2, 0.3);
  contact.normal = Eigen::Vector3d::UnitZ();
  contacts.push_back(contact);

  dart::gui::DebugDrawOptions options;
  const auto lines = dart::gui::extractContactDebugLines(contacts, options);
  ASSERT_FALSE(lines.empty());
  expectFiniteNonZeroLines(lines);

  EXPECT_EQ(countLabel(lines, "contact.point"), 2u);
  EXPECT_TRUE(hasLabel(lines, "contact.normal"));

  // Markers must be centered on the reported contact point.
  for (const auto& line : lines) {
    if (line.label == "contact.point") {
      const Eigen::Vector3d midpoint = 0.5 * (line.from + line.to);
      EXPECT_TRUE(midpoint.isApprox(contact.point, 1e-9));
    }
  }
}

TEST(GuiDebugVisuals, DrawDebugLabelTextRendersAntiAliasedGlyphs)
{
  const int width = 160;
  const int height = 40;
  const int channels = 4;
  std::vector<std::uint8_t> pixels(
      static_cast<std::size_t>(width) * height * channels, 0);

  dart::gui::drawDebugLabelText(
      pixels.data(),
      width,
      height,
      channels,
      "Ag1",
      12,
      6,
      Eigen::Vector4d(1.0, 1.0, 1.0, 1.0),
      2,
      18.0,
      false);

  std::size_t lit = 0;
  std::size_t partial = 0;
  std::size_t solid = 0;
  for (std::size_t i = 0; i < pixels.size(); i += channels) {
    const std::uint8_t value = pixels[i];
    if (value > 0) {
      ++lit;
    }
    if (value > 20 && value < 235) {
      ++partial;
    }
    if (value >= 235) {
      ++solid;
    }
  }
  // The atlas font must actually paint pixels, and anti-aliasing means the
  // glyph edges land at intermediate coverage rather than a hard 1-bit mask.
  EXPECT_GT(lit, 40u);
  EXPECT_GT(partial, 0u);
  EXPECT_GT(solid, 0u);

  // Empty text is a no-op.
  std::vector<std::uint8_t> untouched(
      static_cast<std::size_t>(width) * height * channels, 0);
  dart::gui::drawDebugLabelText(
      untouched.data(),
      width,
      height,
      channels,
      "",
      12,
      6,
      Eigen::Vector4d::Ones(),
      2,
      18.0,
      false);
  EXPECT_EQ(
      std::count(untouched.begin(), untouched.end(), std::uint8_t{0}),
      static_cast<std::ptrdiff_t>(untouched.size()));
}

TEST(GuiDebugVisuals, MakePolylineDebugLinesDropsZeroLengthSegments)
{
  const std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0)}; // final duplicate -> zero-length segment
  const auto lines = dart::gui::makePolylineDebugLines(
      points, Eigen::Vector4d(0.98, 0.55, 0.25, 0.95), "path");
  EXPECT_EQ(lines.size(), 2u);
  for (const auto& line : lines) {
    EXPECT_EQ(line.label, "path");
  }

  EXPECT_TRUE(
      dart::gui::makePolylineDebugLines(
          {Eigen::Vector3d::Zero()}, Eigen::Vector4d::Ones(), "p")
          .empty());
}

TEST(GuiDebugVisuals, AssessViewAcceptsAWellFramedSubject)
{
  const dart::gui::RenderableDescriptor box = makeBoundedBoxDescriptor(
      1u, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.25));

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.distance = 2.5;
  camera.yaw = -0.75;
  camera.pitch = 0.35;

  const dart::gui::ViewQualityReport report
      = dart::gui::assessView({box}, camera, 320, 240);

  std::string issues;
  for (const auto& issue : report.issues) {
    issues += issue + " ";
  }
  EXPECT_TRUE(report.issues.empty()) << issues;
  EXPECT_DOUBLE_EQ(report.cornerCoverage, 1.0);
  EXPECT_EQ(report.occlusionFraction, 0.0);
  EXPECT_TRUE(report.centerVisible);
  EXPECT_GT(report.score, 0.0);
  EXPECT_LE(report.score, 1.0);
}

TEST(GuiDebugVisuals, AssessViewIgnoresHiddenRenderables)
{
  const dart::gui::RenderableDescriptor visible = makeBoundedBoxDescriptor(
      1u, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.25));
  dart::gui::RenderableDescriptor hidden = makeBoundedBoxDescriptor(
      2u, Eigen::Vector3d(20.0, 0.0, 0.0), Eigen::Vector3d::Constant(10.0));
  hidden.material.visible = false;

  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.distance = 2.5;
  camera.yaw = -0.75;
  camera.pitch = 0.35;

  const dart::gui::ViewQualityReport visibleOnly
      = dart::gui::assessView({visible}, camera, 320, 240);
  const dart::gui::ViewQualityReport withHidden
      = dart::gui::assessView({visible, hidden}, camera, 320, 240);

  EXPECT_EQ(withHidden.issues, visibleOnly.issues);
  EXPECT_DOUBLE_EQ(withHidden.cornerCoverage, visibleOnly.cornerCoverage);
  EXPECT_DOUBLE_EQ(withHidden.subjectFraction, visibleOnly.subjectFraction);
  EXPECT_DOUBLE_EQ(withHidden.ambiguityIoU, visibleOnly.ambiguityIoU);
  EXPECT_DOUBLE_EQ(withHidden.score, visibleOnly.score);

  const dart::gui::ViewQualityReport hiddenFocus = dart::gui::assessView(
      {visible, hidden},
      camera,
      320,
      240,
      std::vector<dart::gui::RenderableId>{hidden.id});
  EXPECT_EQ(hiddenFocus.issues, std::vector<std::string>{"no-bounded-focus"});
}

TEST(GuiDebugVisuals, AssessViewFlagsAnOccludedFocus)
{
  // Camera at +X looking toward the origin along -X.
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.distance = 2.0;
  camera.yaw = 0.0;
  camera.pitch = 0.0;

  const dart::gui::RenderableDescriptor focus = makeBoundedBoxDescriptor(
      1u, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(0.15));
  // A large, near wall between the eye and the focus blocks every focus ray.
  const dart::gui::RenderableDescriptor wall = makeBoundedBoxDescriptor(
      2u, Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.05, 1.0, 1.0));

  const dart::gui::ViewQualityReport report = dart::gui::assessView(
      {focus, wall},
      camera,
      320,
      240,
      std::vector<dart::gui::RenderableId>{1u});

  EXPECT_GT(report.occlusionFraction, 0.5);
  EXPECT_TRUE(
      std::find(report.issues.begin(), report.issues.end(), "occluded")
      != report.issues.end());
}

} // namespace
