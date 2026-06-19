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
#include <dart/gui/renderable.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include <cmath>
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

} // namespace
