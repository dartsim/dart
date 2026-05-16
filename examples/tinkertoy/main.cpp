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

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/helpers.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <cstddef>

namespace {

constexpr const char* kTinkertoySkeletonPrefix = "visual_tinkertoy_toy_";
constexpr const char* kTinkertoyTargetFrameName = "tinkertoy_target";
constexpr const char* kTinkertoyForceLineFrameName = "tinkertoy_force_line";
constexpr const char* kTinkertoyAxisFramePrefix = "tinkertoy_axis_";

struct TinkertoyShapes
{
  dart::dynamics::ShapePtr weldJointShape;
  dart::dynamics::ShapePtr revoluteJointShape;
  dart::dynamics::ShapePtr ballJointShape;
  std::shared_ptr<dart::dynamics::BoxShape> blockShape;
  Eigen::Isometry3d blockOffset = Eigen::Isometry3d::Identity();
};

TinkertoyShapes createTinkertoyShapes()
{
  constexpr double blockLength = 0.5;
  constexpr double blockWidth = 0.075;
  constexpr double jointRadius = 1.5 * blockWidth / 2.0;

  TinkertoyShapes shapes;
  shapes.weldJointShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(2.0 * jointRadius, blockWidth, blockWidth));
  shapes.revoluteJointShape = std::make_shared<dart::dynamics::CylinderShape>(
      jointRadius, 1.5 * blockWidth);
  shapes.ballJointShape
      = std::make_shared<dart::dynamics::SphereShape>(jointRadius);
  shapes.blockShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(blockLength, blockWidth, blockWidth));
  shapes.blockOffset.translation().x() = blockLength / 2.0;
  return shapes;
}

template <class JointType>
dart::dynamics::BodyNode* addTinkertoyBlock(
    dart::simulation::World& world,
    const TinkertoyShapes& shapes,
    dart::dynamics::BodyNode* parent,
    const Eigen::Isometry3d& relativeTransform,
    const dart::dynamics::ShapePtr& jointShape,
    std::size_t& nextToyIndex)
{
  constexpr double blockMass = 0.16 * 10e3 * 0.5 * 0.075 * 0.075;
  const Eigen::Vector4d blockColor(0xEE / 255.0, 0xC9 / 255.0, 0.0, 1.0);
  const Eigen::Vector4d jointColor(0.50, 0.50, 1.0, 1.0);

  dart::dynamics::SkeletonPtr skeleton;
  if (parent != nullptr) {
    skeleton = parent->getSkeleton();
  } else {
    skeleton = dart::dynamics::Skeleton::create(
        std::string(kTinkertoySkeletonPrefix) + std::to_string(nextToyIndex++));
    world.addSkeleton(skeleton);
  }

  auto [joint, body] = skeleton->createJointAndBodyNodePair<JointType>(parent);
  body->setName("block_" + std::to_string(skeleton->getNumBodyNodes()));
  joint->setName("joint_" + std::to_string(skeleton->getNumJoints()));
  joint->setTransformFromParentBodyNode(relativeTransform);
  if constexpr (std::is_same_v<JointType, dart::dynamics::RevoluteJoint>) {
    joint->setAxis(Eigen::Vector3d::UnitZ());
  }
  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    joint->setDampingCoefficient(i, 0.4);
  }

  auto* jointShapeNode
      = body->template createShapeNodeWith<dart::dynamics::VisualAspect>(
          jointShape);
  jointShapeNode->getVisualAspect()->setRGBA(jointColor);

  auto* blockShapeNode
      = body->template createShapeNodeWith<dart::dynamics::VisualAspect>(
          shapes.blockShape);
  blockShapeNode->setRelativeTransform(shapes.blockOffset);
  blockShapeNode->getVisualAspect()->setRGBA(blockColor);

  body->setInertia(
      dart::dynamics::Inertia(
          blockMass,
          0.25 * Eigen::Vector3d::UnitX(),
          shapes.blockShape->computeInertia(blockMass)));

  return body;
}

void addTinkertoyAxis(
    dart::simulation::World& world,
    const std::string& name,
    const Eigen::Vector3d& axis,
    const Eigen::Vector3d& color)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(2.5f);
  line->addVertex(Eigen::Vector3d::Zero());
  line->addVertex(axis * 0.55);
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->getVisualAspect(true)->setColor(color);
  world.addSimpleFrame(frame);
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createTargetHandleShape(
    double radius)
{
  auto handle = std::make_shared<dart::dynamics::LineSegmentShape>(7.0f);
  const std::size_t center = handle->addVertex(Eigen::Vector3d::Zero());
  const auto addAxis = [&](const Eigen::Vector3d& axis) {
    handle->addVertex(axis, center);
    handle->addVertex(-axis, center);
  };
  addAxis(Eigen::Vector3d(radius, 0.0, 0.0));
  addAxis(Eigen::Vector3d(0.0, radius, 0.0));
  addAxis(Eigen::Vector3d(0.0, 0.0, 0.75 * radius));
  return handle;
}

void addTinkertoyReferenceFrames(dart::simulation::World& world)
{
  addTinkertoyAxis(
      world,
      std::string(kTinkertoyAxisFramePrefix) + "x",
      Eigen::Vector3d::UnitX(),
      Eigen::Vector3d(0.9, 0.0, 0.0));
  addTinkertoyAxis(
      world,
      std::string(kTinkertoyAxisFramePrefix) + "y",
      Eigen::Vector3d::UnitY(),
      Eigen::Vector3d(0.0, 0.8, 0.0));
  addTinkertoyAxis(
      world,
      std::string(kTinkertoyAxisFramePrefix) + "z",
      Eigen::Vector3d::UnitZ(),
      Eigen::Vector3d(0.0, 0.0, 0.9));

  Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  targetTransform.translation() = Eigen::Vector3d(0.35, -0.55, 0.35);
  auto target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      kTinkertoyTargetFrameName,
      targetTransform);
  target->setShape(createTargetHandleShape(0.15));
  target->getVisualAspect(true)->setRGBA(Eigen::Vector4d(1.0, 0.0, 1.0, 1.0));
  world.addSimpleFrame(target);

  auto forceLine = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kTinkertoyForceLineFrameName);
  auto forceLineShape = std::make_shared<dart::dynamics::LineSegmentShape>(
      Eigen::Vector3d(0.08, -0.15, 0.18), targetTransform.translation(), 3.0f);
  forceLine->setShape(forceLineShape);
  forceLine->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(1.0, 0.63, 0.0, 1.0));
  world.addSimpleFrame(forceLine);
}

void addTinkertoyInitialAssemblies(dart::simulation::World& world)
{
  TinkertoyShapes shapes = createTinkertoyShapes();
  std::size_t nextToyIndex = 0;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(45.0), Eigen::Vector3d::UnitY()));
  auto* firstToy = addTinkertoyBlock<dart::dynamics::BallJoint>(
      world, shapes, nullptr, transform, shapes.ballJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.5;
  transform.prerotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitX()));
  firstToy = addTinkertoyBlock<dart::dynamics::RevoluteJoint>(
      world,
      shapes,
      firstToy,
      transform,
      shapes.revoluteJointShape,
      nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitZ()));
  firstToy = addTinkertoyBlock<dart::dynamics::WeldJoint>(
      world, shapes, firstToy, transform, shapes.weldJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.25;
  transform.translation().z() = 0.075;
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-30.0), Eigen::Vector3d::UnitZ()));
  addTinkertoyBlock<dart::dynamics::BallJoint>(
      world, shapes, firstToy, transform, shapes.ballJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitY()));
  transform.pretranslate(-1.0 * Eigen::Vector3d::UnitX());
  auto* secondToy = addTinkertoyBlock<dart::dynamics::BallJoint>(
      world, shapes, nullptr, transform, shapes.ballJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.5;
  transform.translation().z() = 0.25;
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitY()));
  secondToy = addTinkertoyBlock<dart::dynamics::WeldJoint>(
      world, shapes, secondToy, transform, shapes.weldJointShape, nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-90.0), Eigen::Vector3d::UnitX()));
  transform.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-90.0), Eigen::Vector3d::UnitZ()));
  transform.translation().z() = 0.075 / 2.0;
  addTinkertoyBlock<dart::dynamics::RevoluteJoint>(
      world,
      shapes,
      secondToy,
      transform,
      shapes.revoluteJointShape,
      nextToyIndex);

  transform.translation().x() = 0.5;
  secondToy = addTinkertoyBlock<dart::dynamics::RevoluteJoint>(
      world,
      shapes,
      secondToy,
      transform,
      shapes.revoluteJointShape,
      nextToyIndex);

  transform = Eigen::Isometry3d::Identity();
  transform.translation().x() = 0.5;
  addTinkertoyBlock<dart::dynamics::BallJoint>(
      world, shapes, secondToy, transform, shapes.ballJointShape, nextToyIndex);
}

dart::simulation::WorldPtr createTinkertoyWorld()
{
  auto world = dart::simulation::World::create("tinkertoy");
  world->setGravity(Eigen::Vector3d::Zero());
  addTinkertoyReferenceFrames(*world);
  addTinkertoyInitialAssemblies(*world);
  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  bool showBuilderHints = true;

  dart::gui::Panel controls;
  controls.title = "Tinkertoy Controls";
  controls.buildWithContext
      = [&](dart::gui::PanelBuilder& panel, dart::gui::PanelContext& context) {
          panel.text("Interactive tinkertoy fixture");
          panel.separator();
          if (context.lifecycle != nullptr) {
            if (panel.button(context.lifecycle->paused ? "Resume" : "Pause")) {
              dart::gui::togglePaused(*context.lifecycle);
            }
            panel.sameLine();
            if (panel.button("Step")) {
              dart::gui::requestSingleStep(*context.lifecycle);
            }
          }
          panel.checkbox("Show builder hints", showBuilderHints);
          if (showBuilderHints) {
            panel.text("Select the magenta target handle.");
            panel.text("Ctrl-left drag moves the selected handle.");
            panel.text("Arrow keys and PageUp/PageDown nudge it.");
            panel.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
          }
          panel.text("time: " + std::to_string(context.simulationTime));
          panel.text("contacts: " + std::to_string(context.contactCount));
          panel.text("selected: " + context.selectedLabel);
        };

  dart::gui::ApplicationOptions options;
  options.world = createTinkertoyWorld();
  options.panels.push_back(std::move(controls));

  return dart::gui::runApplication(argc, argv, options);
}
