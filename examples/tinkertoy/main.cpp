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

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/collision_group.hpp>

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

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include <cstddef>

namespace {

constexpr const char* kTinkertoySkeletonPrefix = "visual_tinkertoy_toy_";
constexpr const char* kTinkertoyTargetFrameName = "tinkertoy_target";
constexpr const char* kTinkertoyForceLineFrameName = "tinkertoy_force_line";
constexpr const char* kTinkertoyAxisFramePrefix = "tinkertoy_axis_";
constexpr double kDefaultBlockLength = 0.5;
constexpr double kDefaultBlockWidth = 0.075;
constexpr double kDefaultJointRadius = 1.5 * kDefaultBlockWidth / 2.0;
constexpr double kBalsaWoodDensity = 0.16 * 10e3;
constexpr double kDefaultBlockMass = kBalsaWoodDensity * kDefaultBlockLength
                                     * kDefaultBlockWidth * kDefaultBlockWidth;
constexpr double kDefaultDamping = 0.4;
constexpr double kMaxForce = 200.0;
constexpr double kDefaultForceCoeff = 100.0;
constexpr double kMaxForceCoeff = 1000.0;
constexpr double kMinForceCoeff = 10.0;
constexpr double kForceIncrement = 10.0;

const Eigen::Vector4d kSimulationColor(0.5, 0.5, 1.0, 1.0);
const Eigen::Vector4d kPausedColor(0xEE / 255.0, 0xC9 / 255.0, 0.0, 1.0);
const Eigen::Vector4d kSelectedColor(1.0, 0.0, 0.0, 1.0);
const Eigen::Vector4d kForceBodyColor(1.0, 0.0, 1.0, 1.0);
const Eigen::Vector4d kForceLineColor(1.0, 0.63, 0.0, 1.0);

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
  TinkertoyShapes shapes;
  shapes.weldJointShape
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
          2.0 * kDefaultJointRadius, kDefaultBlockWidth, kDefaultBlockWidth));
  shapes.revoluteJointShape = std::make_shared<dart::dynamics::CylinderShape>(
      kDefaultJointRadius, 1.5 * kDefaultBlockWidth);
  shapes.ballJointShape
      = std::make_shared<dart::dynamics::SphereShape>(kDefaultJointRadius);
  shapes.blockShape
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
          kDefaultBlockLength, kDefaultBlockWidth, kDefaultBlockWidth));
  shapes.blockOffset.translation().x() = kDefaultBlockLength / 2.0;
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
    joint->setDampingCoefficient(i, kDefaultDamping);
  }

  auto* jointShapeNode = body->template createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(jointShape);
  jointShapeNode->getVisualAspect()->setRGBA(kPausedColor);

  auto* blockShapeNode = body->template createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shapes.blockShape);
  blockShapeNode->setRelativeTransform(shapes.blockOffset);
  blockShapeNode->getVisualAspect()->setRGBA(kPausedColor);

  body->setInertia(
      dart::dynamics::Inertia(
          kDefaultBlockMass,
          0.25 * Eigen::Vector3d::UnitX(),
          shapes.blockShape->computeInertia(kDefaultBlockMass)));

  if (auto* solver = world.getConstraintSolver()) {
    solver->getCollisionGroup()->addShapeFramesOf(body);
  }

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

struct TinkertoyReferenceFrames
{
  dart::dynamics::SimpleFramePtr target;
  std::shared_ptr<dart::dynamics::LineSegmentShape> forceLine;
};

TinkertoyReferenceFrames addTinkertoyReferenceFrames(
    dart::simulation::World& world)
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
  forceLine->getVisualAspect(true)->setRGBA(kForceLineColor);
  world.addSimpleFrame(forceLine);
  return {target, forceLineShape};
}

void addTinkertoyInitialAssemblies(
    dart::simulation::World& world,
    const TinkertoyShapes& shapes,
    std::size_t& nextToyIndex)
{
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
  transform.translation().z() = kDefaultBlockWidth / 2.0;
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

class TinkertoyState
{
public:
  TinkertoyState()
    : mWorld(dart::simulation::World::create("tinkertoy")),
      mShapes(createTinkertoyShapes())
  {
    setGravityEnabled(true);
    auto references = addTinkertoyReferenceFrames(*mWorld);
    mTarget = std::move(references.target);
    mForceLine = std::move(references.forceLine);
    addTinkertoyInitialAssemblies(*mWorld, mShapes, mNextToyIndex);
    updateVisualState(false);
  }

  const dart::simulation::WorldPtr& world() const
  {
    return mWorld;
  }

  void syncPickFromSelectionContext(
      const std::string& selectedLabel,
      const std::optional<Eigen::Vector3d>& selectedPoint,
      const std::optional<Eigen::Vector3d>& selectedNormal)
  {
    const bool samePoint
        = (!selectedPoint.has_value() && !mLastSelectedPoint.has_value())
          || (selectedPoint.has_value() && mLastSelectedPoint.has_value()
              && selectedPoint->isApprox(*mLastSelectedPoint));
    if (selectedLabel == mLastSelectedLabel && samePoint) {
      return;
    }

    mLastSelectedLabel = selectedLabel;
    mLastSelectedPoint = selectedPoint;
    auto* selectedBody = findSelectedBody(selectedLabel);
    if (selectedBody == nullptr) {
      return;
    }

    setPickedNode(selectedBody, selectedPoint, selectedNormal);
  }

  void preStep()
  {
    applyPickedForce();
    updateForceLine();
  }

  void updateVisualState(bool simulating)
  {
    mWorld->eachSkeleton([&](dart::dynamics::Skeleton* skeleton) {
      if (skeleton != nullptr
          && skeleton->getName().starts_with(kTinkertoySkeletonPrefix)) {
        skeleton->setColor(simulating ? kSimulationColor : kPausedColor);
      }
    });

    if (mPickedNode != nullptr) {
      mPickedNode->setColor(simulating ? kForceBodyColor : kSelectedColor);
    }

    updateForceLine();
  }

  void setGravityEnabled(bool enabled)
  {
    mGravityEnabled = enabled;
    if (mGravityEnabled) {
      mWorld->setGravity(-9.81 * Eigen::Vector3d::UnitZ());
    } else {
      mWorld->setGravity(Eigen::Vector3d::Zero());
    }
  }

  bool gravityEnabled() const
  {
    return mGravityEnabled;
  }

  void setForceCoeff(double coeff)
  {
    mForceCoeff = std::clamp(coeff, kMinForceCoeff, kMaxForceCoeff);
  }

  double forceCoeff() const
  {
    return mForceCoeff;
  }

  void incrementForceCoeff()
  {
    setForceCoeff(mForceCoeff + kForceIncrement);
    mStatus = "Force coefficient: " + std::to_string(mForceCoeff);
    std::cout << "[Force Coefficient: " << mForceCoeff << "]" << std::endl;
  }

  void decrementForceCoeff()
  {
    setForceCoeff(mForceCoeff - kForceIncrement);
    mStatus = "Force coefficient: " + std::to_string(mForceCoeff);
    std::cout << "[Force Coefficient: " << mForceCoeff << "]" << std::endl;
  }

  const std::string& status() const
  {
    return mStatus;
  }

  void clearPick()
  {
    mPickedNode = nullptr;
    mPickedPoint = Eigen::Vector3d::Zero();
    if (mTarget != nullptr) {
      mTarget->setTransform(Eigen::Isometry3d::Identity());
    }
    mStatus = "No block selected";
    updateForceLine();
  }

  void deletePick(bool paused)
  {
    if (!canEdit(paused, "delete blocks") || mPickedNode == nullptr) {
      return;
    }

    auto removedSubtree = mPickedNode->remove();
    if (auto* solver = mWorld->getConstraintSolver();
        solver != nullptr && removedSubtree != nullptr) {
      solver->getCollisionGroup()->removeShapeFramesOf(removedSubtree.get());
    }
    clearPick();
    mStatus = "Deleted selected block subtree";
  }

  void reorientTarget()
  {
    if (mTarget == nullptr) {
      return;
    }

    Eigen::Isometry3d transform = mTarget->getWorldTransform();
    transform.linear() = Eigen::Matrix3d::Identity();
    mTarget->setTransform(transform);
    mStatus = "Target orientation reset";
  }

  void addWeldJointBlock(bool paused)
  {
    addBlock<dart::dynamics::WeldJoint>(
        paused, mShapes.weldJointShape, "Added WeldJoint block");
  }

  void addRevoluteJointBlock(bool paused)
  {
    addBlock<dart::dynamics::RevoluteJoint>(
        paused, mShapes.revoluteJointShape, "Added RevoluteJoint block");
  }

  void addBallJointBlock(bool paused)
  {
    addBlock<dart::dynamics::BallJoint>(
        paused, mShapes.ballJointShape, "Added BallJoint block");
  }

private:
  dart::dynamics::BodyNode* findSelectedBody(
      const std::string& selectedLabel) const
  {
    const auto firstSlash = selectedLabel.find('/');
    if (firstSlash == std::string::npos) {
      return nullptr;
    }

    const std::string skeletonName = selectedLabel.substr(0, firstSlash);
    if (!skeletonName.starts_with(kTinkertoySkeletonPrefix)) {
      return nullptr;
    }

    const auto secondSlash = selectedLabel.find('/', firstSlash + 1);
    const auto shapeSuffix = selectedLabel.find(" (", firstSlash + 1);
    const auto bodyEnd
        = secondSlash == std::string::npos ? shapeSuffix : secondSlash;
    if (bodyEnd == std::string::npos || bodyEnd <= firstSlash + 1) {
      return nullptr;
    }

    const std::string bodyName
        = selectedLabel.substr(firstSlash + 1, bodyEnd - firstSlash - 1);
    const auto skeleton = mWorld->getSkeleton(skeletonName);
    return skeleton == nullptr ? nullptr : skeleton->getBodyNode(bodyName);
  }

  void setPickedNode(
      dart::dynamics::BodyNode* body,
      const std::optional<Eigen::Vector3d>& selectedPoint = std::nullopt,
      const std::optional<Eigen::Vector3d>& selectedNormal = std::nullopt)
  {
    if (body == nullptr) {
      return;
    }

    mPickedNode = body;
    if (selectedPoint.has_value()) {
      mPickedPoint = body->getWorldTransform().inverse() * *selectedPoint;
    } else {
      mPickedPoint = body->getLocalCOM();
    }

    if (mTarget != nullptr) {
      Eigen::Isometry3d transform = body->getWorldTransform();
      if (selectedPoint.has_value()) {
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        if (selectedNormal.has_value() && selectedNormal->allFinite()
            && selectedNormal->norm() > 1e-12) {
          normal = selectedNormal->normalized();
        }
        transform.translation()
            = *selectedPoint + normal * (kDefaultBlockWidth / 2.0);
      } else {
        transform.translation() = body->getCOM();
      }
      mTarget->setTransform(transform);
    }

    mStatus
        = "Picked " + body->getSkeleton()->getName() + "/" + body->getName();
    updateForceLine();
  }

  bool canEdit(bool paused, const char* action)
  {
    if (!paused) {
      mStatus = std::string("Pause simulation before attempting to ") + action;
      std::cout << " -- Please pause simulation [using the Spacebar] before "
                << "attempting to " << action << "." << std::endl;
      return false;
    }
    return true;
  }

  Eigen::Isometry3d relativeTargetTransform() const
  {
    if (mTarget == nullptr) {
      return Eigen::Isometry3d::Identity();
    }
    return mPickedNode != nullptr ? mTarget->getTransform(mPickedNode)
                                  : mTarget->getWorldTransform();
  }

  template <class JointType>
  void addBlock(
      bool paused,
      const dart::dynamics::ShapePtr& jointShape,
      const char* status)
  {
    if (!canEdit(paused, "add new bodies")) {
      return;
    }

    auto* body = addTinkertoyBlock<JointType>(
        *mWorld,
        mShapes,
        mPickedNode,
        relativeTargetTransform(),
        jointShape,
        mNextToyIndex);
    if (body == nullptr) {
      return;
    }

    setPickedNode(body);
    mStatus = status;
    updateVisualState(false);
  }

  void updateForceLine()
  {
    if (mForceLine == nullptr) {
      return;
    }

    if (mPickedNode == nullptr || mTarget == nullptr) {
      Eigen::Vector3d target = Eigen::Vector3d::Zero();
      if (mTarget != nullptr) {
        target = mTarget->getWorldTransform().translation();
      }
      mForceLine->setVertex(0, target + Eigen::Vector3d(0.08, -0.15, 0.18));
      mForceLine->setVertex(1, target);
      return;
    }

    mForceLine->setVertex(0, mPickedNode->getWorldTransform() * mPickedPoint);
    mForceLine->setVertex(1, mTarget->getWorldTransform().translation());
  }

  void applyPickedForce()
  {
    if (mPickedNode == nullptr || mTarget == nullptr) {
      return;
    }

    Eigen::Vector3d force
        = mForceCoeff
          * (mTarget->getWorldTransform().translation()
             - mPickedNode->getWorldTransform() * mPickedPoint);
    const double forceNorm = force.norm();
    if (forceNorm > kMaxForce) {
      force = kMaxForce * force / forceNorm;
    }
    mPickedNode->addExtForce(force, mPickedPoint);
  }

  dart::simulation::WorldPtr mWorld;
  TinkertoyShapes mShapes;
  std::size_t mNextToyIndex = 0;
  dart::dynamics::SimpleFramePtr mTarget;
  std::shared_ptr<dart::dynamics::LineSegmentShape> mForceLine;
  dart::dynamics::BodyNode* mPickedNode = nullptr;
  Eigen::Vector3d mPickedPoint = Eigen::Vector3d::Zero();
  double mForceCoeff = kDefaultForceCoeff;
  bool mGravityEnabled = true;
  std::string mLastSelectedLabel;
  std::optional<Eigen::Vector3d> mLastSelectedPoint;
  std::string mStatus = "No block selected";
};

std::vector<dart::gui::KeyboardAction> createTinkertoyKeyboardActions(
    const std::shared_ptr<TinkertoyState>& state)
{
  const auto paused = [](dart::gui::KeyboardActionContext& context) {
    return context.lifecycle == nullptr || context.lifecycle->paused;
  };

  std::vector<dart::gui::KeyboardAction> actions;
  const auto addAction = [&](std::string label,
                             dart::gui::KeyboardShortcut shortcut,
                             auto callback) {
    dart::gui::KeyboardAction action;
    action.label = std::move(label);
    action.shortcut = shortcut;
    action.callback = callback;
    actions.push_back(std::move(action));
  };

  addAction(
      "Add WeldJoint block",
      dart::gui::KeyboardShortcut::characterKey('1'),
      [state, paused](dart::gui::KeyboardActionContext& context) {
        state->addWeldJointBlock(paused(context));
      });
  addAction(
      "Add RevoluteJoint block",
      dart::gui::KeyboardShortcut::characterKey('2'),
      [state, paused](dart::gui::KeyboardActionContext& context) {
        state->addRevoluteJointBlock(paused(context));
      });
  addAction(
      "Add BallJoint block",
      dart::gui::KeyboardShortcut::characterKey('3'),
      [state, paused](dart::gui::KeyboardActionContext& context) {
        state->addBallJointBlock(paused(context));
      });
  addAction(
      "Clear Tinkertoy pick",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Backspace),
      [state](dart::gui::KeyboardActionContext&) { state->clearPick(); });
  addAction(
      "Delete Tinkertoy pick",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Delete),
      [state, paused](dart::gui::KeyboardActionContext& context) {
        state->deletePick(paused(context));
      });
  addAction(
      "Increase Tinkertoy force coefficient",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Up),
      [state](dart::gui::KeyboardActionContext&) {
        state->incrementForceCoeff();
      });
  addAction(
      "Decrease Tinkertoy force coefficient",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Down),
      [state](dart::gui::KeyboardActionContext&) {
        state->decrementForceCoeff();
      });
  addAction(
      "Reorient Tinkertoy target",
      dart::gui::KeyboardShortcut::namedKey(
          dart::gui::KeyboardKey::GraveAccent),
      [state](dart::gui::KeyboardActionContext&) { state->reorientTarget(); });
  addAction(
      "Reset Tinkertoy camera",
      dart::gui::KeyboardShortcut::namedKey(dart::gui::KeyboardKey::Tab),
      [](dart::gui::KeyboardActionContext& context) {
        if (context.resetCamera) {
          context.resetCamera();
        }
      });

  return actions;
}

dart::gui::RunOptions makeTinkertoyRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 720;
  return options;
}

} // namespace

int main(int argc, char* argv[])
{
  auto state = std::make_shared<TinkertoyState>();

  dart::gui::Panel controls;
  controls.title = "Tinkertoy Control";
  controls.initialPosition = std::array<double, 2>{10.0, 20.0};
  controls.initialSize = std::array<double, 2>{360.0, 640.0};
  controls.backgroundAlpha = 0.5;
  controls.autoResize = false;
  controls.horizontalScrollbar = true;
  controls.menuBar = true;

  auto showAbout = std::make_shared<bool>(false);
  controls.buildWithContext = [state, showAbout](
                                  dart::gui::PanelBuilder& panel,
                                  dart::gui::PanelContext& context) mutable {
    state->syncPickFromSelectionContext(
        context.selectedLabel, context.selectedPoint, context.selectedNormal);
    const bool paused
        = context.lifecycle == nullptr || context.lifecycle->paused;
    state->updateVisualState(!paused);

    if (panel.beginMenuBar()) {
      if (panel.beginMenu("Menu")) {
        if (panel.menuItem("Exit") && context.lifecycle != nullptr) {
          dart::gui::requestExit(*context.lifecycle);
        }
        panel.endMenu();
      }
      if (panel.beginMenu("Help")) {
        if (panel.menuItem("About DART")) {
          *showAbout = true;
        }
        panel.endMenu();
      }
      panel.endMenuBar();
    }

    panel.text("Interactive tinkertoy fixture");

    if (panel.collapsingHeader("Help")) {
      panel.text("User Guide:");
      panel.text("Left-click on a block to select it.");
      panel.text("Press [Backspace] to deselect.");
      panel.text("Press [Tab] to reset the camera view.");
      panel.text("Press [`] to reset the orientation of the target.");
      panel.text("--- While Simulation is Paused ---");
      panel.text("The selected block will be red; all other blocks yellow.");
      panel.text("Press [1] -> [3] to attach a block to the selected block.");
      panel.text("[1]: Attach using a WeldJoint.");
      panel.text("[2]: Attach using a RevoluteJoint.");
      panel.text("[3]: Attach using a BallJoint.");
      panel.text("The new block follows the target's red x-axis.");
      panel.text("RevoluteJoint axes follow the target's blue z-axis.");
      panel.text("Press [Delete] to remove a block and all its children.");
      panel.text("With no selected block, adding starts a new tree.");
      panel.text("--- While Simulation is Active ---");
      panel.text("The selected block will be fuchsia; all others blue.");
      panel.text("Move the target to pull on the selected block.");
      panel.text("Press [Up] or [Down] to adjust pulling strength.");
      panel.text("Blocks in different trees can collide.");
      panel.text("Blocks in the same tree ignore each other.");
      panel.text("Ctrl-left drag, arrows, and PageUp/PageDown move targets.");
      panel.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    }

    if (panel.collapsingHeader("Simulation", true)
        && context.lifecycle != nullptr) {
      if (panel.button("Play")) {
        context.lifecycle->paused = false;
      }
      panel.sameLine();
      if (panel.button("Pause")) {
        context.lifecycle->paused = true;
      }
      panel.sameLine();
      if (panel.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }

    if (panel.collapsingHeader("World Options", true)) {
      bool gravityEnabled = state->gravityEnabled();
      if (panel.checkbox("Gravity On/Off", gravityEnabled)) {
        state->setGravityEnabled(gravityEnabled);
      }
    }

    if (panel.collapsingHeader("Tinkertoy Options", true)) {
      double forceCoeff = state->forceCoeff();
      if (panel.slider(
              "Force Coeff", forceCoeff, kMinForceCoeff, kMaxForceCoeff)) {
        state->setForceCoeff(forceCoeff);
      }

      if (panel.button("Reorient Target")) {
        state->reorientTarget();
      }
      panel.sameLine();
      if (panel.button("Reset Target")) {
        state->clearPick();
      }

      if (panel.button("Add a Weld-Joint Block")) {
        state->addWeldJointBlock(paused);
      }
      if (panel.button("Add a Revolute-Joint Block")) {
        state->addRevoluteJointBlock(paused);
      }
      if (panel.button("Add a Ball-Joint Block")) {
        state->addBallJointBlock(paused);
      }
      if (panel.button("Delete Block")) {
        state->deletePick(paused);
      }
    }

    if (*showAbout) {
      panel.separator();
      panel.text("About DART: project and libdart simulation libraries.");
    }
    panel.separator();
    panel.text("time: " + std::to_string(context.simulationTime));
    panel.text("contacts: " + std::to_string(context.contactCount));
    panel.text("selected: " + context.selectedLabel);
    panel.text("builder: " + state->status());
  };

  dart::gui::ApplicationOptions options;
  options.world = state->world();
  options.runDefaults = makeTinkertoyRunDefaults();
  options.preStep = [state]() {
    state->preStep();
  };
  options.keyboardActions = createTinkertoyKeyboardActions(state);
  options.panels.push_back(std::move(controls));

  return dart::gui::runApplication(argc, argv, options);
}
