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
#include <dart/gui/debug.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/balance_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/geometry.hpp>
#include <dart/math/optimization/function.hpp>
#include <dart/math/optimization/gradient_descent_solver.hpp>
#include <dart/math/optimization/problem.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace {

constexpr const char* kAtlasUri
    = "dart://sample/sdf/atlas/atlas_v3_no_head.urdf";
constexpr const char* kAtlasSkeletonName = "visual_atlas_robot";
constexpr const char* kGroundSkeletonName = "visual_atlas_puppet_ground";
constexpr const char* kAtlasSupportOverlayName
    = "atlas_puppet_support_polygon_overlay";
constexpr const char* kAtlasSupportComOverlayName
    = "atlas_puppet_support_com_overlay";
constexpr double kTeleopLinearStep = 0.01;
constexpr double kTeleopElevationStep = 0.2 * kTeleopLinearStep;
constexpr double kTeleopYawStep = 2.0 * 3.14159265358979323846 / 180.0;
constexpr double kSupportVisualElevation = 0.05;
constexpr double kSupportComMarkerRadius = 0.06;

enum class PuppetMotion
{
  Forward,
  Backward,
  Left,
  Right,
  Up,
  Down,
  YawLeft,
  YawRight,
};

class RelaxedPosture final : public dart::math::Function
{
public:
  RelaxedPosture(
      const Eigen::VectorXd& idealPosture,
      const Eigen::VectorXd& lower,
      const Eigen::VectorXd& upper,
      const Eigen::VectorXd& weights,
      bool enforceIdeal = false)
    : enforceIdealPosture(enforceIdeal),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    const Eigen::Index dofCount = mIdeal.size();
    if (mLower.size() != dofCount || mUpper.size() != dofCount
        || mWeights.size() != dofCount) {
      throw std::runtime_error(
          "Atlas relaxed-posture objective has mismatched vector sizes");
    }
    mResultVector.setZero(dofCount);
  }

  double eval(const Eigen::VectorXd& x) override
  {
    computeResultVector(x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    computeResultVector(x);

    grad.setZero();
    const Eigen::Index count = std::min(mResultVector.size(), grad.size());
    for (Eigen::Index i = 0; i < count; ++i) {
      grad[i] = mResultVector[i];
    }
  }

  void computeResultVector(const Eigen::VectorXd& x)
  {
    mResultVector.setZero();
    const Eigen::Index count = std::min(mIdeal.size(), x.size());
    for (Eigen::Index i = 0; i < count; ++i) {
      if (enforceIdealPosture) {
        mResultVector[i] = mWeights[i] * (x[i] - mIdeal[i]);
      } else if (x[i] < mLower[i]) {
        mResultVector[i] = mWeights[i] * (x[i] - mLower[i]);
      } else if (mUpper[i] < x[i]) {
        mResultVector[i] = mWeights[i] * (x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

private:
  Eigen::VectorXd mResultVector;
  Eigen::VectorXd mIdeal;
  Eigen::VectorXd mLower;
  Eigen::VectorXd mUpper;
  Eigen::VectorXd mWeights;
};

struct AtlasWholeBodySolverState
{
  std::shared_ptr<RelaxedPosture> posture;
  std::shared_ptr<dart::constraint::BalanceConstraint> balance;
};

void disableSkeletonCollisionAndGravity(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
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
    body->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
        [](dart::dynamics::ShapeNode* shapeNode) {
          shapeNode->getCollisionAspect()->setCollidable(false);
        });
  }
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
computeVisualWorldBounds(const dart::dynamics::SkeletonPtr& skeleton)
{
  if (skeleton == nullptr) {
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

    body->eachShapeNodeWith<dart::dynamics::VisualAspect>(
        [&](const dart::dynamics::ShapeNode* shapeNode) {
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
                includePoint(
                    transform
                    * Eigen::Vector3d(
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

void setRequiredDofPosition(
    const dart::dynamics::SkeletonPtr& skeleton,
    const char* name,
    double position)
{
  auto* dof = skeleton ? skeleton->getDof(name) : nullptr;
  if (dof == nullptr) {
    throw std::runtime_error(
        "Atlas puppet model is missing expected DOF " + std::string(name));
  }
  dof->setPosition(position);
}

void setupAtlasPuppetStartConfiguration(
    const dart::dynamics::SkeletonPtr& atlas)
{
  constexpr double degrees = 3.14159265358979323846 / 180.0;

  setRequiredDofPosition(atlas, "r_leg_hpy", -45.0 * degrees);
  setRequiredDofPosition(atlas, "r_leg_kny", 90.0 * degrees);
  setRequiredDofPosition(atlas, "r_leg_aky", -45.0 * degrees);
  setRequiredDofPosition(atlas, "l_leg_hpy", -45.0 * degrees);
  setRequiredDofPosition(atlas, "l_leg_kny", 90.0 * degrees);
  setRequiredDofPosition(atlas, "l_leg_aky", -45.0 * degrees);

  setRequiredDofPosition(atlas, "r_arm_shx", 65.0 * degrees);
  setRequiredDofPosition(atlas, "r_arm_ely", 90.0 * degrees);
  setRequiredDofPosition(atlas, "r_arm_elx", -90.0 * degrees);
  setRequiredDofPosition(atlas, "r_arm_wry", 65.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_shx", -65.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_ely", 90.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_elx", 90.0 * degrees);
  setRequiredDofPosition(atlas, "l_arm_wry", 65.0 * degrees);

  atlas->getDof("r_leg_kny")->setPositionLowerLimit(10.0 * degrees);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit(10.0 * degrees);
}

dart::dynamics::SkeletonPtr loadAtlasPuppetSkeleton()
{
  const auto atlasUri = dart::common::Uri::createFromString(kAtlasUri);
  auto atlas = dart::io::readSkeleton(atlasUri);
  if (atlas == nullptr) {
    throw std::runtime_error(
        "Failed to load Atlas puppet model from " + atlasUri.toString());
  }

  atlas->setName(kAtlasSkeletonName);
  setupAtlasPuppetStartConfiguration(atlas);

  auto* rootBody = atlas->getRootBodyNode();
  if (rootBody != nullptr
      && dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             != nullptr) {
    Eigen::Isometry3d transform = rootBody->getWorldTransform();
    transform.translation().x() = 0.0;
    transform.translation().y() = 0.0;
    dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    if (const auto bounds = computeVisualWorldBounds(atlas)) {
      constexpr double groundClearance = 0.015;
      transform = rootBody->getWorldTransform();
      transform.translation().z() += groundClearance - bounds->first.z();
      dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
    }
  }

  auto* torso = atlas->getRootBodyNode();
  if (torso != nullptr) {
    auto rootShape = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(0.25, 0.25, 0.125));
    auto* shapeNode
        = torso->createShapeNodeWith<dart::dynamics::VisualAspect>(rootShape);
    shapeNode->setName("atlas_puppet_root_handle");
    shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.1));
    shapeNode->getVisualAspect()->setRGBA(
        Eigen::Vector4d(0.08, 0.09, 0.10, 1.0));
  }

  disableSkeletonCollisionAndGravity(atlas);
  return atlas;
}

dart::math::SupportGeometry makeAtlasPuppetFootSupportGeometry()
{
  dart::math::SupportGeometry support;
  constexpr double supportPositiveX = 0.10 - 0.186;
  constexpr double supportNegativeX = -0.03 - 0.186;
  constexpr double supportPositiveY = 0.03;
  constexpr double supportNegativeY = -0.03;
  support.emplace_back(supportNegativeX, supportNegativeY, 0.0);
  support.emplace_back(supportPositiveX, supportNegativeY, 0.0);
  support.emplace_back(supportPositiveX, supportPositiveY, 0.0);
  support.emplace_back(supportNegativeX, supportPositiveY, 0.0);
  return support;
}

void addDisconnectedLine(
    dart::dynamics::LineSegmentShape& shape,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to)
{
  const auto start = shape.addVertex(from);
  if (start > 0u) {
    shape.removeConnection(start - 1u, start);
  }
  shape.addVertex(to, start);
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createLineShape(
    const std::vector<dart::gui::DebugLineDescriptor>& lines)
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(2.0f);
  for (const auto& line : lines) {
    addDisconnectedLine(*shape, line.from, line.to);
  }
  return shape;
}

void appendMarkerAxisLines(
    std::vector<dart::gui::DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const std::string& label)
{
  dart::gui::DebugLineDescriptor x;
  x.from = center - Eigen::Vector3d::UnitX() * radius;
  x.to = center + Eigen::Vector3d::UnitX() * radius;
  x.label = label + ".x";
  lines.push_back(x);

  dart::gui::DebugLineDescriptor y;
  y.from = center - Eigen::Vector3d::UnitY() * radius;
  y.to = center + Eigen::Vector3d::UnitY() * radius;
  y.label = label + ".y";
  lines.push_back(y);

  dart::gui::DebugLineDescriptor z;
  z.from = center - Eigen::Vector3d::UnitZ() * radius;
  z.to = center + Eigen::Vector3d::UnitZ() * radius;
  z.label = label + ".z";
  lines.push_back(z);
}

std::vector<dart::gui::DebugLineDescriptor> makeAtlasSupportPolygonLines(
    const dart::dynamics::SkeletonPtr& atlas)
{
  if (atlas == nullptr) {
    return {};
  }

  dart::gui::DebugDrawOptions options;
  options.drawGrid = false;
  options.drawWorldFrame = false;
  options.drawSupportPolygons = true;
  options.supportPolygonElevation = kSupportVisualElevation;
  return dart::gui::makeSupportPolygonDebugLines(*atlas, options, "atlas");
}

std::optional<Eigen::Vector2d> computeAtlasComSupportProjection(
    const dart::dynamics::SkeletonPtr& atlas)
{
  if (atlas == nullptr || atlas->getMass() <= 0.0
      || !std::isfinite(atlas->getMass())) {
    return std::nullopt;
  }

  const auto& axes = atlas->getSupportAxes();
  if (!axes.first.allFinite() || !axes.second.allFinite()) {
    return std::nullopt;
  }

  const Eigen::Vector3d com = atlas->getCOM();
  if (!com.allFinite()) {
    return std::nullopt;
  }

  return Eigen::Vector2d(com.dot(axes.first), com.dot(axes.second));
}

std::vector<dart::gui::DebugLineDescriptor> makeAtlasSupportComLines(
    const dart::dynamics::SkeletonPtr& atlas)
{
  if (atlas == nullptr) {
    return {};
  }

  const auto projectedCom = computeAtlasComSupportProjection(atlas);
  if (!projectedCom) {
    return {};
  }

  const auto& axes = atlas->getSupportAxes();
  const Eigen::Vector3d up = axes.first.cross(axes.second);
  if (!up.allFinite() || up.squaredNorm() <= 1e-18) {
    return {};
  }

  const Eigen::Vector3d center = axes.first * projectedCom->x()
                                 + axes.second * projectedCom->y()
                                 + up.normalized() * kSupportVisualElevation;
  std::vector<dart::gui::DebugLineDescriptor> lines;
  lines.reserve(3);
  appendMarkerAxisLines(
      lines, center, kSupportComMarkerRadius, "atlas.support_com");
  return lines;
}

Eigen::Vector4d atlasSupportComColor(const dart::dynamics::SkeletonPtr& atlas)
{
  const auto projectedCom = computeAtlasComSupportProjection(atlas);
  if (atlas == nullptr || !projectedCom) {
    return Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
  }

  return dart::math::isInsideSupportPolygon(
             *projectedCom, atlas->getSupportPolygon())
             ? Eigen::Vector4d(0.0, 0.0, 1.0, 1.0)
             : Eigen::Vector4d(1.0, 0.0, 0.0, 1.0);
}

dart::dynamics::SimpleFramePtr createAtlasSupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& atlas)
{
  auto overlay = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kAtlasSupportOverlayName);
  overlay->setShape(createLineShape(makeAtlasSupportPolygonLines(atlas)));
  overlay->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.22, 0.86, 0.38, 0.86));
  return overlay;
}

dart::dynamics::SimpleFramePtr createAtlasSupportComOverlay(
    const dart::dynamics::SkeletonPtr& atlas)
{
  auto overlay = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kAtlasSupportComOverlayName);
  overlay->setShape(createLineShape(makeAtlasSupportComLines(atlas)));
  overlay->getVisualAspect(true)->setRGBA(atlasSupportComColor(atlas));
  return overlay;
}

void updateAtlasSupportPolygonOverlay(
    const dart::dynamics::SkeletonPtr& atlas,
    const dart::dynamics::SimpleFramePtr& overlay)
{
  if (overlay == nullptr) {
    return;
  }

  overlay->setShape(createLineShape(makeAtlasSupportPolygonLines(atlas)));
}

void updateAtlasSupportComOverlay(
    const dart::dynamics::SkeletonPtr& atlas,
    const dart::dynamics::SimpleFramePtr& overlay)
{
  if (overlay == nullptr) {
    return;
  }

  overlay->setShape(createLineShape(makeAtlasSupportComLines(atlas)));
  overlay->getVisualAspect(true)->setRGBA(atlasSupportComColor(atlas));
}

void setUnconstrainedIkBounds(const dart::dynamics::InverseKinematicsPtr& ik)
{
  Eigen::Vector3d linearBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d angularBounds
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
}

void setVectorEntry(Eigen::VectorXd& values, Eigen::Index index, double value)
{
  if (0 <= index && index < values.size()) {
    values[index] = value;
  }
}

void scaleVectorEntry(Eigen::VectorXd& values, Eigen::Index index, double scale)
{
  if (0 <= index && index < values.size()) {
    values[index] *= scale;
  }
}

AtlasWholeBodySolverState setupAtlasWholeBodySolver(
    const dart::dynamics::SkeletonPtr& atlas)
{
  AtlasWholeBodySolverState state;
  if (atlas == nullptr) {
    return state;
  }

  auto wholeBodyIk = atlas->getIK(true);
  auto solver = std::dynamic_pointer_cast<dart::math::GradientDescentSolver>(
      wholeBodyIk->getSolver());
  if (solver != nullptr) {
    solver->setNumMaxIterations(10);
  }

  const Eigen::Index dofCount = static_cast<Eigen::Index>(atlas->getNumDofs());
  constexpr double defaultWeight = 0.01;
  Eigen::VectorXd weights = defaultWeight * Eigen::VectorXd::Ones(dofCount);
  setVectorEntry(weights, 2, 0.0);
  setVectorEntry(weights, 3, 0.0);
  setVectorEntry(weights, 4, 0.0);
  scaleVectorEntry(weights, 6, 0.2);
  scaleVectorEntry(weights, 7, 0.2);
  scaleVectorEntry(weights, 8, 0.2);

  Eigen::VectorXd lowerPosture = Eigen::VectorXd::Constant(
      dofCount, -std::numeric_limits<double>::infinity());
  setVectorEntry(lowerPosture, 0, -0.35);
  setVectorEntry(lowerPosture, 1, -0.35);
  setVectorEntry(lowerPosture, 5, 0.600);
  setVectorEntry(lowerPosture, 6, -0.1);
  setVectorEntry(lowerPosture, 7, -0.1);
  setVectorEntry(lowerPosture, 8, -0.1);

  Eigen::VectorXd upperPosture = Eigen::VectorXd::Constant(
      dofCount, std::numeric_limits<double>::infinity());
  setVectorEntry(upperPosture, 0, 0.35);
  setVectorEntry(upperPosture, 1, 0.35);
  setVectorEntry(upperPosture, 5, 0.885);
  setVectorEntry(upperPosture, 6, 0.1);
  setVectorEntry(upperPosture, 7, 0.1);
  setVectorEntry(upperPosture, 8, 0.1);

  state.posture = std::make_shared<RelaxedPosture>(
      atlas->getPositions(), lowerPosture, upperPosture, weights);
  wholeBodyIk->setObjective(state.posture);

  state.balance = std::make_shared<dart::constraint::BalanceConstraint>(
      wholeBodyIk,
      dart::constraint::BalanceConstraint::SHIFT_SUPPORT,
      dart::constraint::BalanceConstraint::FROM_CENTROID);
  wholeBodyIk->getProblem()->addEqConstraint(state.balance);
  return state;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(4.0, 4.0, 0.04)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.02));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.86, 0.88, 0.90, 1.0));
  return ground;
}

struct AtlasPuppetScene
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr atlas;
  dart::dynamics::SimpleFramePtr supportOverlay;
  dart::dynamics::SimpleFramePtr supportComOverlay;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::Gizmo> gizmos;
  AtlasWholeBodySolverState wholeBodySolver;
  struct TargetState
  {
    dart::simulation::WorldPtr world;
    dart::dynamics::EndEffector* effector = nullptr;
    dart::dynamics::InverseKinematicsPtr ik;
    dart::dynamics::SimpleFramePtr target;
    std::pair<Eigen::Vector6d, Eigen::Vector6d> defaultBounds;
    Eigen::Isometry3d defaultTargetTransform = Eigen::Isometry3d::Identity();
    std::string label;
    char hotkey = '\0';
    bool active = false;

    void activate(bool resetTargetTransform = true)
    {
      if (active || world == nullptr || effector == nullptr || target == nullptr
          || ik == nullptr) {
        return;
      }

      ik->getErrorMethod().setBounds();
      if (resetTargetTransform) {
        target->setTransform(effector->getWorldTransform());
      }
      world->addSimpleFrame(target);
      active = true;
      std::cout << "Activated IK target '" << effector->getName() << "'.\n";
    }

    void deactivate()
    {
      if (!active || world == nullptr || target == nullptr || ik == nullptr) {
        return;
      }

      ik->getErrorMethod().setBounds(defaultBounds);
      target->setTransform(defaultTargetTransform);
      world->removeSimpleFrame(target);
      active = false;
      std::cout << "Deactivated IK target '" << effector->getName() << "'.\n";
    }

    void solve()
    {
      if (active && ik != nullptr) {
        ik->solveAndApply(true);
      }
    }

    void toggle()
    {
      if (active) {
        deactivate();
      } else {
        activate();
      }
    }
  };

  std::vector<std::shared_ptr<TargetState>> targetStates;
  Eigen::VectorXd restConfiguration;
};

void addAtlasPuppetIkTargets(
    AtlasPuppetScene& scene, const dart::dynamics::SkeletonPtr& atlas)
{
  struct Config
  {
    const char* bodyNode;
    const char* effectorName;
    const char* targetName;
    const char* label;
    int hotkey;
    Eigen::Isometry3d relativeTransform;
    bool supportContact;
  };

  constexpr double halfPi = 1.5707963267948966;
  Eigen::Isometry3d leftHand = Eigen::Isometry3d::Identity();
  leftHand.translation() = Eigen::Vector3d(0.0009, 0.1254, 0.012);
  leftHand.rotate(Eigen::AngleAxisd(halfPi, Eigen::Vector3d::UnitZ()));

  Eigen::Isometry3d rightHand = leftHand;
  rightHand.translation().x() = -rightHand.translation().x();
  rightHand.translation().y() = -rightHand.translation().y();
  rightHand.linear() = rightHand.linear().inverse().eval();

  Eigen::Isometry3d foot = Eigen::Isometry3d::Identity();
  foot.translation() = Eigen::Vector3d(0.186, 0.0, -0.08);

  const std::array<Config, 4> configs{{
      {"l_hand",
       "atlas_puppet_left_hand",
       "atlas_puppet_ik_target_left_hand",
       "1 left hand",
       '1',
       leftHand,
       false},
      {"r_hand",
       "atlas_puppet_right_hand",
       "atlas_puppet_ik_target_right_hand",
       "2 right hand",
       '2',
       rightHand,
       false},
      {"l_foot",
       "atlas_puppet_left_foot",
       "atlas_puppet_ik_target_left_foot",
       "3 left foot",
       '3',
       foot,
       true},
      {"r_foot",
       "atlas_puppet_right_foot",
       "atlas_puppet_ik_target_right_foot",
       "4 right foot",
       '4',
       foot,
       true},
  }};

  const auto footSupportGeometry = makeAtlasPuppetFootSupportGeometry();
  for (const Config& config : configs) {
    auto* bodyNode = atlas->getBodyNode(config.bodyNode);
    if (bodyNode == nullptr) {
      throw std::runtime_error(
          "Atlas puppet model is missing body node "
          + std::string(config.bodyNode));
    }

    auto* endEffector = bodyNode->createEndEffector(config.effectorName);
    if (config.supportContact) {
      endEffector->setRelativeTransform(config.relativeTransform);
      auto* support = endEffector->getSupport(true);
      support->setGeometry(footSupportGeometry);
      support->setActive(true);
    } else {
      endEffector->setDefaultRelativeTransform(config.relativeTransform, true);
    }

    auto ik = endEffector->getIK(true);
    ik->useWholeBody();
    ik->setGradientMethod<
        dart::dynamics::InverseKinematics::JacobianTranspose>();
    ik->getSolver()->setNumMaxIterations(30);
    if (config.supportContact) {
      ik->setHierarchyLevel(1);
      Eigen::Vector3d linearBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d angularBounds
          = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      linearBounds.z() = 1e-8;
      angularBounds.x() = 1e-8;
      angularBounds.y() = 1e-8;
      ik->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
      ik->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
    } else {
      setUnconstrainedIkBounds(ik);
    }

    auto target = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(),
        config.targetName,
        endEffector->getWorldTransform());
    ik->setTarget(target);

    auto state = std::make_shared<AtlasPuppetScene::TargetState>();
    state->world = scene.world;
    state->effector = endEffector;
    state->ik = ik;
    state->target = target;
    state->defaultBounds = ik->getErrorMethod().getBounds();
    state->defaultTargetTransform = target->getRelativeTransform();
    state->label = config.label;
    state->hotkey = config.hotkey;

    dart::gui::InverseKinematicsHandle handle;
    handle.label = config.label;
    handle.hotkey = config.hotkey;
    handle.target = target;
    handle.ik = std::move(ik);

    dart::gui::Gizmo gizmo;
    gizmo.label = config.targetName;
    gizmo.target = target;
    gizmo.size = 0.24;
    gizmo.isVisible = []() {
      return true;
    };
    gizmo.onChanged = [state](const Eigen::Isometry3d&) {
      if (!state->active) {
        state->activate(false);
      }
      state->solve();
    };

    scene.gizmos.push_back(std::move(gizmo));
    scene.ikHandles.push_back(std::move(handle));
    scene.targetStates.push_back(std::move(state));
  }
}

AtlasPuppetScene createAtlasPuppetScene()
{
  AtlasPuppetScene scene;
  scene.world = dart::simulation::World::create("dartsim_atlas_puppet");
  scene.world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  scene.world->addSkeleton(createGround());

  auto atlas = loadAtlasPuppetSkeleton();
  scene.atlas = atlas;
  scene.world->addSkeleton(atlas);
  addAtlasPuppetIkTargets(scene, atlas);
  scene.wholeBodySolver = setupAtlasWholeBodySolver(scene.atlas);
  scene.restConfiguration = atlas->getPositions();
  scene.supportOverlay = createAtlasSupportPolygonOverlay(scene.atlas);
  scene.world->addSimpleFrame(scene.supportOverlay);
  scene.supportComOverlay = createAtlasSupportComOverlay(scene.atlas);
  scene.world->addSimpleFrame(scene.supportComOverlay);
  return scene;
}

void solveAtlasWholeBody(const dart::dynamics::SkeletonPtr& atlas)
{
  const auto wholeBodyIk = atlas ? atlas->getIK() : nullptr;
  if (wholeBodyIk != nullptr) {
    wholeBodyIk->solveAndApply(true);
  }
}

bool applyRootTeleoperationStep(
    const dart::dynamics::SkeletonPtr& atlas, PuppetMotion motion)
{
  auto* rootBody = atlas ? atlas->getRootBodyNode() : nullptr;
  if (rootBody == nullptr
      || dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
             == nullptr) {
    return false;
  }

  const Eigen::Isometry3d current = rootBody->getWorldTransform();
  Eigen::Isometry3d next = current;

  Eigen::Vector3d forward = current.linear().col(0);
  forward.z() = 0.0;
  if (forward.norm() > 1e-10) {
    forward.normalize();
  } else {
    forward.setZero();
  }

  Eigen::Vector3d left = current.linear().col(1);
  left.z() = 0.0;
  if (left.norm() > 1e-10) {
    left.normalize();
  } else {
    left.setZero();
  }

  switch (motion) {
    case PuppetMotion::Forward:
      next.translation() += kTeleopLinearStep * forward;
      break;
    case PuppetMotion::Backward:
      next.translation() -= kTeleopLinearStep * forward;
      break;
    case PuppetMotion::Left:
      next.translation() += kTeleopLinearStep * left;
      break;
    case PuppetMotion::Right:
      next.translation() -= kTeleopLinearStep * left;
      break;
    case PuppetMotion::Up:
      next.translation() += kTeleopElevationStep * Eigen::Vector3d::UnitZ();
      break;
    case PuppetMotion::Down:
      next.translation() -= kTeleopElevationStep * Eigen::Vector3d::UnitZ();
      break;
    case PuppetMotion::YawLeft:
      next.linear()
          = Eigen::AngleAxisd(kTeleopYawStep, Eigen::Vector3d::UnitZ())
                .toRotationMatrix()
            * current.linear();
      break;
    case PuppetMotion::YawRight:
      next.linear()
          = Eigen::AngleAxisd(-kTeleopYawStep, Eigen::Vector3d::UnitZ())
                .toRotationMatrix()
            * current.linear();
      break;
  }

  dart::dynamics::FreeJoint::setTransformOf(rootBody, next);
  return true;
}

std::vector<dart::gui::KeyboardAction> createAtlasPuppetKeyboardActions(
    const dart::dynamics::SkeletonPtr& atlas,
    const std::vector<std::shared_ptr<AtlasPuppetScene::TargetState>>&
        targetStates,
    const Eigen::VectorXd& restConfiguration,
    const AtlasWholeBodySolverState& wholeBodySolver)
{
  struct Config
  {
    char key;
    const char* label;
    PuppetMotion motion;
  };

  const std::array<Config, 8> configs{{
      {'w', "Move Atlas forward", PuppetMotion::Forward},
      {'s', "Move Atlas backward", PuppetMotion::Backward},
      {'a', "Move Atlas left", PuppetMotion::Left},
      {'d', "Move Atlas right", PuppetMotion::Right},
      {'f', "Raise Atlas root", PuppetMotion::Up},
      {'z', "Lower Atlas root", PuppetMotion::Down},
      {'q', "Yaw Atlas left", PuppetMotion::YawLeft},
      {'e', "Yaw Atlas right", PuppetMotion::YawRight},
  }};

  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(configs.size() + targetStates.size() + 6);
  for (const Config& config : configs) {
    dart::gui::KeyboardAction action;
    action.label = config.label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(config.key);
    action.repeat = true;
    action.callback = [atlas, motion = config.motion](
                          dart::gui::KeyboardActionContext& context) {
      if (applyRootTeleoperationStep(atlas, motion)) {
        solveAtlasWholeBody(atlas);
        if (context.lifecycle != nullptr) {
          context.lifecycle->paused = true;
        }
      }
    };
    actions.push_back(std::move(action));
  }

  for (const auto& state : targetStates) {
    if (state == nullptr || state->hotkey == '\0') {
      continue;
    }

    dart::gui::KeyboardAction action;
    action.label = "Toggle Atlas target " + state->label;
    action.shortcut = dart::gui::KeyboardShortcut::characterKey(state->hotkey);
    action.callback
        = [atlas, state](dart::gui::KeyboardActionContext& context) {
            state->toggle();
            solveAtlasWholeBody(atlas);
            if (context.lifecycle != nullptr) {
              context.lifecycle->paused = true;
            }
          };
    actions.push_back(std::move(action));
  }

  const auto addSupportToggle
      = [&](char key, const char* effectorName, const char* label) {
          dart::gui::KeyboardAction action;
          action.label = label;
          action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
          action.callback =
              [atlas, effectorName](dart::gui::KeyboardActionContext& context) {
                auto* effector
                    = atlas ? atlas->getEndEffector(effectorName) : nullptr;
                auto* support = effector ? effector->getSupport() : nullptr;
                if (support != nullptr) {
                  support->setActive(!support->isActive());
                  if (context.lifecycle != nullptr) {
                    context.lifecycle->paused = true;
                  }
                }
              };
          actions.push_back(std::move(action));
        };
  addSupportToggle(
      'x', "atlas_puppet_left_foot", "Toggle left Atlas foot support");
  addSupportToggle(
      'c', "atlas_puppet_right_foot", "Toggle right Atlas foot support");

  dart::gui::KeyboardAction printDofs;
  printDofs.label = "Print Atlas DOFs";
  printDofs.shortcut = dart::gui::KeyboardShortcut::characterKey('p');
  printDofs.callback = [atlas](dart::gui::KeyboardActionContext&) {
    if (atlas == nullptr) {
      return;
    }
    for (std::size_t i = 0; i < atlas->getNumDofs(); ++i) {
      auto* dof = atlas->getDof(i);
      std::cout << dof->getName() << ": " << dof->getPosition() << "\n";
    }
  };
  actions.push_back(std::move(printDofs));

  dart::gui::KeyboardAction resetPosture;
  resetPosture.label = "Reset Atlas relaxed posture";
  resetPosture.shortcut = dart::gui::KeyboardShortcut::characterKey('t');
  resetPosture.callback = [atlas, restConfiguration](
                              dart::gui::KeyboardActionContext& context) {
    if (atlas == nullptr
        || static_cast<std::size_t>(restConfiguration.size())
               != atlas->getNumDofs()) {
      return;
    }

    for (std::size_t i = 0; i < atlas->getNumDofs(); ++i) {
      if (i < 2 || 4 < i) {
        atlas->getDof(i)->setPosition(restConfiguration[static_cast<int>(i)]);
      }
    }
    solveAtlasWholeBody(atlas);
    if (context.lifecycle != nullptr) {
      context.lifecycle->paused = true;
    }
  };
  actions.push_back(std::move(resetPosture));

  dart::gui::KeyboardAction optimizePosture;
  optimizePosture.label = "Optimize Atlas posture and balance";
  optimizePosture.shortcut = dart::gui::KeyboardShortcut::characterKey('r');
  optimizePosture.callback
      = [wholeBodySolver, atlas](dart::gui::KeyboardActionContext& context) {
          if (wholeBodySolver.posture != nullptr) {
            wholeBodySolver.posture->enforceIdealPosture = true;
          }
          if (wholeBodySolver.balance != nullptr) {
            wholeBodySolver.balance->setErrorMethod(
                dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);
          }
          solveAtlasWholeBody(atlas);
          if (context.lifecycle != nullptr) {
            context.lifecycle->paused = true;
          }
        };
  actions.push_back(std::move(optimizePosture));

  dart::gui::KeyboardAction restoreBalanceMode;
  restoreBalanceMode.label = "Restore Atlas centroid balance mode";
  restoreBalanceMode.shortcut = dart::gui::KeyboardShortcut::characterKey('r');
  restoreBalanceMode.trigger = dart::gui::KeyboardActionTrigger::Release;
  restoreBalanceMode.callback
      = [wholeBodySolver, atlas](dart::gui::KeyboardActionContext& context) {
          if (wholeBodySolver.posture != nullptr) {
            wholeBodySolver.posture->enforceIdealPosture = false;
          }
          if (wholeBodySolver.balance != nullptr) {
            wholeBodySolver.balance->setErrorMethod(
                dart::constraint::BalanceConstraint::FROM_CENTROID);
          }
          solveAtlasWholeBody(atlas);
          if (context.lifecycle != nullptr) {
            context.lifecycle->paused = true;
          }
        };
  actions.push_back(std::move(restoreBalanceMode));
  return actions;
}

dart::gui::Panel createAtlasPuppetPanel()
{
  dart::gui::Panel panel;
  panel.title = "Atlas Puppet";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Atlas whole-body IK puppet");
    builder.text("Press 1-4 to toggle/select active targets.");
    builder.text("Left-drag target gizmo handles.");
    builder.text("Arrow keys and PageUp/PageDown nudge it.");
    builder.text("Hold X/Y/Z with Ctrl-drag to constrain an axis.");
    builder.text("WASD moves the root; Q/E yaw; F/Z height.");
    builder.text("X/C toggles foot support; P prints DOFs; T resets posture.");
    builder.text("Hold R to optimize whole-body posture and balance.");
    builder.text("Blue/red COM marker shows support-polygon validity.");
    builder.text("Whole-body IK solves active targets and balance each step.");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

dart::gui::RunOptions makeAtlasPuppetRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeAtlasPuppetCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 1.0);
  camera.yaw = 0.5118558424318241;
  camera.pitch = 0.22626228031830084;
  camera.distance = 6.285196894290584;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    AtlasPuppetScene scene = createAtlasPuppetScene();

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.ikHandles = scene.ikHandles;
    options.gizmos = scene.gizmos;
    options.runDefaults = makeAtlasPuppetRunDefaults();
    options.camera = makeAtlasPuppetCamera();
    options.preStep = [atlas = scene.atlas,
                       supportOverlay = scene.supportOverlay,
                       supportComOverlay = scene.supportComOverlay]() {
      solveAtlasWholeBody(atlas);
      updateAtlasSupportPolygonOverlay(atlas, supportOverlay);
      updateAtlasSupportComOverlay(atlas, supportComOverlay);
    };
    options.keyboardActions = createAtlasPuppetKeyboardActions(
        scene.atlas,
        scene.targetStates,
        scene.restConfiguration,
        scene.wholeBodySolver);
    options.panels.push_back(createAtlasPuppetPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "atlas_puppet: " << e.what() << "\n";
    return 1;
  }
}
