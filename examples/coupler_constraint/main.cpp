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

#include <dart/gui/grid_visual.hpp>
#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/im_gui_viewer.hpp>
#include <dart/gui/im_gui_widget.hpp>
#include <dart/gui/include_im_gui.hpp>
#include <dart/gui/real_time_world_node.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/constants.hpp>

#include <CLI/CLI.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <cassert>
#include <cstdlib>

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::Joint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::simulation::World;
using dart::simulation::WorldPtr;

namespace {

struct MimicAssembly
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::Joint* referenceJoint;
  dart::dynamics::Joint* followerJoint;
  dart::dynamics::BodyNode* referenceBody;
  dart::dynamics::BodyNode* followerBody;
  dart::dynamics::SimpleFramePtr lowerLimitGuide;
  dart::dynamics::SimpleFramePtr upperLimitGuide;
};

dart::dynamics::SimpleFramePtr createLimitGuide(
    const std::string& name,
    dart::dynamics::Joint* followerJoint,
    double angle,
    const Eigen::Vector3d& color,
    const dart::simulation::WorldPtr& world)
{
  using namespace dart::dynamics;
  if (!world || !followerJoint) {
    return nullptr;
  }

  const auto* revolute
      = dynamic_cast<dart::dynamics::RevoluteJoint*>(followerJoint);
  if (revolute == nullptr) {
    return nullptr;
  }

  auto frame = SimpleFrame::createShared(Frame::World(), name);
  auto line = std::make_shared<LineSegmentShape>(0.05f);
  line->addVertex(Eigen::Vector3d::Zero());
  line->addVertex(Eigen::Vector3d::UnitX() * 0.65);
  line->addConnection(0, 1);
  frame->setShape(line);
  auto visual = frame->createVisualAspect();
  visual->setColor(color);

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  if (const auto* parent = followerJoint->getParentBodyNode()) {
    tf = parent->getWorldTransform();
  }
  tf = tf * followerJoint->getTransformFromParentBodyNode();
  tf.rotate(Eigen::AngleAxisd(angle, revolute->getAxis()));
  tf.translate(Eigen::Vector3d(0.0, 0.0, 0.02));
  frame->setRelativeTransform(tf);

  world->addSimpleFrame(frame);
  return frame;
}

MimicAssembly createMimicAssembly(
    const std::string& label,
    const Eigen::Vector3d& baseOffset,
    bool useCouplerConstraint,
    const Eigen::Vector3d& referenceColor,
    const Eigen::Vector3d& followerColor,
    const dart::simulation::WorldPtr& world)
{
  SkeletonPtr skeleton = Skeleton::create(label + "_rig");

  const std::string baseJointName = label + "_base_joint";
  const std::string baseBodyName = label + "_base";
  const std::string referenceJointName = label + "_reference_joint";
  const std::string referenceBodyName = label + "_reference_body";
  const std::string followerJointName = label + "_follower_joint";
  const std::string followerBodyName = label + "_follower_body";

  dart::dynamics::WeldJoint::Properties baseJointProps;
  baseJointProps.mName = baseJointName;
  baseJointProps.mT_ParentBodyToJoint = Eigen::Translation3d(baseOffset);

  BodyNode::Properties baseBodyProps;
  baseBodyProps.mName = baseBodyName;
  dart::dynamics::Inertia baseInertia;
  baseInertia.setMass(0.1);
  baseInertia.setMoment(1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0);
  baseBodyProps.mInertia = baseInertia;

  auto basePair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
          nullptr, baseJointProps, baseBodyProps);
  auto* baseBody = basePair.second;

  RevoluteJoint::Properties referenceJointProps;
  referenceJointProps.mName = referenceJointName;
  referenceJointProps.mAxis = Eigen::Vector3d::UnitZ();
  referenceJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, 0.15, 0.0);
  referenceJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  BodyNode::Properties referenceBodyProps;
  referenceBodyProps.mName = referenceBodyName;
  dart::dynamics::Inertia referenceInertia;
  referenceInertia.setMass(1.0);
  referenceInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  referenceBodyProps.mInertia = referenceInertia;

  auto referencePair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      baseBody, referenceJointProps, referenceBodyProps);
  auto* referenceJoint = referencePair.first;
  auto* referenceBody = referencePair.second;

  referenceJoint->setActuatorType(Joint::FORCE);
  referenceJoint->setForceLowerLimit(0, -120.0);
  referenceJoint->setForceUpperLimit(0, 120.0);
  referenceJoint->setDampingCoefficient(0, 0.02);

  auto referenceShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.05, 0.05));
  auto referenceShapeNode = referenceBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(referenceShape);
  referenceShapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));
  referenceShapeNode->getVisualAspect()->setColor(referenceColor);

  RevoluteJoint::Properties followerJointProps;
  followerJointProps.mName = followerJointName;
  followerJointProps.mAxis = Eigen::Vector3d::UnitZ();
  followerJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, -0.15, 0.0);
  followerJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  BodyNode::Properties followerBodyProps;
  followerBodyProps.mName = followerBodyName;
  dart::dynamics::Inertia followerInertia;
  followerInertia.setMass(1.0);
  followerInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  followerBodyProps.mInertia = followerInertia;

  auto followerPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      baseBody, followerJointProps, followerBodyProps);
  auto* followerJoint = followerPair.first;
  auto* followerBody = followerPair.second;

  followerJoint->setActuatorType(Joint::MIMIC);
  followerJoint->setMimicJoint(referenceJoint, -1.0, 0.0);
  followerJoint->setUseCouplerConstraint(useCouplerConstraint);
  followerJoint->setForceLowerLimit(0, -120.0);
  followerJoint->setForceUpperLimit(0, 120.0);
  followerJoint->setDampingCoefficient(0, 0.02);
  followerJoint->setPositionLowerLimit(0, -0.35);
  followerJoint->setPositionUpperLimit(0, 0.35);
  followerJoint->setLimitEnforcement(true);

  auto followerShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.05, 0.05));
  auto followerShapeNode = followerBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(followerShape);
  followerShapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));
  followerShapeNode->getVisualAspect()->setColor(followerColor);

  referenceJoint->setPosition(0, 0.0);
  followerJoint->setPosition(0, 0.0);

  MimicAssembly assembly{
      skeleton,
      referenceJoint,
      followerJoint,
      referenceBody,
      followerBody,
      nullptr,
      nullptr,
  };

  if (world) {
    const double lower = followerJoint->getPositionLowerLimit(0);
    const double upper = followerJoint->getPositionUpperLimit(0);
    assembly.lowerLimitGuide = createLimitGuide(
        label + "_lower_limit",
        followerJoint,
        lower,
        Eigen::Vector3d(0.92, 0.35, 0.35),
        world);
    assembly.upperLimitGuide = createLimitGuide(
        label + "_upper_limit",
        followerJoint,
        upper,
        Eigen::Vector3d(0.35, 0.92, 0.35),
        world);
  }

  return assembly;
}

class CouplerController
{
public:
  struct PairRegistration
  {
    std::string label;
    bool usesCoupler;
    dart::dynamics::SkeletonPtr skeleton;
    dart::dynamics::Joint* referenceJoint;
    dart::dynamics::Joint* followerJoint;
    dart::dynamics::BodyNode* referenceBody;
    dart::dynamics::BodyNode* followerBody;
    dart::dynamics::SimpleFramePtr linkFrame;
    dart::dynamics::LineSegmentShapePtr linkShape;
    dart::dynamics::VisualAspect* linkVisual;
    double targetAngle{0.0};
    double torqueLimit{80.0};
    double proportionalGain{300.0};
    double dampingGain{20.0};
  };

  struct PairStatus
  {
    std::string label;
    bool usesCoupler;
    bool couplerEnabled;
    double targetAngle;
    double referencePosition;
    double followerPosition;
    double desiredFollowerPosition;
    double positionError;
    double followerLowerLimit;
    double followerUpperLimit;
    bool followerAtLowerLimit;
    bool followerAtUpperLimit;
  };

  void addPair(PairRegistration registration)
  {
    PairData pair;
    pair.label = registration.label;
    pair.usesCoupler = registration.usesCoupler;
    pair.couplerEnabled
        = registration.usesCoupler
              ? registration.followerJoint->isUsingCouplerConstraint()
              : false;
    pair.skeleton = std::move(registration.skeleton);
    pair.referenceJoint = registration.referenceJoint;
    pair.followerJoint = registration.followerJoint;
    pair.referenceBody = registration.referenceBody;
    pair.followerBody = registration.followerBody;
    pair.linkFrame = std::move(registration.linkFrame);
    pair.linkShape = std::move(registration.linkShape);
    pair.linkVisual = registration.linkVisual;
    pair.initialPositions = pair.skeleton->getPositions();
    pair.targetAngle = registration.targetAngle;
    pair.torqueLimit = registration.torqueLimit;
    pair.kp = registration.proportionalGain;
    pair.kd = registration.dampingGain;

    mPairs.push_back(std::move(pair));
    refreshPairVisual(mPairs.back());
  }

  void update()
  {
    for (auto& pair : mPairs) {
      driveReferenceJoint(pair);
      refreshPairVisual(pair);
    }
  }

  void reset()
  {
    for (auto& pair : mPairs) {
      if (!pair.skeleton) {
        continue;
      }
      pair.skeleton->setPositions(pair.initialPositions);
      pair.skeleton->setVelocities(
          Eigen::VectorXd::Zero(pair.skeleton->getNumDofs()));
      pair.skeleton->clearExternalForces();
    }
  }

  std::vector<PairStatus> getStatuses() const
  {
    std::vector<PairStatus> result;
    result.reserve(mPairs.size());

    for (const auto& pair : mPairs) {
      PairStatus status;
      status.label = pair.label;
      status.usesCoupler = pair.usesCoupler;
      status.couplerEnabled = pair.couplerEnabled;
      status.targetAngle = pair.targetAngle;
      status.referencePosition
          = pair.referenceJoint ? pair.referenceJoint->getPosition(0) : 0.0;
      status.followerPosition
          = pair.followerJoint ? pair.followerJoint->getPosition(0) : 0.0;
      status.desiredFollowerPosition
          = pair.followerJoint ? computeDesiredFollowerPosition(pair) : 0.0;
      status.positionError
          = status.followerPosition - status.desiredFollowerPosition;
      status.followerLowerLimit
          = pair.followerJoint ? pair.followerJoint->getPositionLowerLimit(0)
                               : 0.0;
      status.followerUpperLimit
          = pair.followerJoint ? pair.followerJoint->getPositionUpperLimit(0)
                               : 0.0;
      const double lowerSlack
          = status.followerPosition - status.followerLowerLimit;
      const double upperSlack
          = status.followerUpperLimit - status.followerPosition;
      status.followerAtLowerLimit = lowerSlack < 1e-3;
      status.followerAtUpperLimit = upperSlack < 1e-3;

      result.push_back(status);
    }

    return result;
  }

private:
  struct PairData
  {
    std::string label;
    bool usesCoupler{false};
    bool couplerEnabled{false};
    dart::dynamics::SkeletonPtr skeleton;
    dart::dynamics::Joint* referenceJoint{nullptr};
    dart::dynamics::Joint* followerJoint{nullptr};
    dart::dynamics::BodyNode* referenceBody{nullptr};
    dart::dynamics::BodyNode* followerBody{nullptr};
    dart::dynamics::SimpleFramePtr linkFrame;
    dart::dynamics::LineSegmentShapePtr linkShape;
    dart::dynamics::VisualAspect* linkVisual{nullptr};
    Eigen::VectorXd initialPositions;
    double targetAngle{0.0};
    double torqueLimit{80.0};
    double kp{300.0};
    double kd{20.0};
  };

  double computeDesiredFollowerPosition(const PairData& pair) const
  {
    if (pair.followerJoint == nullptr) {
      return 0.0;
    }

    const auto& props = pair.followerJoint->getMimicDofProperties()[0];
    const auto* refJoint = props.mReferenceJoint;
    if (refJoint == nullptr) {
      return 0.0;
    }

    return refJoint->getPosition(props.mReferenceDofIndex) * props.mMultiplier
           + props.mOffset;
  }

  void refreshPairVisual(PairData& pair)
  {
    if (!pair.linkShape || !pair.referenceBody || !pair.followerBody) {
      return;
    }

    pair.linkShape->setVertex(
        0, pair.referenceBody->getTransform().translation());
    pair.linkShape->setVertex(
        1, pair.followerBody->getTransform().translation());

    if (!pair.linkVisual) {
      return;
    }

    const double desired = computeDesiredFollowerPosition(pair);
    const double actual
        = pair.followerJoint ? pair.followerJoint->getPosition(0) : 0.0;
    const double error = actual - desired;
    const double severity = std::min(1.0, std::abs(error) * 8.0);

    Eigen::Vector3d color;
    if (pair.usesCoupler) {
      const Eigen::Vector3d good(0.6, 0.95, 0.4);
      const Eigen::Vector3d bad(1.0, 0.6, 0.2);
      color = good + severity * (bad - good);
    } else {
      const Eigen::Vector3d base(0.7, 0.7, 0.85);
      const Eigen::Vector3d alert(1.0, 0.4, 0.3);
      color = base + severity * (alert - base);
    }
    pair.linkVisual->setColor(color);
  }

  void driveReferenceJoint(PairData& pair)
  {
    if (pair.referenceJoint == nullptr) {
      return;
    }

    const double position = pair.referenceJoint->getPosition(0);
    const double velocity = pair.referenceJoint->getVelocity(0);
    double torque
        = pair.kp * (pair.targetAngle - position) - pair.kd * velocity;
    torque = dart::math::clip(torque, -pair.torqueLimit, pair.torqueLimit);
    pair.referenceJoint->setForce(0, torque);
  }

  std::vector<PairData> mPairs;
};

class CouplerWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  CouplerWorldNode(const WorldPtr& world, CouplerController* controller)
    : RealTimeWorldNode(world), mController(controller)
  {
  }

  void customPreStep() override
  {
    mController->update();
  }

private:
  CouplerController* mController;
};

class CouplerEventHandler : public ::osgGA::GUIEventHandler
{
public:
  CouplerEventHandler(CouplerController* controller) : mController(controller)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN) {
      return false;
    }

    switch (ea.getKey()) {
      case 'r':
        mController->reset();
        std::cout << "Reset both rigs to their initial configuration"
                  << std::endl;
        return true;
      default:
        return false;
    }
  }

private:
  CouplerController* mController;
};

class CouplerOverlay : public dart::gui::ImGuiWidget
{
public:
  CouplerOverlay(dart::gui::ImGuiViewer* viewer, CouplerController* controller)
    : mViewer(viewer), mController(controller)
  {
  }

  void render() override
  {
    if (mViewer == nullptr || mController == nullptr) {
      return;
    }

    ImGui::SetNextWindowPos(ImVec2(12, 12));
    ImGui::SetNextWindowSize(ImVec2(360, 280), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.55f);

    if (!ImGui::Begin(
            "Coupler Constraint",
            nullptr,
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::End();
      return;
    }

    const double radToDeg = 180.0 / dart::math::pi;
    const auto statuses = mController->getStatuses();

    ImGui::TextWrapped(
        "Left rig: CouplerConstraint (bilateral). Right rig: Mimic motor "
        "(servo). Both reference joints chase the same command while the "
        "followers are limited to ±20°.");
    ImGui::Separator();

    for (const auto& status : statuses) {
      ImGui::Separator();
      ImGui::TextColored(
          status.usesCoupler ? ImVec4(0.8f, 1.0f, 0.6f, 1.0f)
                             : ImVec4(0.9f, 0.7f, 1.0f, 1.0f),
          "%s",
          status.label.c_str());
      ImGui::Text(
          "Constraint: %s",
          status.usesCoupler ? "Coupler (bilateral)" : "Mimic motor (servo)");
      ImGui::Text("Reference target: %6.2f deg", status.targetAngle * radToDeg);
      ImGui::Text(
          "Reference:        %6.2f deg", status.referencePosition * radToDeg);
      ImGui::Text(
          "Follower:         %6.2f deg", status.followerPosition * radToDeg);
      ImGui::Text(
          "Follower limits:  [%5.2f, %5.2f] deg",
          status.followerLowerLimit * radToDeg,
          status.followerUpperLimit * radToDeg);
      ImGui::Text(
          "Desired mimic:   %6.2f deg",
          status.desiredFollowerPosition * radToDeg);
      ImGui::TextColored(
          std::abs(status.positionError) < 1e-3
              ? ImVec4(0.8f, 1.0f, 0.6f, 1.0f)
              : ImVec4(1.0f, 0.6f, 0.3f, 1.0f),
          "Error:            %+6.3f deg",
          status.positionError * radToDeg);
      if (status.followerAtLowerLimit) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "Lower limit engaged");
      } else if (status.followerAtUpperLimit) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "Upper limit engaged");
      }
    }

    if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::TextWrapped("%s", mViewer->getInstructions().c_str());
    }

    ImGui::End();
  }

private:
  dart::gui::ImGuiViewer* mViewer;
  CouplerController* mController;
};

} // namespace

int main(int argc, char* argv[])
{
  CLI::App app("Coupler constraint demo");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1e-3);

  auto couplerAssembly = createMimicAssembly(
      "coupler",
      Eigen::Vector3d(-0.45, 0.0, 0.0),
      true,
      Eigen::Vector3d(0.85, 0.35, 0.25),
      Eigen::Vector3d(0.25, 0.58, 0.92),
      world);
  auto motorAssembly = createMimicAssembly(
      "motor",
      Eigen::Vector3d(0.45, 0.0, 0.0),
      false,
      Eigen::Vector3d(0.68, 0.32, 0.70),
      Eigen::Vector3d(0.25, 0.75, 0.70),
      world);

  world->addSkeleton(couplerAssembly.skeleton);
  world->addSkeleton(motorAssembly.skeleton);
  const double targetAngle
      = 45.0 * dart::math::pi / 180.0; // command for references

  std::cout
      << "Coupler constraint demo:\n"
      << "  • Left rig uses the bilateral CouplerConstraint (red/blue).\n"
      << "  • Right rig uses the legacy MimicMotorConstraint (purple/teal).\n"
      << "Both reference joints chase the same command while follower joints\n"
      << "are clamped to ±20°. When the follower saturates, only the Coupler\n"
      << "propagates the reaction torque back to the reference joint.\n"
      << "Press 'r' to reset both rigs to their initial pose.\n"
      << std::endl;

  auto controller = std::make_unique<CouplerController>();

  auto couplerFrame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "coupler_visual");
  auto couplerLine = std::make_shared<dart::dynamics::LineSegmentShape>(0.06f);
  couplerLine->addVertex(Eigen::Vector3d::Zero());
  couplerLine->addVertex(Eigen::Vector3d::Zero());
  couplerLine->addConnection(0, 1);
  couplerFrame->setShape(couplerLine);
  auto couplerVisual = couplerFrame->createVisualAspect();
  couplerVisual->setColor(Eigen::Vector3d(0.95, 0.95, 0.2));
  world->addSimpleFrame(couplerFrame);

  auto motorFrame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "motor_visual");
  auto motorLine = std::make_shared<dart::dynamics::LineSegmentShape>(0.04f);
  motorLine->addVertex(Eigen::Vector3d::Zero());
  motorLine->addVertex(Eigen::Vector3d::Zero());
  motorLine->addConnection(0, 1);
  motorFrame->setShape(motorLine);
  auto motorVisual = motorFrame->createVisualAspect();
  motorVisual->setColor(Eigen::Vector3d(0.75, 0.75, 0.9));
  world->addSimpleFrame(motorFrame);

  controller->addPair({
      .label = "Coupler pair (left)",
      .usesCoupler = true,
      .skeleton = couplerAssembly.skeleton,
      .referenceJoint = couplerAssembly.referenceJoint,
      .followerJoint = couplerAssembly.followerJoint,
      .referenceBody = couplerAssembly.referenceBody,
      .followerBody = couplerAssembly.followerBody,
      .linkFrame = couplerFrame,
      .linkShape = couplerLine,
      .linkVisual = couplerVisual,
      .targetAngle = targetAngle,
      .torqueLimit = 90.0,
      .proportionalGain = 320.0,
      .dampingGain = 25.0,
  });

  controller->addPair({
      .label = "Mimic motor pair (right)",
      .usesCoupler = false,
      .skeleton = motorAssembly.skeleton,
      .referenceJoint = motorAssembly.referenceJoint,
      .followerJoint = motorAssembly.followerJoint,
      .referenceBody = motorAssembly.referenceBody,
      .followerBody = motorAssembly.followerBody,
      .linkFrame = motorFrame,
      .linkShape = motorLine,
      .linkVisual = motorVisual,
      .targetAngle = targetAngle,
      .torqueLimit = 90.0,
      .proportionalGain = 320.0,
      .dampingGain = 25.0,
  });

  ::osg::ref_ptr<CouplerWorldNode> worldNode
      = new CouplerWorldNode(world, controller.get());
  ::osg::ref_ptr<CouplerEventHandler> handler
      = new CouplerEventHandler(controller.get());

  osg::ref_ptr<dart::gui::ImGuiViewer> viewer = new dart::gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  if (osg::GraphicsContext::getWindowingSystemInterface() == nullptr) {
    std::cerr << "No OSG windowing system detected. Running the GUI example "
                 "requires an active display server.\n";
    return EXIT_FAILURE;
  }
  viewer->addWorldNode(worldNode);
  viewer->addEventHandler(handler);

  viewer->addInstructionText("space: toggle simulation (auto-starts)\n");
  viewer->addInstructionText("'r': reset both rigs\n");
  std::cout << viewer->getInstructions() << std::endl;

  auto grid
      = ::osg::ref_ptr<dart::gui::GridVisual>(new dart::gui::GridVisual());
  grid->setPlaneType(dart::gui::GridVisual::PlaneType::XY);
  grid->setNumCells(25);
  grid->setMinorLineStepSize(0.05);
  viewer->addAttachment(grid);

  viewer->setUpViewInWindow(0, 0, 960, 720);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(1.5f, 1.5f, 1.2f),
      ::osg::Vec3(0.4f, 0.0f, 0.2f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  viewer->simulate(true);

  viewer->getImGuiHandler()->addWidget(
      std::make_shared<CouplerOverlay>(viewer.get(), controller.get()));

  if (!viewer->isRealized()) {
    viewer->realize();
  }

  osg::ref_ptr<osg::GraphicsContext> gc
      = viewer->getCamera() ? viewer->getCamera()->getGraphicsContext()
                            : nullptr;
  if (!viewer->isRealized() || !gc || !gc->valid()) {
    std::cerr << "Failed to create an OSG window. "
              << "Ensure DISPLAY is set or use a virtual framebuffer.\n";
    return EXIT_FAILURE;
  }

  const int runResult = viewer->run();
  if (runResult != 0) {
    std::cerr << "OSG viewer exited early (status " << runResult
              << "). Ensure a valid OpenGL context is available.\n";
    return runResult;
  }

  return 0;
}
