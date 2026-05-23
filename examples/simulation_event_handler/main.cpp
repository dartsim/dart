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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/sensor/sensor.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace {

constexpr const char* kGroundName = "simulation_event_handler_ground";
constexpr const char* kBoxPrefix = "simulation_event_handler_box_";
constexpr const char* kSphereName = "simulation_event_handler_sphere";
constexpr const char* kFastSensorFrameName
    = "simulation_event_handler_fast_sensor";
constexpr const char* kSlowSensorFrameName
    = "simulation_event_handler_slow_sensor";
constexpr const char* kForceArrowFrameName
    = "simulation_event_handler_force_arrow";
constexpr const char* kTorqueArrowFrameName
    = "simulation_event_handler_torque_arrow";
constexpr double kDefaultTimeStep = 0.001;
constexpr double kDefaultForceMagnitude = 100.0;
constexpr double kDefaultTorqueMagnitude = 50.0;

struct SensorMarker
{
  std::shared_ptr<dart::dynamics::SimpleFrame> frame;
  dart::dynamics::VisualAspect* visual = nullptr;
};

struct ArrowVisual
{
  dart::dynamics::SimpleFramePtr frame;
  std::shared_ptr<dart::dynamics::LineSegmentShape> shape;
  dart::dynamics::VisualAspect* visual = nullptr;
};

class BlinkingMarkerSensor final : public dart::sensor::Sensor
{
public:
  BlinkingMarkerSensor(
      const Properties& properties,
      const SensorMarker& marker,
      const Eigen::Vector4d& activeColor,
      const Eigen::Vector4d& inactiveColor)
    : dart::sensor::Sensor(properties),
      mMarker(marker.frame),
      mVisual(marker.visual),
      mActiveColor(activeColor),
      mInactiveColor(inactiveColor)
  {
    if (mVisual != nullptr) {
      mVisual->setRGBA(mInactiveColor);
    }
  }

private:
  void updateImpl(
      const dart::simulation::World&,
      const dart::sensor::SensorUpdateContext&) override
  {
    if (mVisual != nullptr) {
      mVisual->setRGBA(mPulseOn ? mActiveColor : mInactiveColor);
    }
    mPulseOn = !mPulseOn;
  }

  void resetImpl() override
  {
    mPulseOn = false;
    if (mVisual != nullptr) {
      mVisual->setRGBA(mInactiveColor);
    }
  }

  std::shared_ptr<dart::dynamics::SimpleFrame> mMarker;
  dart::dynamics::VisualAspect* mVisual = nullptr;
  Eigen::Vector4d mActiveColor;
  Eigen::Vector4d mInactiveColor;
  bool mPulseOn = false;
};

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(8.0, 8.0, 0.1)));
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.30, 0.30, 0.30));
  return ground;
}

dart::dynamics::SkeletonPtr createDynamicBox(
    const std::string& suffix,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  auto box = dart::dynamics::Skeleton::create(std::string(kBoxPrefix) + suffix);
  auto [joint, body]
      = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  joint->setTransformFromParentBodyNode(transform);

  auto shape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);
  body->setInertia(
      dart::dynamics::Inertia(
          1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));
  return box;
}

dart::dynamics::SkeletonPtr createDynamicSphere()
{
  auto sphere = dart::dynamics::Skeleton::create(kSphereName);
  auto [joint, body]
      = sphere->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, 0.0, 2.5);
  joint->setTransformFromParentBodyNode(transform);

  auto shape = std::make_shared<dart::dynamics::SphereShape>(0.3);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.20, 0.20, 0.80));
  body->setInertia(
      dart::dynamics::Inertia(
          1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));
  return sphere;
}

SensorMarker createSensorMarker(
    const std::string& name,
    dart::dynamics::Frame* parent,
    const Eigen::Isometry3d& relativeTransform,
    double radius,
    const Eigen::Vector4d& color)
{
  auto marker = dart::dynamics::SimpleFrame::createShared(
      parent == nullptr ? dart::dynamics::Frame::World() : parent,
      name,
      relativeTransform);
  marker->setShape(std::make_shared<dart::dynamics::SphereShape>(radius));
  auto* visual = marker->getVisualAspect(true);
  visual->setRGBA(color);
  return SensorMarker{std::move(marker), visual};
}

ArrowVisual createArrowVisual(
    const std::string& name, const Eigen::Vector4d& color)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(4.0f);
  for (std::size_t i = 0; i < 6; ++i) {
    shape->addVertex(Eigen::Vector3d::Zero());
  }
  shape->addConnection(0, 1);
  shape->addConnection(1, 2);
  shape->addConnection(1, 3);
  shape->addConnection(1, 4);
  shape->addConnection(1, 5);
  frame->setShape(shape);
  auto* visual = frame->getVisualAspect(true);
  visual->setRGBA(color);
  visual->hide();
  return ArrowVisual{std::move(frame), std::move(shape), visual};
}

void updateArrowShape(
    ArrowVisual& arrow,
    const dart::dynamics::BodyNode* body,
    const Eigen::Vector3d& vector,
    bool visible)
{
  if (arrow.visual == nullptr || arrow.shape == nullptr || body == nullptr
      || !visible || vector.norm() < 1e-9) {
    if (arrow.visual != nullptr) {
      arrow.visual->hide();
    }
    return;
  }

  const Eigen::Vector3d start = body->getCOM();
  const Eigen::Vector3d direction = vector.normalized();
  const Eigen::Vector3d end = start + 0.55 * direction;
  Eigen::Vector3d side = direction.cross(Eigen::Vector3d::UnitZ());
  if (side.squaredNorm() < 1e-8) {
    side = direction.cross(Eigen::Vector3d::UnitX());
  }
  side.normalize();
  const Eigen::Vector3d secondSide = direction.cross(side).normalized();
  const Eigen::Vector3d headBase = end - 0.14 * direction;

  arrow.shape->setVertex(0, start);
  arrow.shape->setVertex(1, end);
  arrow.shape->setVertex(2, headBase + 0.10 * side);
  arrow.shape->setVertex(3, headBase - 0.10 * side);
  arrow.shape->setVertex(4, headBase + 0.10 * secondSide);
  arrow.shape->setVertex(5, headBase - 0.10 * secondSide);
  arrow.visual->show();
}

dart::simulation::WorldPtr createSimulationEventHandlerWorld()
{
  auto world
      = dart::simulation::World::create("dartsim_simulation_event_handler");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(kDefaultTimeStep);
  world->addSkeleton(createGround());

  auto sensorCarrier = createDynamicBox(
      "sensor_carrier",
      Eigen::Vector3d(0.4, 0.4, 0.4),
      Eigen::Vector3d(-1.0, 0.0, 2.0),
      Eigen::Vector3d(0.80, 0.20, 0.20));
  auto* sensorParent = sensorCarrier->getBodyNode(0);
  world->addSkeleton(sensorCarrier);

  world->addSkeleton(createDynamicBox(
      "green",
      Eigen::Vector3d(0.6, 0.3, 0.3),
      Eigen::Vector3d(0.0, 0.0, 3.0),
      Eigen::Vector3d(0.20, 0.80, 0.20)));
  world->addSkeleton(createDynamicBox(
      "yellow",
      Eigen::Vector3d(0.3, 0.5, 0.4),
      Eigen::Vector3d(0.5, 1.0, 2.2),
      Eigen::Vector3d(0.90, 0.90, 0.20)));
  world->addSkeleton(createDynamicSphere());

  Eigen::Isometry3d fastOffset = Eigen::Isometry3d::Identity();
  fastOffset.translation() = Eigen::Vector3d(0.2, 0.0, 0.35);
  Eigen::Isometry3d slowOffset = Eigen::Isometry3d::Identity();
  slowOffset.translation() = Eigen::Vector3d(-0.2, 0.0, 0.35);

  const Eigen::Vector4d fastInactive(0.10, 0.80, 0.20, 0.25);
  const Eigen::Vector4d fastActive(0.10, 0.95, 0.25, 0.90);
  const Eigen::Vector4d slowInactive(0.95, 0.45, 0.10, 0.25);
  const Eigen::Vector4d slowActive(1.00, 0.65, 0.15, 0.90);
  auto fastMarker = createSensorMarker(
      kFastSensorFrameName, sensorParent, fastOffset, 0.08, fastInactive);
  auto slowMarker = createSensorMarker(
      kSlowSensorFrameName, sensorParent, slowOffset, 0.10, slowInactive);

  dart::sensor::Sensor::Properties fastProps;
  fastProps.name = "fast_sensor";
  fastProps.updateRate = 30.0;
  fastProps.relativeTransform = fastOffset;
  auto fastSensor = std::make_shared<BlinkingMarkerSensor>(
      fastProps, fastMarker, fastActive, fastInactive);
  fastSensor->setParentFrame(sensorParent);

  dart::sensor::Sensor::Properties slowProps;
  slowProps.name = "slow_sensor";
  slowProps.updateRate = 5.0;
  slowProps.relativeTransform = slowOffset;
  auto slowSensor = std::make_shared<BlinkingMarkerSensor>(
      slowProps, slowMarker, slowActive, slowInactive);
  slowSensor->setParentFrame(sensorParent);

  world->addSensor(fastSensor);
  world->addSensor(slowSensor);
  world->addSimpleFrame(fastMarker.frame);
  world->addSimpleFrame(slowMarker.frame);

  return world;
}

class SimulationEventState
{
public:
  explicit SimulationEventState(dart::simulation::WorldPtr world)
    : mWorld(std::move(world)),
      mForceMagnitude(kDefaultForceMagnitude),
      mTorqueMagnitude(kDefaultTorqueMagnitude),
      mTimeStep(kDefaultTimeStep)
  {
    if (mWorld == nullptr) {
      return;
    }

    mForceArrow = createArrowVisual(
        kForceArrowFrameName, Eigen::Vector4d(1.0, 0.05, 0.05, 1.0));
    mTorqueArrow = createArrowVisual(
        kTorqueArrowFrameName, Eigen::Vector4d(0.95, 0.20, 1.0, 1.0));
    mWorld->addSimpleFrame(mForceArrow.frame);
    mWorld->addSimpleFrame(mTorqueArrow.frame);
    collectRigidBodies();
    storeInitialState();
    printInstructions();
    printSelectedBody();
  }

  const dart::simulation::WorldPtr& world() const
  {
    return mWorld;
  }

  std::string selectedBodyLabel() const
  {
    const auto* body = selectedBody();
    if (body == nullptr) {
      return "none";
    }

    return body->getSkeleton()->getName() + "::" + body->getName() + " ("
           + std::to_string(mSelectedBodyIndex + 1) + "/"
           + std::to_string(mRigidBodies.size()) + ")";
  }

  double forceMagnitude() const
  {
    return mForceMagnitude;
  }

  double torqueMagnitude() const
  {
    return mTorqueMagnitude;
  }

  double timeStep() const
  {
    return mTimeStep;
  }

  bool showForceArrows() const
  {
    return mShowForceArrows;
  }

  void setShowForceArrows(bool show)
  {
    mShowForceArrows = show;
    updateForceArrows();
  }

  void beforeStep()
  {
    updateForceArrows();
  }

  void selectNextBody()
  {
    if (mRigidBodies.empty()) {
      return;
    }

    mSelectedBodyIndex = (mSelectedBodyIndex + 1) % mRigidBodies.size();
    printSelectedBody();
  }

  void selectPreviousBody()
  {
    if (mRigidBodies.empty()) {
      return;
    }

    if (mSelectedBodyIndex == 0) {
      mSelectedBodyIndex = mRigidBodies.size() - 1;
    } else {
      --mSelectedBodyIndex;
    }
    printSelectedBody();
  }

  void applyForce(const Eigen::Vector3d& force)
  {
    auto* body = selectedBody();
    if (body == nullptr) {
      std::cout << "No body selected!" << std::endl;
      return;
    }

    body->addExtForce(force);
    mLastForceBody = body;
    mLastForce = force;
    updateForceArrows();
    std::cout << "Applied force [" << force.transpose()
              << "] to body: " << body->getName() << std::endl;
  }

  void applyTorque(const Eigen::Vector3d& torque)
  {
    auto* body = selectedBody();
    if (body == nullptr) {
      std::cout << "No body selected!" << std::endl;
      return;
    }

    body->addExtTorque(torque);
    mLastTorqueBody = body;
    mLastTorque = torque;
    updateForceArrows();
    std::cout << "Applied torque [" << torque.transpose()
              << "] to body: " << body->getName() << std::endl;
  }

  void scaleMagnitudes(double scale)
  {
    mForceMagnitude *= scale;
    mTorqueMagnitude *= scale;
    std::cout << "Force magnitude: " << mForceMagnitude
              << ", Torque magnitude: " << mTorqueMagnitude << std::endl;
  }

  void scaleTimeStep(double scale)
  {
    if (mWorld == nullptr) {
      return;
    }

    mTimeStep *= scale;
    mWorld->setTimeStep(mTimeStep);
    std::cout << "Time step: " << mTimeStep << " seconds" << std::endl;
  }

  void toggleForceArrows()
  {
    setShowForceArrows(!mShowForceArrows);
    std::cout << "Force arrows: " << (mShowForceArrows ? "ON" : "OFF")
              << std::endl;
  }

  void resetSimulation()
  {
    if (mWorld == nullptr) {
      return;
    }

    mWorld->reset();
    const std::size_t numSkeletons = std::min<std::size_t>(
        mWorld->getNumSkeletons(), mInitialPositions.size());
    for (std::size_t i = 0; i < numSkeletons; ++i) {
      auto skeleton = mWorld->getSkeleton(i);
      if (skeleton == nullptr) {
        continue;
      }

      skeleton->setPositions(mInitialPositions[i]);
      skeleton->setVelocities(mInitialVelocities[i]);
      skeleton->clearExternalForces();
    }
    mWorld->setTimeStep(mTimeStep);
    mLastForce.setZero();
    mLastTorque.setZero();
    mLastForceBody = nullptr;
    mLastTorqueBody = nullptr;
    updateForceArrows();
    std::cout << "Simulation reset" << std::endl;
  }

  void printSimulationState() const
  {
    if (mWorld == nullptr) {
      return;
    }

    std::cout << "\n=== Simulation State ===" << std::endl;
    std::cout << "Time: " << std::fixed << std::setprecision(4)
              << mWorld->getTime() << " seconds" << std::endl;
    std::cout << "Time step: " << mTimeStep << " seconds" << std::endl;
    std::cout << "Force magnitude: " << mForceMagnitude << " N" << std::endl;
    std::cout << "Torque magnitude: " << mTorqueMagnitude << " Nm" << std::endl;
    std::cout << "Show arrows: " << (mShowForceArrows ? "YES" : "NO")
              << std::endl;
    std::cout << "Selected body: " << selectedBodyLabel() << std::endl;
    if (const auto* body = selectedBody()) {
      std::cout << "  Position: [" << body->getCOM().transpose() << "]"
                << std::endl;
      std::cout << "  Velocity: [" << body->getCOMLinearVelocity().transpose()
                << "]" << std::endl;
    }
    std::cout << "Number of rigid bodies: " << mRigidBodies.size() << std::endl;
    std::cout << "========================\n" << std::endl;
  }

  void printInstructions() const
  {
    std::cout << "\n=== Simulation Event Handler Controls ===" << std::endl;
    std::cout << "Simulation Control:" << std::endl;
    std::cout << "  Space      - Toggle simulation play/pause" << std::endl;
    std::cout << "  S          - Step simulation one frame" << std::endl;
    std::cout << "  R          - Reset simulation" << std::endl;
    std::cout << "\nBody Selection:" << std::endl;
    std::cout << "  Tab        - Select next rigid body" << std::endl;
    std::cout << "  Backspace  - Select previous rigid body" << std::endl;
    std::cout << "\nForce Application (on selected body):" << std::endl;
    std::cout << "  Arrow Keys - Apply force in X/Y directions" << std::endl;
    std::cout << "  U/D        - Apply upward/downward force" << std::endl;
    std::cout << "\nTorque Application (on selected body):" << std::endl;
    std::cout << "  Q/A        - Apply torque around X axis (+/-)" << std::endl;
    std::cout << "  W/Z        - Apply torque around Y axis (+/-)" << std::endl;
    std::cout << "  E/C        - Apply torque around Z axis (+/-)" << std::endl;
    std::cout << "\nMagnitude Adjustment:" << std::endl;
    std::cout << "  +/=        - Increase force/torque magnitude" << std::endl;
    std::cout << "  -/_        - Decrease force/torque magnitude" << std::endl;
    std::cout << "\nTime Step Adjustment:" << std::endl;
    std::cout << "  >/.        - Increase simulation time step" << std::endl;
    std::cout << "  </,        - Decrease simulation time step" << std::endl;
    std::cout << "\nVisualization:" << std::endl;
    std::cout << "  V          - Toggle force arrow visualization" << std::endl;
    std::cout << "\nInformation:" << std::endl;
    std::cout << "  I          - Print current simulation state" << std::endl;
    std::cout << "  H/?        - Show this help" << std::endl;
    std::cout << "==========================================\n" << std::endl;
  }

private:
  dart::dynamics::BodyNode* selectedBody() const
  {
    if (mSelectedBodyIndex >= mRigidBodies.size()) {
      return nullptr;
    }

    return mRigidBodies[mSelectedBodyIndex];
  }

  void printSelectedBody() const
  {
    if (const auto* body = selectedBody()) {
      std::cout << "Selected body: " << body->getName() << " ("
                << (mSelectedBodyIndex + 1) << "/" << mRigidBodies.size() << ")"
                << std::endl;
    }
  }

  void collectRigidBodies()
  {
    mRigidBodies.clear();
    if (mWorld == nullptr) {
      return;
    }

    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
      auto skeleton = mWorld->getSkeleton(i);
      if (skeleton == nullptr) {
        continue;
      }

      for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
        auto* body = skeleton->getBodyNode(j);
        if (body == nullptr) {
          continue;
        }

        const auto* joint = body->getParentJoint();
        if (joint != nullptr && joint->getNumDofs() > 0) {
          mRigidBodies.push_back(body);
        }
      }
    }
  }

  void storeInitialState()
  {
    mInitialPositions.clear();
    mInitialVelocities.clear();
    if (mWorld == nullptr) {
      return;
    }

    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
      auto skeleton = mWorld->getSkeleton(i);
      if (skeleton == nullptr) {
        continue;
      }

      mInitialPositions.push_back(skeleton->getPositions());
      mInitialVelocities.push_back(skeleton->getVelocities());
    }
  }

  void updateForceArrows()
  {
    updateArrowShape(mForceArrow, mLastForceBody, mLastForce, mShowForceArrows);
    updateArrowShape(
        mTorqueArrow, mLastTorqueBody, mLastTorque, mShowForceArrows);
  }

  dart::simulation::WorldPtr mWorld;
  std::vector<dart::dynamics::BodyNode*> mRigidBodies;
  std::size_t mSelectedBodyIndex = 0;
  double mForceMagnitude = kDefaultForceMagnitude;
  double mTorqueMagnitude = kDefaultTorqueMagnitude;
  double mTimeStep = kDefaultTimeStep;
  bool mShowForceArrows = true;
  Eigen::Vector3d mLastForce = Eigen::Vector3d::Zero();
  Eigen::Vector3d mLastTorque = Eigen::Vector3d::Zero();
  dart::dynamics::BodyNode* mLastForceBody = nullptr;
  dart::dynamics::BodyNode* mLastTorqueBody = nullptr;
  ArrowVisual mForceArrow;
  ArrowVisual mTorqueArrow;
  std::vector<Eigen::VectorXd> mInitialPositions;
  std::vector<Eigen::VectorXd> mInitialVelocities;
};

dart::gui::Panel createSimulationEventHandlerPanel(
    const std::shared_ptr<SimulationEventState>& state)
{
  dart::gui::Panel panel;
  panel.title = "Simulation Events";
  panel.buildWithContext = [state](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("=== Simulation Event Handler Demo ===");
    builder.text("Comprehensive event handler replacing MyWindow.cpp behavior");
    builder.text("Blinking sensor markers update at 30 Hz and 5 Hz");
    builder.text("dynamic boxes and sphere exercise event-time updates");
    builder.text("Sensors: green=fast (30 Hz), orange=slow (5 Hz)");
    builder.text("Press H for detailed controls");
    builder.text("Press Space to start/pause simulation");
    builder.text("Use Tab/Backspace to select rigid bodies");
    builder.text("Use arrow keys and U/D to apply forces");
    builder.text("Use Q/W/E/A/Z/C for torques");
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
    builder.sameLine();
    if (builder.button("Reset")) {
      state->resetSimulation();
    }
    builder.separator();
    builder.text("selected: " + state->selectedBodyLabel());
    if (builder.button("Next Body")) {
      state->selectNextBody();
    }
    builder.sameLine();
    if (builder.button("Previous Body")) {
      state->selectPreviousBody();
    }
    bool showArrows = state->showForceArrows();
    if (builder.checkbox("Force arrows", showArrows)) {
      state->setShowForceArrows(showArrows);
    }
    builder.text("force magnitude: " + std::to_string(state->forceMagnitude()));
    builder.text(
        "torque magnitude: " + std::to_string(state->torqueMagnitude()));
    builder.text("time step: " + std::to_string(state->timeStep()));
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

dart::gui::KeyboardAction makeSimulationEventAction(
    std::string label,
    dart::gui::KeyboardShortcut shortcut,
    std::function<void(dart::gui::KeyboardActionContext&)> callback,
    bool repeat = false)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = shortcut;
  action.callback = std::move(callback);
  action.repeat = repeat;
  return action;
}

std::vector<dart::gui::KeyboardAction> createSimulationEventKeyboardActions(
    const std::shared_ptr<SimulationEventState>& state)
{
  std::vector<dart::gui::KeyboardAction> actions;
  const auto character = dart::gui::KeyboardShortcut::characterKey;
  const auto named = dart::gui::KeyboardShortcut::namedKey;

  actions.push_back(makeSimulationEventAction(
      "Step simulation",
      character('s'),
      [](dart::gui::KeyboardActionContext& context) {
        if (context.lifecycle != nullptr) {
          dart::gui::requestSingleStep(*context.lifecycle);
        }
      }));
  actions.push_back(makeSimulationEventAction(
      "Reset simulation",
      character('r'),
      [state](dart::gui::KeyboardActionContext&) {
        state->resetSimulation();
      }));
  actions.push_back(makeSimulationEventAction(
      "Select next body",
      named(dart::gui::KeyboardKey::Tab),
      [state](dart::gui::KeyboardActionContext&) { state->selectNextBody(); }));
  actions.push_back(makeSimulationEventAction(
      "Select previous body",
      named(dart::gui::KeyboardKey::Backspace),
      [state](dart::gui::KeyboardActionContext&) {
        state->selectPreviousBody();
      }));

  const auto addForceAction = [&](std::string label,
                                  dart::gui::KeyboardShortcut shortcut,
                                  const Eigen::Vector3d& direction) {
    actions.push_back(makeSimulationEventAction(
        std::move(label),
        shortcut,
        [state, direction](dart::gui::KeyboardActionContext&) {
          state->applyForce(state->forceMagnitude() * direction);
        },
        true));
  };
  addForceAction(
      "Apply +Y force",
      named(dart::gui::KeyboardKey::Up),
      Eigen::Vector3d::UnitY());
  addForceAction(
      "Apply -Y force",
      named(dart::gui::KeyboardKey::Down),
      -Eigen::Vector3d::UnitY());
  addForceAction(
      "Apply -X force",
      named(dart::gui::KeyboardKey::Left),
      -Eigen::Vector3d::UnitX());
  addForceAction(
      "Apply +X force",
      named(dart::gui::KeyboardKey::Right),
      Eigen::Vector3d::UnitX());
  addForceAction("Apply +Z force", character('u'), Eigen::Vector3d::UnitZ());
  addForceAction("Apply -Z force", character('d'), -Eigen::Vector3d::UnitZ());

  const auto addTorqueAction
      = [&](std::string label, char key, const Eigen::Vector3d& axis) {
          actions.push_back(makeSimulationEventAction(
              std::move(label),
              character(key),
              [state, axis](dart::gui::KeyboardActionContext&) {
                state->applyTorque(state->torqueMagnitude() * axis);
              },
              true));
        };
  addTorqueAction("Apply +X torque", 'q', Eigen::Vector3d::UnitX());
  addTorqueAction("Apply +Y torque", 'w', Eigen::Vector3d::UnitY());
  addTorqueAction("Apply +Z torque", 'e', Eigen::Vector3d::UnitZ());
  addTorqueAction("Apply -X torque", 'a', -Eigen::Vector3d::UnitX());
  addTorqueAction("Apply -Y torque", 'z', -Eigen::Vector3d::UnitY());
  addTorqueAction("Apply -Z torque", 'c', -Eigen::Vector3d::UnitZ());

  actions.push_back(makeSimulationEventAction(
      "Increase force and torque magnitude",
      character('='),
      [state](dart::gui::KeyboardActionContext&) {
        state->scaleMagnitudes(1.5);
      }));
  actions.push_back(makeSimulationEventAction(
      "Decrease force and torque magnitude",
      character('-'),
      [state](dart::gui::KeyboardActionContext&) {
        state->scaleMagnitudes(0.67);
      }));
  actions.push_back(makeSimulationEventAction(
      "Increase simulation timestep",
      character('.'),
      [state](dart::gui::KeyboardActionContext&) {
        state->scaleTimeStep(1.5);
      }));
  actions.push_back(makeSimulationEventAction(
      "Decrease simulation timestep",
      character(','),
      [state](dart::gui::KeyboardActionContext&) {
        state->scaleTimeStep(0.67);
      }));
  actions.push_back(makeSimulationEventAction(
      "Toggle force arrow visualization",
      character('v'),
      [state](dart::gui::KeyboardActionContext&) {
        state->toggleForceArrows();
      }));
  actions.push_back(makeSimulationEventAction(
      "Print simulation state",
      character('i'),
      [state](dart::gui::KeyboardActionContext&) {
        state->printSimulationState();
      }));
  actions.push_back(makeSimulationEventAction(
      "Print controls",
      character('h'),
      [state](dart::gui::KeyboardActionContext&) {
        state->printInstructions();
      }));
  actions.push_back(makeSimulationEventAction(
      "Print controls (?)",
      character('?'),
      [state](dart::gui::KeyboardActionContext&) {
        state->printInstructions();
      }));

  return actions;
}

dart::gui::RunOptions makeSimulationEventRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 960;
  return options;
}

dart::gui::OrbitCamera makeSimulationEventCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 1.0);
  camera.yaw = 0.7853981633974483;
  camera.pitch = 0.2756427992162654;
  camera.distance = 7.3484692283495345;
  return camera;
}

} // namespace

int main(int argc, char* argv[])
{
  auto world = createSimulationEventHandlerWorld();
  auto state = std::make_shared<SimulationEventState>(world);

  dart::gui::ApplicationOptions options;
  options.world = state->world();
  options.runDefaults = makeSimulationEventRunDefaults();
  options.camera = makeSimulationEventCamera();
  options.preStep = [state]() {
    state->beforeStep();
  };
  options.panels.push_back(createSimulationEventHandlerPanel(state));
  options.keyboardActions = createSimulationEventKeyboardActions(state);
  return dart::gui::runApplication(argc, argv, options);
}
