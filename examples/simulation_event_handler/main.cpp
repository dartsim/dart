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
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/sensor/sensor.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace {

constexpr const char* kGroundName = "simulation_event_handler_ground";
constexpr const char* kBoxPrefix = "simulation_event_handler_box_";
constexpr const char* kSphereName = "simulation_event_handler_sphere";
constexpr const char* kFastSensorFrameName
    = "simulation_event_handler_fast_sensor";
constexpr const char* kSlowSensorFrameName
    = "simulation_event_handler_slow_sensor";

struct SensorMarker
{
  std::shared_ptr<dart::dynamics::SimpleFrame> frame;
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

dart::simulation::WorldPtr createSimulationEventHandlerWorld()
{
  auto world
      = dart::simulation::World::create("dartsim_simulation_event_handler");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
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

dart::gui::Panel createSimulationEventHandlerPanel()
{
  dart::gui::Panel panel;
  panel.title = "Simulation Events";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Blinking sensor markers update at 30 Hz and 5 Hz");
    builder.text("dynamic boxes and sphere exercise event-time updates");
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
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createSimulationEventHandlerWorld();
  options.panels.push_back(createSimulationEventHandlerPanel());
  return dart::gui::runApplication(argc, argv, options);
}
