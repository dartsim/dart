#include "DoublePendulumScene.hpp"

#include "../core/WorkbenchConstants.hpp"

#include <dart/all.hpp>

#include <imgui.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <numeric>
#include <sstream>
#include <vector>

namespace workbench {

namespace {

struct PendulumState
{
  Eigen::Vector2d initialPositions{Eigen::Vector2d::Zero()};
  Eigen::Vector2d initialVelocities{Eigen::Vector2d::Zero()};
  float damping{0.01f};
  std::vector<float> energySamples;
};

} // namespace

ExampleRecord makeDoublePendulumScene()
{
  ExampleRecord record;
  record.id = "double_pendulum";
  record.name = "Double Pendulum";
  record.category = kInAppCategory;
  record.description
      = "Interactive planar double pendulum with adjustable damping and reset controls.";
  record.factory = [](WorkbenchServices&) -> ExampleSession {
    ExampleSession session;

    auto world = dart::simulation::World::create("DoublePendulum");
    world->setTimeStep(0.001);
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    auto pendulum = dart::dynamics::Skeleton::create("pendulum");

    dart::dynamics::RevoluteJoint::Properties shoulderProps;
    shoulderProps.mName = "shoulder";
    shoulderProps.mAxis = Eigen::Vector3d::UnitY();
    shoulderProps.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);
    shoulderProps.mT_ChildBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.25);

    dart::dynamics::BodyNode::Properties upperProps;
    upperProps.mName = "upper_link";
    auto [shoulder, upper]
        = pendulum->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
            nullptr, shoulderProps, upperProps);
    upper->setMass(1.0);

    auto upperShape = upper->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(0.05, 0.05, 0.5)));
    upperShape->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.25));
    upperShape->getVisualAspect()->setColor(Eigen::Vector3d(0.20, 0.55, 0.80));

    dart::dynamics::RevoluteJoint::Properties elbowProps;
    elbowProps.mName = "elbow";
    elbowProps.mAxis = Eigen::Vector3d::UnitY();
    elbowProps.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, -0.5);
    elbowProps.mT_ChildBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.25);

    dart::dynamics::BodyNode::Properties lowerProps;
    lowerProps.mName = "lower_link";
    auto [elbow, lower]
        = pendulum->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
            upper, elbowProps, lowerProps);
    lower->setMass(0.8);

    auto lowerShape = lower->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(0.05, 0.05, 0.5)));
    lowerShape->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.25));
    lowerShape->getVisualAspect()->setColor(Eigen::Vector3d(0.80, 0.35, 0.25));

    shoulder->setDampingCoefficient(0, 0.01);
    elbow->setDampingCoefficient(0, 0.01);

    shoulder->setPosition(0, dart::math::toRadian(60.0));
    elbow->setPosition(0, dart::math::toRadian(-20.0));

    world->addSkeleton(pendulum);

    auto floorSkeleton = dart::dynamics::Skeleton::create("ground");
    dart::dynamics::WeldJoint::Properties floorJointProps;
    floorJointProps.mName = "ground_joint";
    dart::dynamics::BodyNode::Properties floorBodyProps;
    floorBodyProps.mName = "ground_body";
    auto [floorJoint, floorBody]
        = floorSkeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
            nullptr, floorJointProps, floorBodyProps);
    (void)floorJoint;

    auto floorShapeNode = floorBody->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(8.0, 8.0, 0.02)));
    floorShapeNode->getVisualAspect()->setColor(Eigen::Vector3d::Constant(0.45).eval());
    Eigen::Isometry3d floorTransform = Eigen::Isometry3d::Identity();
    floorTransform.translate(Eigen::Vector3d(0.0, 0.0, -0.01));
    floorJoint->setTransformFromParentBodyNode(floorTransform);
    world->addSkeleton(floorSkeleton);

    auto state = std::make_shared<PendulumState>();
    state->initialPositions = Eigen::Vector2d(
        shoulder->getPosition(0), elbow->getPosition(0));

    session.world = world;
    session.onActivate = [state](WorkbenchServices& svc) {
      svc.log("Double pendulum ready. Adjust damping or reset from Properties.");
      state->energySamples.clear();
    };
    session.onDeactivate = [](WorkbenchServices& svc) {
      svc.log("Double pendulum scene unloaded.");
    };
    session.onStep = [pendulum, state](WorkbenchServices&, double) {
      const double potential = pendulum->getMass() * 9.81 * pendulum->getCOM()[2];
      const double energy = potential + pendulum->computeKineticEnergy();
      state->energySamples.push_back(static_cast<float>(energy));
      if (state->energySamples.size() > 300)
        state->energySamples.erase(state->energySamples.begin());
    };
    session.onOverviewUi = [state](WorkbenchServices&) {
      ImGui::Text("Energy samples: %zu", state->energySamples.size());
      if (!state->energySamples.empty()) {
        const float maxVal = *std::max_element(
            state->energySamples.begin(), state->energySamples.end());
        ImGui::PlotLines(
            "Total energy",
            state->energySamples.data(),
            static_cast<int>(state->energySamples.size()),
            0,
            nullptr,
            0.0f,
            maxVal > 0.0f ? maxVal : 1.0f,
            ImVec2(-1.0f, 80.0f));
      }
    };
    session.onPropertiesUi = [pendulum, state](WorkbenchServices& svc) {
      float damping = state->damping;
      if (ImGui::SliderFloat("Joint damping", &damping, 0.0f, 0.2f)) {
        state->damping = damping;
        pendulum->getJoint(0)->setDampingCoefficient(0, damping);
        pendulum->getJoint(1)->setDampingCoefficient(0, damping);
      }

      if (ImGui::Button("Reset pose")) {
        pendulum->setPositions(state->initialPositions);
        pendulum->setVelocities(state->initialVelocities);
        svc.log("Double pendulum reset to initial pose.");
      }
    };

    return session;
  };

  return record;
}

} // namespace workbench
