#include "BlockStackScene.hpp"

#include "../core/WorkbenchConstants.hpp"
#include "../core/WorkbenchWorldNode.hpp"

#include <dart/all.hpp>

#include <imgui.h>

#include <Eigen/Geometry>

#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

namespace workbench {

namespace {

dart::dynamics::SkeletonPtr spawnBox(
    const dart::simulation::WorldPtr& world,
    double height,
    int index)
{
  auto skeleton = dart::dynamics::Skeleton::create(
      "box_" + std::to_string(index));

  auto [joint, body]
      = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  body->setMass(1.0);
  auto boxShape = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.2)));
  boxShape->getVisualAspect()->setColor(
      Eigen::Vector3d::Random().cwiseAbs().eval());

  Eigen::Matrix<double, 6, 1> pose = Eigen::Matrix<double, 6, 1>::Zero();
  pose[4] = height;
  joint->setPositions(pose);

  world->addSkeleton(skeleton);
  return skeleton;
}

struct StackState
{
  std::vector<dart::dynamics::SkeletonPtr> boxes;
  double spawnHeight{0.6};
  double kickImpulse{20.0};
  bool queueKick{false};
};

} // namespace

ExampleRecord makeBlockStackScene()
{
  ExampleRecord record;
  record.id = "block_stack";
  record.name = "Block Stack";
  record.category = kInAppCategory;
  record.description
      = "Spawn and kick rigid boxes to test collision, stacking, and logging.";
  record.factory = [](WorkbenchServices&) -> ExampleSession {
    ExampleSession session;

    auto world = dart::simulation::World::create("BlockStack");
    world->setTimeStep(0.001);
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    auto floorSkeleton = dart::dynamics::Skeleton::create("floor");
    dart::dynamics::WeldJoint::Properties weldProps;
    weldProps.mName = "floor_joint";
    dart::dynamics::BodyNode::Properties floorBodyProps;
    floorBodyProps.mName = "floor_body";
    auto [floorJoint, floorBody]
        = floorSkeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
            nullptr, weldProps, floorBodyProps);
    (void)floorJoint;

    auto floorShape = floorBody->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(
        std::make_shared<dart::dynamics::BoxShape>(
            Eigen::Vector3d(6.0, 6.0, 0.1)));
    floorShape->getVisualAspect()->setColor(Eigen::Vector3d::Constant(0.3).eval());
    floorShape->getDynamicsAspect()->setFrictionCoeff(0.9);
    Eigen::Isometry3d floorOffset = Eigen::Isometry3d::Identity();
    floorOffset.translate(Eigen::Vector3d(0.0, 0.0, -0.05));
    floorJoint->setTransformFromParentBodyNode(floorOffset);
    world->addSkeleton(floorSkeleton);

    auto state = std::make_shared<StackState>();

    for (int i = 0; i < 3; ++i)
      state->boxes.push_back(spawnBox(world, 0.3 + i * 0.25, i));

    session.world = world;
    session.onActivate = [state](WorkbenchServices& svc) {
      svc.log("Block stack scene ready. Spawn boxes or kick them from Properties.");
      state->queueKick = false;
    };
    session.onStep = [state](WorkbenchServices&, double) {
      if (!state->queueKick)
        return;
      state->queueKick = false;
      for (auto& box : state->boxes) {
        if (!box)
          continue;
        auto body = box->getBodyNode(0);
        body->clearExternalForces();
        body->addExtForce(Eigen::Vector3d(state->kickImpulse, 0.0, 0.0));
      }
    };
    session.onPropertiesUi = [state](WorkbenchServices& svc) {
      ImGui::Text("Boxes: %zu", state->boxes.size());

      float spawnHeight = static_cast<float>(state->spawnHeight);
      if (ImGui::SliderFloat("Spawn height", &spawnHeight, 0.3f, 2.0f, "%.2f m")) {
        state->spawnHeight = spawnHeight;
      }

      if (ImGui::Button("Spawn box")) {
        const int index = static_cast<int>(state->boxes.size());
        auto world = svc.worldNodeRef().getWorld();
        if (world)
          state->boxes.push_back(spawnBox(world, state->spawnHeight, index));
        std::ostringstream oss;
        oss << "Spawned new box at height " << std::fixed << std::setprecision(2)
            << state->spawnHeight << " m";
        svc.log(oss.str());
      }

      ImGui::Spacing();

      float kick = static_cast<float>(state->kickImpulse);
      if (ImGui::SliderFloat("Kick impulse", &kick, 5.0f, 60.0f, "%.1f N"))
        state->kickImpulse = kick;

      if (ImGui::Button("Kick forward"))
        state->queueKick = true;
    };

    return session;
  };

  return record;
}

} // namespace workbench
