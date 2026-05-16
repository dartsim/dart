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

#include <dart/constraint/constraint_solver.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/lcp/pivoting/dantzig_solver.hpp>

#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace {

constexpr const char* kGroundName = "lcp_physics_ground";
constexpr const char* kBoxPrefix = "lcp_physics_contact_box_";
constexpr const char* kSpherePrefix = "lcp_physics_ball_drop_sphere_";
constexpr int kSphereCount = 12;

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  constexpr double thickness = 0.08;
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(7.0, thickness, 4.5)));
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.72, 0.74, 0.72));
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().y() = -0.5 * thickness;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);
  return ground;
}

dart::dynamics::SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color)
{
  auto box = dart::dynamics::Skeleton::create(name);
  auto [joint, body]
      = box->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  joint->setTransformFromParentBodyNode(transform);

  auto boxShape = std::make_shared<dart::dynamics::BoxShape>(size);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.3);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);
  shapeNode->getDynamicsAspect()->setPrimarySlipCompliance(0.0);
  shapeNode->getDynamicsAspect()->setSecondarySlipCompliance(0.0);
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), boxShape->computeInertia(mass)));
  return box;
}

dart::dynamics::SkeletonPtr createTranslatedBox(
    const std::string& suffix,
    const Eigen::Vector3d& size,
    double mass,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  return createBox(
      std::string(kBoxPrefix) + suffix, size, mass, transform, color);
}

dart::dynamics::SkeletonPtr createSphere(
    const std::string& suffix,
    double radius,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double mass)
{
  auto sphere
      = dart::dynamics::Skeleton::create(std::string(kSpherePrefix) + suffix);
  auto [joint, body]
      = sphere->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = position;
  joint->setTransformFromParentBodyNode(transform);

  auto sphereShape = std::make_shared<dart::dynamics::SphereShape>(radius);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(sphereShape);
  shapeNode->getVisualAspect()->setColor(color);
  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), sphereShape->computeInertia(mass)));
  return sphere;
}

dart::simulation::WorldPtr createLcpPhysicsWorld()
{
  auto world = dart::simulation::World::create("dartsim_lcp_physics");
  world->setTimeStep(0.001);
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  if (auto* solver = world->getConstraintSolver()) {
    solver->setLcpSolver(std::make_shared<dart::math::DantzigSolver>());
  }

  world->addSkeleton(createGround());

  constexpr double massBoxSize = 0.36;
  world->addSkeleton(createTranslatedBox(
      "light_mass",
      Eigen::Vector3d::Constant(massBoxSize),
      1.0,
      Eigen::Vector3d(-2.0, 0.5 * massBoxSize, -0.85),
      Eigen::Vector3d(0.30, 0.72, 0.36)));
  world->addSkeleton(createTranslatedBox(
      "heavy_mass",
      Eigen::Vector3d::Constant(massBoxSize),
      1000.0,
      Eigen::Vector3d(-2.0, 1.5 * massBoxSize, -0.85),
      Eigen::Vector3d(0.84, 0.24, 0.20)));

  constexpr int pyramidLayers = 4;
  constexpr double pyramidBoxSize = 0.28;
  int pyramidBoxIndex = 0;
  for (int layer = 0; layer < pyramidLayers; ++layer) {
    const int boxesInLayer = pyramidLayers - layer;
    const double y = 0.5 * pyramidBoxSize + layer * pyramidBoxSize * 1.05;
    const double startX = -0.5 * (boxesInLayer - 1) * pyramidBoxSize * 1.12;
    for (int i = 0; i < boxesInLayer; ++i) {
      const double x = startX + i * pyramidBoxSize * 1.12;
      const double t
          = static_cast<double>(pyramidBoxIndex)
            / static_cast<double>(pyramidLayers * (pyramidLayers + 1) / 2);
      world->addSkeleton(createTranslatedBox(
          "stack_" + std::to_string(pyramidBoxIndex),
          Eigen::Vector3d::Constant(pyramidBoxSize),
          1.0,
          Eigen::Vector3d(x, y, -0.55),
          Eigen::Vector3d(0.28 + 0.42 * t, 0.52, 0.82 - 0.34 * t)));
      ++pyramidBoxIndex;
    }
  }

  constexpr int dominoCount = 8;
  constexpr double dominoSpacing = 0.16;
  for (int i = 0; i < dominoCount; ++i) {
    const double x
        = (static_cast<double>(i) - 0.5 * (dominoCount - 1)) * dominoSpacing;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(x, 0.18, 0.85);
    if (i == 0) {
      transform.rotate(Eigen::AngleAxisd(0.30, Eigen::Vector3d::UnitZ()));
    }
    const double t = static_cast<double>(i) / static_cast<double>(dominoCount);
    world->addSkeleton(createBox(
        std::string(kBoxPrefix) + "domino_" + std::to_string(i),
        Eigen::Vector3d(0.055, 0.32, 0.15),
        0.5,
        transform,
        Eigen::Vector3d(0.18 + 0.52 * t, 0.34, 0.84 - 0.42 * t)));
  }

  int sphereIndex = 0;
  for (int xIndex = 0; xIndex < 4; ++xIndex) {
    for (int zIndex = 0; zIndex < 3; ++zIndex) {
      const double t = static_cast<double>(sphereIndex)
                       / static_cast<double>(kSphereCount);
      world->addSkeleton(createSphere(
          std::to_string(sphereIndex),
          0.11,
          Eigen::Vector3d(
              1.45 + (static_cast<double>(xIndex) - 1.5) * 0.28,
              0.65 + 0.10 * static_cast<double>(zIndex),
              -0.55 + (static_cast<double>(zIndex) - 1.0) * 0.28),
          Eigen::Vector3d(0.86 - 0.35 * t, 0.42 + 0.35 * t, 0.36),
          0.5));
      ++sphereIndex;
    }
  }

  return world;
}

dart::gui::Panel createLcpPanel()
{
  dart::gui::Panel panel;
  panel.title = "LCP Physics";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("Dantzig contact solver benchmark scene");
    builder.text("Mass-ratio, box-stack, domino, and ball-drop cases");
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
  options.world = createLcpPhysicsWorld();
  options.panels.push_back(createLcpPanel());
  return dart::gui::runApplication(argc, argv, options);
}
