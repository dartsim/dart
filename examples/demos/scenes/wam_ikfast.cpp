/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

// Static WAM-7 visualization. The standalone wam_ikfast example wired an
// analytical IKFast solver into a draggable target frame; that solver is
// loaded at runtime via dart::dynamics::SharedLibraryIkFast (dlopen), so the
// scene is now available unconditionally — if the wamIk shared library is
// found at runtime the analytical IK is wired, otherwise the scene falls back
// to the default numerical (Jacobian-pseudoinverse) IK and renders the arm at
// its zero pose. Either way the scene loads.

#include "scenes.hpp"

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/end_effector.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/inverse_kinematics.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/shared_library_ik_fast.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/common/uri.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kWamPackageName = "herb_description";
constexpr const char* kWamPackagePath = "urdf/wam";
constexpr const char* kWamUrdfPath = "urdf/wam/wam.urdf";
constexpr const char* kWamSkeletonName = "visual_wam_ikfast_robot";
constexpr const char* kEndEffectorName = "ee";
constexpr const char* kTargetFrameName = "lh_target";
constexpr const char* kGroundSkeletonName = "visual_wam_ikfast_ground";

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

dart::dynamics::SkeletonPtr loadWamSkeleton()
{
  dart::io::ReadOptions options;
  options.addPackageDirectory(
      kWamPackageName, dart::config::dataPath(kWamPackagePath));
  const auto wamUri
      = dart::common::Uri::createFromPath(dart::config::dataPath(kWamUrdfPath));
  auto wam = dart::io::readSkeleton(wamUri, options);
  if (wam == nullptr) {
    throw std::runtime_error(
        "Failed to load WAM IKFast robot from " + wamUri.toString());
  }

  wam->setName(kWamSkeletonName);
  for (const char* jointName :
       {"/j1", "/j2", "/j3", "/j4", "/j5", "/j6", "/j7"}) {
    auto* dof = wam->getDof(jointName);
    if (dof == nullptr) {
      throw std::runtime_error(
          "WAM IKFast robot is missing expected DOF " + std::string(jointName));
    }
    dof->setPosition(0.0);
  }

  disableSkeletonCollisionAndGravity(wam);
  return wam;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::dynamics::Skeleton::create(kGroundSkeletonName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(10.0, 10.0, 0.01)));
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.005));
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.18, 0.32, 0.58, 1.0));
  return ground;
}

std::string defaultIkFastLibraryPath()
{
  std::stringstream stream;
  stream << DART_SHARED_LIB_PREFIX << "wamIk";
#if (DART_OS_LINUX || DART_OS_MACOS) && !defined(NDEBUG)
  stream << "d";
#endif
  stream << "." << DART_SHARED_LIB_EXTENSION;
  return stream.str();
}

bool tryAttachIkFast(
    const dart::dynamics::InverseKinematicsPtr& ik,
    const std::string& libraryPath)
{
  if (libraryPath.empty() || ik == nullptr) {
    return false;
  }
  try {
    const std::vector<std::size_t> dofs{0, 1, 2, 3, 4, 5, 6};
    const std::vector<std::size_t> freeDofs;
    auto solver = ik->setGradientMethod<
        dart::dynamics::SharedLibraryIkFast>(libraryPath, dofs, freeDofs);
    (void)solver;
    return true;
  } catch (const std::exception& e) {
    std::cerr << "wam_ikfast: analytical IKFast unavailable (" << e.what()
              << "); falling back to numerical IK\n";
    return false;
  }
}

struct WamSetup
{
  dart::dynamics::SkeletonPtr wam;
  dart::dynamics::SimpleFramePtr target;
  bool ikfastAttached = false;
};

WamSetup createWamSetup()
{
  WamSetup setup;
  setup.wam = loadWamSkeleton();
  auto* endEffectorBody = setup.wam->getBodyNode("/wam7");
  if (endEffectorBody == nullptr) {
    throw std::runtime_error("WAM IKFast robot is missing /wam7 body node");
  }

  Eigen::Isometry3d handOffset = Eigen::Isometry3d::Identity();
  handOffset.translate(Eigen::Vector3d(0.0, 0.0, -0.09));
  auto* effector = endEffectorBody->createEndEffector(kEndEffectorName);
  effector->setDefaultRelativeTransform(handOffset, true);

  setup.target = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(),
      kTargetFrameName,
      effector->getWorldTransform());
  setup.target->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.04)));
  setup.target->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.90, 0.45, 0.20, 0.85));

  const auto ik = effector->getIK(true);
  ik->getErrorMethod().setLinearBounds(
      -Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
       Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()));
  ik->getErrorMethod().setAngularBounds(
      -Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),
       Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()));
  setup.ikfastAttached = tryAttachIkFast(ik, defaultIkFastLibraryPath());
  return setup;
}

dart::gui::Panel makePanel(const std::shared_ptr<WamSetup>& setup)
{
  dart::gui::Panel panel;
  panel.title = "WAM IKFast";
  panel.buildWithContext = [setup](
                              dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext&) {
    builder.text("Barrett WAM-7 arm with a tracked target frame.");
    builder.text(
        setup->ikfastAttached
            ? std::string("IK solver: analytical (IKFast, runtime-loaded)")
            : std::string("IK solver: numerical (IKFast lib not found)"));
  };
  return panel;
}

dart::gui::OrbitCamera makeCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.5);
  camera.distance = 2.2;
  camera.yaw = 0.6;
  camera.pitch = 0.3;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeWamIkFastScene()
{
  auto setup = std::make_shared<WamSetup>(createWamSetup());

  auto world = dart::simulation::World::create("dartsim_wam_ikfast");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createGround());
  world->addSkeleton(setup->wam);
  world->addSimpleFrame(setup->target);

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.simulateWorld = false;
  options.camera = makeCamera();
  options.panels.push_back(makePanel(setup));
  return options;
}

} // namespace dart::examples::demos
