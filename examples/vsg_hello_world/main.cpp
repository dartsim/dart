/*
 * Minimal VulkanSceneGraph viewer example for DART.
 */

#include <dart/gui/vsg/Viewer.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/simulation/World.hpp>

#include <Eigen/Geometry>

#include <filesystem>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

using namespace dart;

namespace {

dynamics::SkeletonPtr createFloor()
{
  auto floor = dynamics::Skeleton::create("floor");
  auto [joint, body]
      = floor->createJointAndBodyNodePair<dynamics::WeldJoint, dynamics::BodyNode>();
  auto shape = std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d(5.0, 5.0, 0.2));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector4d(0.8, 0.8, 0.8, 1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -0.1);
  joint->setTransformFromParentBodyNode(tf);
  return floor;
}

dynamics::SkeletonPtr createBox()
{
  auto box = dynamics::Skeleton::create("box");
  auto [joint, body]
      = box->createJointAndBodyNodePair<dynamics::FreeJoint, dynamics::BodyNode>();
  auto shape
      = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Constant(0.2));
  auto* shapeNode = body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector4d(0.2, 0.4, 0.8, 1.0));
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(0.0, 0.0, 1.0);
  joint->setTransformFromParentBodyNode(tf);
  return box;
}

} // namespace

int main()
{
  const char* run_loop = std::getenv("DART_VSG_RUN");
  const bool should_run = run_loop && std::string_view(run_loop) == "1";
  if (!should_run) {
    std::cout
        << "[vsg_hello_world] Example built; skipping viewer startup. Set "
           "DART_VSG_RUN=1 to create a window. (Optional: set DART_VSG_FRAMES "
           "to a positive integer to run a fixed number of frames.)\n";
    return 0;
  }

  const char* display = std::getenv("DISPLAY");
  const char* wayland = std::getenv("WAYLAND_DISPLAY");
  if ((!display || display[0] == '\0') && (!wayland || wayland[0] == '\0')) {
    std::cerr
        << "[vsg_hello_world] No DISPLAY or WAYLAND_DISPLAY found; skipping "
           "run.\n";
    return 0;
  }

  bool has_icd = false;
  if (const char* icd_env = std::getenv("VK_ICD_FILENAMES")) {
    has_icd = icd_env[0] != '\0';
  } else {
    const std::vector<std::filesystem::path> icd_paths{
        (std::getenv("CONDA_PREFIX")
             ? std::filesystem::path(std::getenv("CONDA_PREFIX"))
                   / "share/vulkan/icd.d"
             : std::filesystem::path()),
        "/usr/share/vulkan/icd.d",
        "/etc/vulkan/icd.d",
        "/usr/local/share/vulkan/icd.d"};
    for (const auto& path : icd_paths) {
      if (path.empty()) {
        continue;
      }
      if (std::filesystem::exists(path) && !std::filesystem::is_empty(path)) {
        has_icd = true;
        break;
      }
    }
  }
  if (!has_icd) {
    std::cerr
        << "[vsg_hello_world] No Vulkan ICD detected; skipping run.\n";
    return 0;
  }

  auto world = std::make_shared<simulation::World>();
  world->setName("vsg-example");
  world->addSkeleton(createFloor());
  world->addSkeleton(createBox());

  try {
    gui::vsg::Viewer viewer(world);
    viewer.simulate(true);
    viewer.setNumStepsPerCycle(4);

    const char* frames_env = std::getenv("DART_VSG_FRAMES");
    const long frames = frames_env ? std::strtol(frames_env, nullptr, 10) : 0;
    if (frames > 0) {
      for (long i = 0; i < frames; ++i) {
        if (!viewer.step()) {
          break;
        }
      }
    } else {
      viewer.run();
    }
  } catch (const ::vsg::Exception& e) {
    std::cerr << "[vsg_hello_world] VSG error: " << e.message << "\n";
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "[vsg_hello_world] Failed to start VSG viewer: " << e.what()
              << "\n";
    return 1;
  }

  return 0;
}
