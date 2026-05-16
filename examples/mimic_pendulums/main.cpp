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
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

constexpr const char* kWorldUri
    = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";
constexpr const char* kVisualGroundName = "visual_mimic_pendulums_ground";
constexpr const char* kSkeletonPrefix = "visual_mimic_pendulum_";

dart::dynamics::SkeletonPtr createVisualGround()
{
  auto ground = dart::dynamics::Skeleton::create(kVisualGroundName);
  auto* body
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>().second;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 3.0, -0.02);
  auto* shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect>(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(2.2, 7.2, 0.04)));
  shapeNode->setRelativeTransform(transform);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.47, 0.50, 0.47));
  return ground;
}

dart::simulation::WorldPtr createMimicPendulumsWorld()
{
  auto world = dart::io::readWorld(kWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load mimic_pendulums world from " + std::string(kWorldUri));
  }
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

  auto ground = world->getSkeleton("ground_plane");
  if (ground == nullptr) {
    throw std::runtime_error("mimic_pendulums world is missing ground_plane");
  }
  ground->setName("visual_mimic_pendulums_imported_ground");
  world->addSkeleton(createVisualGround());

  const std::array<std::pair<const char*, Eigen::Vector3d>, 3> rigs{{
      {"pendulum_with_base", Eigen::Vector3d(0.62, 0.62, 0.62)},
      {"pendulum_with_base_mimic_slow_follows_fast",
       Eigen::Vector3d(0.90, 0.34, 0.34)},
      {"pendulum_with_base_mimic_fast_follows_slow",
       Eigen::Vector3d(0.28, 0.48, 0.92)},
  }};

  for (std::size_t i = 0; i < rigs.size(); ++i) {
    auto skeleton = world->getSkeleton(rigs[i].first);
    if (skeleton == nullptr) {
      throw std::runtime_error(
          "mimic_pendulums world is missing skeleton: "
          + std::string(rigs[i].first));
    }
    skeleton->setName(std::string(kSkeletonPrefix) + std::to_string(i));
    for (std::size_t bodyIndex = 0; bodyIndex < skeleton->getNumBodyNodes();
         ++bodyIndex) {
      if (auto* body = skeleton->getBodyNode(bodyIndex)) {
        body->setColor(rigs[i].second);
      }
    }
  }

  return world;
}

dart::gui::Panel createMimicPendulumsPanel()
{
  dart::gui::Panel panel;
  panel.title = "Mimic Pendulums";
  panel.buildWithContext = [](dart::gui::PanelBuilder& builder,
                              dart::gui::PanelContext& context) {
    builder.text("SDF mimic-joint pendulum comparison");
    builder.text("gray: uncoupled, red/blue: opposite mimic mappings");
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
  try {
    dart::gui::ApplicationOptions options;
    options.world = createMimicPendulumsWorld();
    options.panels.push_back(createMimicPendulumsPanel());
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "mimic_pendulums: " << e.what() << "\n";
    return 1;
  }
}
