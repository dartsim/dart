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

#include "Scenes.hpp"

#include <dart/utils/CompositeResourceRetriever.hpp>
#include <dart/utils/PackageResourceRetriever.hpp>
#include <dart/utils/assets/ModelResourceRetriever.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

#include <cmath>

namespace dart_demos {
namespace {

struct PoseEdits
{
  int selected = 0;
  int pendingJoint = -1;
  double pendingPosition = 0.0;
  bool reset = false;
};

//==============================================================================
DemoScene makeModelScene(
    const std::string& sceneId,
    const std::string& title,
    const std::string& modelId,
    const std::string& revision,
    const std::string& entrypoint,
    const std::string& packageName,
    double rootHeight)
{
  DemoScene scene;
  scene.id = sceneId;
  scene.title = title;
  scene.category = "Robots";
  scene.summary = "Inspect a verified cached humanoid and pose its joints.";
  const std::string root = "model://" + modelId + "/" + revision + "/";
  scene.factory = [root, entrypoint, packageName, rootHeight, modelId] {
    auto model
        = std::make_shared<dart::utils::ModelResourceRetriever>("", true);
    const dart::common::Uri uri(root + entrypoint);
    try {
      if (model->getFilePath(uri).empty())
        throw std::runtime_error("Verified model cache is unavailable.");
    } catch (const std::exception& error) {
      throw std::runtime_error(
          std::string(error.what())
          + " Prefetch with: pixi run fetch-robot-assets " + modelId);
    }

    auto package
        = std::make_shared<dart::utils::PackageResourceRetriever>(model);
    package->addPackageDirectory(packageName, root);
    auto retriever
        = std::make_shared<dart::utils::CompositeResourceRetriever>();
    retriever->addSchemaRetriever("model", model);
    retriever->addSchemaRetriever("package", package);
    dart::utils::DartLoader loader(dart::utils::DartLoader::Options(
        retriever, dart::utils::DartLoader::RootJointType::FIXED));
    auto robot = loader.parseSkeleton(uri);
    if (!robot || robot->getNumDofs() == 0)
      throw std::runtime_error(
          "Unable to parse cached model: " + root + entrypoint);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation().z() = rootHeight;
    robot->getRootJoint()->setTransformFromParentBodyNode(transform);
    robot->setMobile(false);
    const Eigen::VectorXd initialPose = robot->getPositions();
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());
    world->addSkeleton(robot);

    DemoSceneSetup setup;
    setup.world = world;
    setup.enableShadows = false;
    // Frame the full robot above the diagnostics panel and between side panels.
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(6.0, 5.0, 3.0),
        ::osg::Vec3d(0.0, 0.0, rootHeight - 0.6),
        ::osg::Vec3d(0.0, 0.0, 1.0)};
    auto edits = std::make_shared<PoseEdits>();
    setup.preRefresh = [robot, edits, initialPose] {
      if (edits->reset) {
        robot->setPositions(initialPose);
        edits->reset = false;
      }
      if (edits->pendingJoint >= 0) {
        robot->getDof(edits->pendingJoint)->setPosition(edits->pendingPosition);
        edits->pendingJoint = -1;
      }
    };
    setup.renderPanel = [robot, edits] {
      ImGui::Text("Fixed base; kinematic joint inspection.");
      ImGui::Text(
          "%zu bodies, %zu joints to pose",
          robot->getNumBodyNodes(),
          robot->getNumDofs());
      const int lastJoint = static_cast<int>(robot->getNumDofs()) - 1;
      int selected = edits->selected;
      ImGui::SliderInt(
          "Joint", &selected, 0, lastJoint, "%d", ImGuiSliderFlags_AlwaysClamp);
      edits->selected = std::clamp(selected, 0, lastJoint);
      auto* dof = robot->getDof(edits->selected);
      ImGui::TextUnformatted(dof->getName().c_str());
      double position = dof->getPosition();
      const double lower = std::isfinite(dof->getPositionLowerLimit())
                               ? dof->getPositionLowerLimit()
                               : -dart::math::constantsd::pi();
      const double upper = std::isfinite(dof->getPositionUpperLimit())
                               ? dof->getPositionUpperLimit()
                               : dart::math::constantsd::pi();
      if (ImGui::SliderScalar(
              "Position (rad)",
              ImGuiDataType_Double,
              &position,
              &lower,
              &upper,
              "%.3f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(position)) {
        edits->pendingJoint = edits->selected;
        edits->pendingPosition = std::clamp(position, lower, upper);
      }
      if (ImGui::Button("Reset pose")) {
        edits->reset = true;
        edits->pendingJoint = -1;
      }
    };
    setup.onActivate = [](DemoHostContext& context) {
      auto* viewer = context.viewer();
      const bool resume = viewer->isSimulating();
      viewer->allowSimulation(false);
      context.addTeardown([viewer, resume] {
        viewer->allowSimulation(true);
        if (resume)
          viewer->simulate(true);
      });
    };
    return setup;
  };
  return scene;
}

} // namespace

//==============================================================================
DemoScene makeAtlasV5Scene()
{
  return makeModelScene(
      "atlas_v5",
      "Atlas v5 (No Head)",
      "atlas-v5",
      "0d92c25f336db51049a9516de46886838e6ea596",
      "atlas_v5_no_head.urdf",
      "atlas_description",
      1.0);
}

//==============================================================================
DemoScene makeUnitreeG1Scene()
{
  return makeModelScene(
      "unitree_g1",
      "Unitree G1 (29 DOF)",
      "unitree-g1",
      "5994d4faef0a9cadd3287f8de0199a67eeb2a259",
      "g1_29dof_mode_15.urdf",
      "g1_description",
      0.8);
}

} // namespace dart_demos
