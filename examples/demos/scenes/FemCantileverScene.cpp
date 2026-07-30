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

// Shows what elastic element forces do for a volumetric FEM body. Two identical
// tetrahedral boxes are pinned along their left face and left to hang. The near
// body carries a stiffness so low that elastic forces are negligible, which is
// how the body behaved before a constitutive model existed: nothing couples the
// nodes, so the unpinned ones simply fall away and the body comes apart. The
// far body uses co-rotated elasticity at a realistic stiffness, so it holds
// together and settles into a sagging cantilever.
//
// The body has no renderable shape of its own yet, so each node is drawn as a
// small sphere carried by a SimpleFrame. Pinned nodes are drawn dark.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/fem/DeformableBody.hpp>
#include <dart/dynamics/fem/TetMesh.hpp>

#include <memory>
#include <vector>

namespace dart_demos {

namespace {

const Eigen::Vector3d kBoxSize(0.5, 0.16, 0.12);
const Eigen::Vector3i kBoxDivisions(6, 2, 2);
constexpr double kNodeRadius = 0.016;
constexpr double kDamping = 6.0;

/// One rendered deformable body: the simulated body plus the spheres that
/// visualize its nodes.
struct VisualBody
{
  dart::dynamics::fem::DeformableBodyPtr body;
  std::vector<dart::dynamics::SimpleFramePtr> nodeFrames;
  /// Each node's starting height, so sag measures displacement rather than the
  /// node's offset from the body origin.
  std::vector<double> initialHeights;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  double maxSag = 0.0;
};

//==============================================================================
VisualBody makeBody(
    const dart::simulation::WorldPtr& world,
    const std::string& name,
    const Eigen::Vector3d& origin,
    double youngsModulus,
    const Eigen::Vector3d& color)
{
  const auto mesh
      = dart::dynamics::fem::TetMesh::createBox(kBoxSize, kBoxDivisions);

  dart::dynamics::fem::Material material;
  material.mYoungsModulus = youngsModulus;
  material.mLinearDamping = kDamping;

  VisualBody visual;
  visual.origin = origin;
  visual.body = dart::dynamics::fem::DeformableBody::create(mesh, material);

  // Shift the whole body to its place in the scene, then pin the left face.
  const double anchorX = -0.5 * kBoxSize[0] + 1e-9;
  for (std::size_t i = 0; i < visual.body->getNumNodes(); ++i) {
    const Eigen::Vector3d rest = mesh.getRestPosition(i);
    visual.body->setNodePosition(i, rest + origin);
    if (rest[0] <= anchorX)
      visual.body->setNodeFixed(i, true);
  }

  for (std::size_t i = 0; i < visual.body->getNumNodes(); ++i) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), name + "_node_" + std::to_string(i));
    frame->setShape(std::make_shared<dart::dynamics::SphereShape>(kNodeRadius));
    frame->createVisualAspect();
    frame->getVisualAspect()->setColor(
        visual.body->isNodeFixed(i) ? Eigen::Vector3d(0.15, 0.15, 0.18)
                                    : color);
    frame->setTranslation(visual.body->getNodePosition(i));
    world->addSimpleFrame(frame);
    visual.nodeFrames.push_back(frame);
    visual.initialHeights.push_back(visual.body->getNodePosition(i)[2]);
  }

  visual.body->attachTo(world);
  return visual;
}

//==============================================================================
void refresh(VisualBody& visual)
{
  double sag = 0.0;
  for (std::size_t i = 0; i < visual.body->getNumNodes(); ++i) {
    const Eigen::Vector3d position = visual.body->getNodePosition(i);
    visual.nodeFrames[i]->setTranslation(position);
    // Measure each node against where it started. The box is centered on its
    // origin, so comparing against the origin would report half the box
    // thickness as sag before anything had moved.
    if (!visual.body->isNodeFixed(i))
      sag = std::max(sag, visual.initialHeights[i] - position[2]);
  }
  visual.maxSag = sag;
}

struct SceneState
{
  VisualBody withoutElasticity;
  VisualBody withElasticity;
};

} // namespace

//==============================================================================
DemoScene makeFemCantileverScene()
{
  DemoScene scene;
  scene.id = "fem_cantilever";
  scene.title = "FEM Cantilever";
  scene.category = "Soft Bodies";
  scene.summary
      = "Two pinned tetrahedral boxes hang side by side: one with negligible "
        "stiffness that falls apart, one with co-rotated elasticity that holds "
        "together and sags.";

  scene.factory = []() {
    DemoSceneSetup setup;

    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
    world->setTimeStep(0.001);

    auto state = std::make_shared<SceneState>();

    // Negligible stiffness stands in for having no constitutive model at all:
    // the nodes are not coupled, so the free ones fall away on their own.
    state->withoutElasticity = makeBody(
        world,
        "no_elasticity",
        Eigen::Vector3d(0.0, -0.28, 0.0),
        1.0,
        Eigen::Vector3d(0.85, 0.35, 0.25));

    state->withElasticity = makeBody(
        world,
        "corotational",
        Eigen::Vector3d(0.0, 0.28, 0.0),
        5.0e5,
        Eigen::Vector3d(0.25, 0.55, 0.9));

    setup.world = world;
    setup.enableShadows = false;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(1.15, -1.30, 0.55),
        ::osg::Vec3d(0.00, 0.00, -0.13),
        ::osg::Vec3d(0.00, 0.00, 1.00)};

    setup.postStep = [state]() {
      refresh(state->withoutElasticity);
      refresh(state->withElasticity);
    };

    setup.preRefresh = [state]() {
      refresh(state->withoutElasticity);
      refresh(state->withElasticity);
    };

    setup.renderPanel = [state, world]() {
      ImGui::Text(
          "Frame: %d   time: %.2f s", world->getSimFrames(), world->getTime());
      ImGui::Separator();
      ImGui::TextColored(
          ImVec4(0.85f, 0.35f, 0.25f, 1.0f), "Negligible stiffness (near)");
      ImGui::Text("  max drop: %.3f m", state->withoutElasticity.maxSag);
      ImGui::Text(
          "  elastic energy: %.3e J",
          state->withoutElasticity.body->getElasticEnergy());
      ImGui::TextColored(
          ImVec4(0.25f, 0.55f, 0.9f, 1.0f), "Co-rotated elasticity (far)");
      ImGui::Text("  max sag: %.3f m", state->withElasticity.maxSag);
      ImGui::Text(
          "  elastic energy: %.3e J",
          state->withElasticity.body->getElasticEnergy());
      ImGui::TextWrapped(
          "Both boxes are pinned along their left face. Without elastic forces "
          "the unpinned nodes are uncoupled and fall away; with co-rotated "
          "elasticity the body holds together and settles into a cantilever.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
