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

// Ported from examples/simple_frames: a static hierarchy of nested
// SimpleFrames (boxes F1 -> F2 -> F3, ellipsoids A -> A1/A2/A3, and a free
// arrow), demonstrating that Frame children render through their ancestor's
// world attachment. No physics; there are no skeletons in this world.

#include "Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <memory>

namespace dart_demos {

namespace {

//==============================================================================
struct SimpleFramesState
{
  dart::dynamics::SimpleFramePtr arrow;
  bool showArrow = true;
};

} // namespace

//==============================================================================
DemoScene makeSimpleFramesScene()
{
  DemoScene scene;
  scene.id = "simple_frames";
  scene.title = "Simple Frames";
  scene.category = "Visualization";
  scene.summary
      = "Nested SimpleFrames with ellipsoid markers and an arrow shape.";

  scene.factory = [] {
    using dart::dynamics::ArrowShape;
    using dart::dynamics::BoxShape;
    using dart::dynamics::EllipsoidShape;
    using dart::dynamics::Frame;
    using dart::dynamics::SimpleFrame;
    using dart::dynamics::SimpleFramePtr;

    auto world = dart::simulation::World::create();

    Eigen::Isometry3d tf1(Eigen::Isometry3d::Identity());
    tf1.translate(Eigen::Vector3d(0.1, -0.1, 0));

    Eigen::Isometry3d tf2(Eigen::Isometry3d::Identity());
    tf2.translate(Eigen::Vector3d(0, 0.1, 0));
    tf2.rotate(Eigen::AngleAxisd(
        dart::math::toRadian(45.0), Eigen::Vector3d(1, 0, 0)));

    Eigen::Isometry3d tf3(Eigen::Isometry3d::Identity());
    tf3.translate(Eigen::Vector3d(0, 0, 0.1));
    tf3.rotate(Eigen::AngleAxisd(
        dart::math::toRadian(60.0), Eigen::Vector3d(0, 1, 0)));

    // The original never calls createVisualAspect()/setColor() on any of
    // these frames; ShapeFrameNode only renders a shape frame that has a
    // VisualAspect (dart/gui/osg/ShapeFrameNode.cpp), so the original's shape
    // frames are actually never drawn -- setShape() alone does not create
    // one. That leaves the demo showing nothing, defeating the whole point
    // of a "visualize nested frames" scene, so this port adds an explicit
    // createVisualAspect() + a distinguishing color per frame (R/G/B for
    // F1/F2/F3, matched on A1/A2/A3 to show which ellipsoid tracks which box
    // frame's pose).
    auto withColor = [](const dart::dynamics::SimpleFramePtr& frame,
                        const Eigen::Vector3d& color) {
      frame->createVisualAspect()->setColor(color);
      return frame;
    };

    auto f1 = std::make_shared<SimpleFrame>(Frame::World(), "F1", tf1);
    f1->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.05, 0.05, 0.02)));
    withColor(f1, Eigen::Vector3d(0.85, 0.25, 0.25));
    auto f2 = std::make_shared<SimpleFrame>(f1.get(), "F2", tf2);
    f2->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.05, 0.05, 0.02)));
    withColor(f2, Eigen::Vector3d(0.25, 0.75, 0.25));
    auto f3 = std::make_shared<SimpleFrame>(f2.get(), "F3", tf3);
    f3->setShape(std::make_shared<BoxShape>(Eigen::Vector3d(0.05, 0.05, 0.02)));
    withColor(f3, Eigen::Vector3d(0.25, 0.45, 0.90));

    // Adding a Frame to the world also renders every Entity that descends
    // from it; F2 and F3 render through F1's world attachment even though
    // only F1 is added below.
    world->addSimpleFrame(f1);

    auto a = std::make_shared<SimpleFrame>(Frame::World(), "A");
    a->setShape(
        std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.02, 0.02, 0.02)));
    withColor(a, Eigen::Vector3d(0.85, 0.85, 0.85));
    auto a1 = std::make_shared<SimpleFrame>(
        a.get(), "A1", f1->getTransform(a.get()));
    a1->setShape(
        std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.01, 0.01, 0.01)));
    withColor(a1, Eigen::Vector3d(0.85, 0.25, 0.25));
    auto a2 = std::make_shared<SimpleFrame>(
        a.get(), "A2", f2->getTransform(a.get()));
    a2->setShape(
        std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.01, 0.01, 0.01)));
    withColor(a2, Eigen::Vector3d(0.25, 0.75, 0.25));
    auto a3 = std::make_shared<SimpleFrame>(
        a.get(), "A3", f3->getTransform(a.get()));
    a3->setShape(
        std::make_shared<EllipsoidShape>(Eigen::Vector3d(0.01, 0.01, 0.01)));
    withColor(a3, Eigen::Vector3d(0.25, 0.45, 0.90));
    world->addSimpleFrame(a);

    auto arrow = std::make_shared<SimpleFrame>(Frame::World(), "arrow");
    arrow->setShape(std::make_shared<ArrowShape>(
        Eigen::Vector3d(0.1, -0.1, 0.0),
        Eigen::Vector3d(0.1, 0.0, 0.0),
        ArrowShape::Properties(0.002, 1.8),
        Eigen::Vector4d(1.0, 0.5, 0.5, 1.0)));
    withColor(arrow, Eigen::Vector3d(1.0, 0.5, 0.5));
    world->addSimpleFrame(arrow);

    auto state = std::make_shared<SimpleFramesState>();
    state->arrow = arrow;

    // DART's Frame parent-child link is a non-owning raw pointer (the
    // original example kept F2/F3/A1-A3 alive as stack locals for the whole
    // program); here every frame -- including the ones only reachable as a
    // child of f1/a -- must be kept alive by the preStep lambda for the
    // scene's lifetime instead.
    std::vector<SimpleFramePtr> anchors{f1, f2, f3, a, a1, a2, a3, arrow};

    DemoSceneSetup setup;
    setup.world = world;
    // All eight frames do render (each gets an explicit VisualAspect above, and
    // F2/F3 and A1/A2/A3 draw through their parents' world attachment), but the
    // whole hierarchy is a ~0.25 m cluster sitting near the origin -- the
    // original's 2,1,2 -> origin view left it a tiny speck in the middle of an
    // otherwise empty viewport. Pull the eye in close and aim at the cluster's
    // centroid so the boxes, ellipsoid markers, and arrow fill the frame.
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(0.55, 0.25, 0.5),
        ::osg::Vec3d(0.08, 0.0, 0.03),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.preStep = [anchors] {
      // Nothing to simulate; the frames are purely kinematic. This lambda
      // only keeps the anchors' shared_ptrs alive for the scene's lifetime.
      (void)anchors;
    };

    setup.renderPanel = [state] {
      ImGui::TextWrapped(
          "F1 -> F2 -> F3 are nested box frames; A is an ellipsoid with "
          "child markers A1/A2/A3 tracking F1/F2/F3's poses.");

      bool showArrow = state->showArrow;
      if (ImGui::Checkbox("Show arrow", &showArrow)) {
        state->showArrow = showArrow;
        if (auto* visual = state->arrow->getVisualAspect(true))
          visual->setHidden(!showArrow);
      }
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
