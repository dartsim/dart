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

// Ported from examples/tinkertoy: interactively assemble jointed structures
// from balsa-wood blocks. Left-click picks a block; a draggable target frame
// (an InteractiveFrame gizmo) then either pulls the picked block with a spring
// force (while simulating) or serves as the attachment pose for a new block
// (while paused, keys 1/2/3 or the panel buttons attach a Weld/Revolute/
// Ball-jointed block).
//
// Deviations from the original:
//  - The drag target is an InteractiveFrame dragged through
//    Viewer::enableDragAndDrop(InteractiveFrame*) (InteractiveFrameDnD), so its
//    per-axis/plane gizmo tools are fully usable -- the same as the original.
//    This relies on the accompanying gui-osg library fix that gives
//    InteractiveFrameDnD a real destructor (it deletes the 9 per-tool sub-DnDs
//    it allocates), making it safe to tear down on scene switch; the DnD is
//    registered in onActivate and released via ctx.addTeardown(
//    disableDragAndDrop). (DemoSceneSetup::dragFrames / SimpleFrameDnD is still
//    the right tool for plain SimpleFrames that want only an unconstrained
//    free-drag.)
//  - The original's Headlights On/Off checkbox (viewer->switchHeadlights) is
//    not ported: dart-demos exposes no per-scene headlight control and DART's
//    built-in Ctrl+H toggle still works. This is a dropped tunable, noted here.
//  - The keyboard input handler (Tab/1/2/3/Backspace/Delete/Up/Down/`) is
//    expressed entirely as KeyActions (auto-mirrored as panel buttons) rather
//    than a custom osgGA::GUIEventHandler; only the mouse-pick handler needs
//    a real handler class (MouseEventHandler, registered directly on the
//    host's DefaultEventHandler; its destruction auto-unregisters it, see
//    TinkertoyState.hpp).
//  - customPreRefresh's per-render-frame recolor + force-line update (which
//    must keep running while paused, since pausing is when you pick/attach
//    blocks) is done at the top of renderPanel, which the host already calls
//    once per rendered frame regardless of pause state -- the same per-frame
//    cadence customPreRefresh had, without needing a new host hook.
//  - Enter/Return (screencap recording to DART_DATA_PATH/screencap) is
//    dropped: writing into the source data path is inappropriate for an
//    installed/consolidated app, and dart-demos has no generic recording
//    feature to hang it on.
//  - The original's Help text advertised "[G] to toggle Gravity" with no
//    actual key handler wired to it (a documented parity bug: "NOTE: ImGui
//    help text advertises '[G]' ... but NO key handler for G exists"). Since
//    this is the one B2 scene that explicitly promises a keybinding it never
//    delivers, it is fixed here: 'g' now toggles gravity, matching the
//    existing Gravity On/Off checkbox.
//  - mPickedNode/mPickedPoint are zero-initialized in TinkertoyState (the
//    original left them default-constructed/uninitialized in the
//    constructor -- a documented latent-UB portRisk).
//  - The `mViewer->disableDragAndDrop(mViewer->enableDragAndDrop(...))`
//    dance in the original's deletePick() (run once per removed body node)
//    is not ported: it exists only to defensively tear down any DnD that
//    might be registered directly on a body node, but this port never
//    registers per-body DnD (only the target frame has one), so there is
//    nothing for it to guard against.
//  - Window renaming (realize() + getWindows()) is host-specific chrome and
//    is not replicated; the host owns the window title.

#include "../Scenes.hpp"
#include "TinkertoyState.hpp"

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <algorithm>
#include <memory>
#include <utility>

#include <cmath>

namespace dart_demos {

namespace {

using namespace tinkertoy; // NOLINT: scene-local convenience, matches the
                           // original's flat structure within one scene.

//==============================================================================
void createShapes(TinkertoyState& state)
{
  state.weldJointShape
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
          2.0 * kDefaultJointRadius, kDefaultBlockWidth, kDefaultBlockWidth));
  state.weldJointShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);

  state.revoluteJointShape = std::make_shared<dart::dynamics::CylinderShape>(
      kDefaultJointRadius, 1.5 * kDefaultBlockWidth);
  state.revoluteJointShape->addDataVariance(
      dart::dynamics::Shape::DYNAMIC_COLOR);

  state.ballJointShape
      = std::make_shared<dart::dynamics::SphereShape>(kDefaultJointRadius);
  state.ballJointShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);

  state.blockShape = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(
      kDefaultBlockLength, kDefaultBlockWidth, kDefaultBlockWidth));
  state.blockShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);

  state.blockOffset = Eigen::Isometry3d::Identity();
  state.blockOffset.translation()[0] = kDefaultBlockLength / 2.0;
}

//==============================================================================
template <class JointType>
std::pair<JointType*, dart::dynamics::BodyNode*> addBlock(
    const dart::simulation::WorldPtr& world,
    TinkertoyState& state,
    dart::dynamics::BodyNode* parent,
    const Eigen::Isometry3d& relTf,
    const dart::dynamics::ShapePtr& jointShape)
{
  if (state.isSimulating()) {
    // Pause first, as in the original -- which printed a hint; route it to the
    // host log so the mirrored panel buttons don't just silently no-op.
    if (state.log)
      state.log(
          "Tinkertoy: pause the simulation (Spacebar) before adding a "
          "block.");
    return {nullptr, nullptr};
  }

  dart::dynamics::SkeletonPtr skel;
  if (parent) {
    skel = parent->getSkeleton();
  } else {
    skel = dart::dynamics::Skeleton::create(
        "toy_#" + std::to_string(world->getNumSkeletons() + 1));
    world->addSkeleton(skel);
  }

  auto pair = skel->createJointAndBodyNodePair<JointType>(parent);
  JointType* joint = pair.first;
  dart::dynamics::BodyNode* bn = pair.second;
  bn->setName("block_#" + std::to_string(skel->getNumBodyNodes()));
  joint->setName("joint_#" + std::to_string(skel->getNumJoints()));

  joint->setTransformFromParentBodyNode(relTf);
  for (std::size_t i = 0; i < joint->getNumDofs(); ++i)
    joint->getDof(i)->setDampingCoefficient(kDefaultDamping);

  bn->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(jointShape);

  auto* block = bn->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(state.blockShape);
  block->setRelativeTransform(state.blockOffset);

  dart::dynamics::Inertia inertia = bn->getInertia();
  inertia.setMass(kDefaultBlockMass);
  inertia.setMoment(state.blockShape->computeInertia(kDefaultBlockMass));
  inertia.setLocalCOM(kDefaultBlockLength / 2.0 * Eigen::Vector3d::UnitX());
  bn->setInertia(inertia);

  world->getConstraintSolver()->getCollisionGroup()->addShapeFramesOf(bn);

  state.clearPick();

  return {joint, bn};
}

//==============================================================================
Eigen::Isometry3d getRelTf(const TinkertoyState& state)
{
  return state.pickedNode ? state.target->getTransform(state.pickedNode)
                          : state.target->getWorldTransform();
}

//==============================================================================
dart::dynamics::BodyNode* addWeldJointBlock(
    const dart::simulation::WorldPtr& world,
    TinkertoyState& state,
    dart::dynamics::BodyNode* parent,
    const Eigen::Isometry3d& relTf)
{
  return addBlock<dart::dynamics::WeldJoint>(
             world, state, parent, relTf, state.weldJointShape)
      .second;
}

//==============================================================================
dart::dynamics::BodyNode* addRevoluteJointBlock(
    const dart::simulation::WorldPtr& world,
    TinkertoyState& state,
    dart::dynamics::BodyNode* parent,
    const Eigen::Isometry3d& relTf)
{
  auto pair = addBlock<dart::dynamics::RevoluteJoint>(
      world, state, parent, relTf, state.revoluteJointShape);
  if (pair.first)
    pair.first->setAxis(Eigen::Vector3d::UnitZ());
  return pair.second;
}

//==============================================================================
dart::dynamics::BodyNode* addBallJointBlock(
    const dart::simulation::WorldPtr& world,
    TinkertoyState& state,
    dart::dynamics::BodyNode* parent,
    const Eigen::Isometry3d& relTf)
{
  return addBlock<dart::dynamics::BallJoint>(
             world, state, parent, relTf, state.ballJointShape)
      .second;
}

//==============================================================================
void createInitialToy1(
    const dart::simulation::WorldPtr& world, TinkertoyState& state)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(45.0), Eigen::Vector3d::UnitY()));
  dart::dynamics::BodyNode* bn = addBallJointBlock(world, state, nullptr, tf);

  tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = kDefaultBlockLength;
  tf.linear() = Eigen::Matrix3d::Identity();
  tf.prerotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitX()));
  bn = addRevoluteJointBlock(world, state, bn, tf);

  tf = Eigen::Isometry3d::Identity();
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitZ()));
  bn = addWeldJointBlock(world, state, bn, tf);

  tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = kDefaultBlockLength / 2.0;
  tf.translation()[2] = kDefaultBlockWidth;
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-30.0), Eigen::Vector3d::UnitZ()));
  addBallJointBlock(world, state, bn, tf);
}

//==============================================================================
void createInitialToy2(
    const dart::simulation::WorldPtr& world, TinkertoyState& state)
{
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitY()));
  tf.pretranslate(-1.0 * Eigen::Vector3d::UnitX());
  dart::dynamics::BodyNode* bn = addBallJointBlock(world, state, nullptr, tf);

  tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = kDefaultBlockLength;
  tf.translation()[2] = kDefaultBlockLength / 2.0;
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(90.0), Eigen::Vector3d::UnitY()));
  bn = addWeldJointBlock(world, state, bn, tf);

  tf = Eigen::Isometry3d::Identity();
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-90.0), Eigen::Vector3d::UnitX()));
  tf.rotate(
      Eigen::AngleAxisd(dart::math::toRadian(-90.0), Eigen::Vector3d::UnitZ()));
  tf.translation()[2] = kDefaultBlockWidth / 2.0;
  // The original discards this first revolute's return value so both revolute
  // blocks attach as siblings to the weld block (bn stays the weld block);
  // assigning bn here would instead chain the second revolute under the first.
  addRevoluteJointBlock(world, state, bn, tf);

  tf.translation()[0] = kDefaultBlockLength;
  bn = addRevoluteJointBlock(world, state, bn, tf);

  tf = Eigen::Isometry3d::Identity();
  tf.translation()[0] = kDefaultBlockLength;
  addBallJointBlock(world, state, bn, tf);
}

//==============================================================================
void createForceLine(
    const dart::simulation::WorldPtr& world, TinkertoyState& state)
{
  auto lineFrame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World());

  state.forceLine = std::make_shared<dart::dynamics::LineSegmentShape>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 3.0f);
  state.forceLine->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);

  lineFrame->setShape(state.forceLine);
  lineFrame->createVisualAspect();
  lineFrame->getVisualAspect()->setColor(Eigen::Vector4d(1.0, 0.63, 0.0, 1.0));

  world->addSimpleFrame(lineFrame);
}

//==============================================================================
void deletePick(const dart::simulation::WorldPtr& world, TinkertoyState& state)
{
  if (!state.pickedNode)
    return;
  if (state.isSimulating()) {
    if (state.log)
      state.log(
          "Tinkertoy: pause the simulation (Spacebar) before deleting a "
          "block.");
    return; // Pause first, as in the original.
  }

  dart::dynamics::SkeletonPtr removed = state.pickedNode->remove();
  world->getConstraintSolver()->getCollisionGroup()->removeShapeFramesOf(
      removed.get());

  state.clearPick();
}

//==============================================================================
void setAllBodyColors(
    const dart::simulation::WorldPtr& world, const Eigen::Vector4d& color)
{
  world->eachSkeleton(
      [&](dart::dynamics::Skeleton* skel) { skel->setColor(color); });
}

//==============================================================================
// Runs the same per-frame refresh the original's customPreRefresh did
// (recolor by sim/paused/selected state; update the force-line visual from
// the picked point to the target) -- see the file comment for why this lives
// at the top of renderPanel instead.
void updateVisuals(
    const dart::simulation::WorldPtr& world, TinkertoyState& state)
{
  const bool simulating = state.isSimulating();

  if (simulating) {
    setAllBodyColors(world, Eigen::Vector4d(0.5, 0.5, 1.0, 1.0));
    if (state.pickedNode)
      state.pickedNode->setColor(dart::Color::Fuchsia(1.0));
  } else {
    setAllBodyColors(
        world, Eigen::Vector4d(238.0 / 255.0, 201.0 / 255.0, 0.0 / 255.0, 1.0));
    if (state.pickedNode)
      state.pickedNode->setColor(dart::Color::Red(1.0));
  }

  if (state.pickedNode) {
    state.forceLine->setVertex(
        0, state.pickedNode->getWorldTransform() * state.pickedPoint);
    state.forceLine->setVertex(
        1, state.target->getWorldTransform().translation());
  } else {
    state.forceLine->setVertex(0, Eigen::Vector3d::Zero());
    state.forceLine->setVertex(1, Eigen::Vector3d::Zero());
  }
}

} // namespace

//==============================================================================
DemoScene makeTinkertoyScene()
{
  DemoScene scene;
  scene.id = "tinkertoy";
  scene.title = "Tinkertoy";
  scene.category = "Constraints & Joints";
  scene.summary
      = "Interactively assemble jointed structures from balsa-wood blocks.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();

    // Purely-visual world-origin reference axes; all tools disabled, so it
    // is never dragged (matches the original's "coordinates" InteractiveFrame).
    auto coordinates = std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World(),
        "coordinates",
        Eigen::Isometry3d::Identity(),
        0.2);
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j)
        coordinates
            ->getTool(static_cast<dart::gui::osg::InteractiveTool::Type>(i), j)
            ->setEnabled(false);
    world->addSimpleFrame(coordinates);

    auto state = std::make_shared<TinkertoyState>();
    state->world = world;
    state->target = std::make_shared<dart::gui::osg::InteractiveFrame>(
        dart::dynamics::Frame::World());
    world->addSimpleFrame(state->target);

    createShapes(*state);
    createInitialToy1(world, *state);
    createInitialToy2(world, *state);
    createForceLine(world, *state);
    // Prime colors/force-line geometry once at build time, matching the B1
    // convention of initializing visuals so a paused fresh scene looks right
    // immediately (updateVisuals otherwise only runs from renderPanel).
    updateVisuals(world, *state);

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(3.0, 3.0, 2.0),
        ::osg::Vec3d(0.0, 0.0, 0.3),
        ::osg::Vec3d(0.0, 0.0, 1.0)};

    setup.onActivate = [state](DemoHostContext& ctx) {
      state->viewer = ctx.viewer();
      state->log = [ctx](const std::string& message) {
        ctx.log(message);
      };

      auto mouse = std::make_shared<tinkertoy::TinkertoyMouseHandler>(
          ctx.viewer(), state);
      ctx.viewer()->getDefaultEventHandler()->addMouseEventHandler(mouse.get());
      // Kept alive by this teardown lambda; deleting it on teardown unregisters
      // it cleanly from the DefaultEventHandler (see TinkertoyState.hpp).
      ctx.addTeardown([mouse] {});

      // Make the target's gizmo tools draggable via InteractiveFrameDnD, torn
      // down on scene switch (safe now that the gui-osg fix gives that DnD a
      // real destructor; see the file header).
      auto* viewer = ctx.viewer();
      if (auto* dnd = viewer->enableDragAndDrop(state->target.get()))
        ctx.addTeardown([viewer, dnd] { viewer->disableDragAndDrop(dnd); });
    };

    setup.preStep = [world, state] {
      if (!state->pickedNode)
        return;

      Eigen::Vector3d force
          = static_cast<double>(state->forceCoeff)
            * (state->target->getWorldTransform().translation()
               - state->pickedNode->getWorldTransform() * state->pickedPoint);
      const double forceNorm = force.norm();
      if (forceNorm > kMaxForce)
        force = kMaxForce * force / forceNorm;

      state->pickedNode->addExtForce(force, state->pickedPoint);
    };

    auto setGravity = [world](bool on) {
      world->setGravity(
          on ? Eigen::Vector3d(0.0, 0.0, -9.81) : Eigen::Vector3d::Zero());
    };

    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Tab),
        "Reset camera",
        [state] {
          if (state->viewer)
            state->viewer->home();
        }});
    setup.keyActions.push_back(KeyAction{
        '1', "Add weld-joint block", [world, state] {
          addWeldJointBlock(world, *state, state->pickedNode, getRelTf(*state));
        }});
    setup.keyActions.push_back(
        KeyAction{'2', "Add revolute-joint block", [world, state] {
                    addRevoluteJointBlock(
                        world, *state, state->pickedNode, getRelTf(*state));
                  }});
    setup.keyActions.push_back(KeyAction{
        '3', "Add ball-joint block", [world, state] {
          addBallJointBlock(world, *state, state->pickedNode, getRelTf(*state));
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_BackSpace),
        "Clear selection",
        [state] {
          state->clearPick();
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Delete),
        "Delete picked block",
        [world, state] {
          deletePick(world, *state);
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Up),
        "Increase force coeff",
        [state] {
          state->incrementForceCoeff();
        }});
    setup.keyActions.push_back(KeyAction{
        static_cast<int>(::osgGA::GUIEventAdapter::KEY_Down),
        "Decrease force coeff",
        [state] {
          state->decrementForceCoeff();
        }});
    setup.keyActions.push_back(KeyAction{'`', "Reorient target", [state] {
                                           state->reorientTarget();
                                         }});
    setup.keyActions.push_back(
        KeyAction{'g', "Toggle gravity", [world, setGravity] {
                    // Read the world's actual gravity (not a cached flag) so
                    // this stays in sync with the host toolbar's Gravity
                    // toggle.
                    const bool on = world->getGravity().norm() > 1e-8;
                    setGravity(!on);
                  }});

    setup.renderPanel = [world, state, setGravity] {
      updateVisuals(world, *state);

      if (state->pickedNode)
        ImGui::Text("Selected: %s", state->pickedNode->getName().c_str());
      else
        ImGui::TextUnformatted("Selected: (none)");

      auto force = state->forceCoeff;
      ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
      if (ImGui::SliderFloat(
              "Force Coeff",
              &force,
              kMinForceCoeff,
              kMaxForceCoeff,
              "%.1f",
              ImGuiSliderFlags_AlwaysClamp)
          && std::isfinite(force))
        state->forceCoeff = std::clamp(force, kMinForceCoeff, kMaxForceCoeff);

      // Read the world's actual gravity so this checkbox tracks the host
      // toolbar's Gravity toggle instead of drifting out of sync.
      bool gravityOn = world->getGravity().norm() > 1e-8;
      if (ImGui::Checkbox("Gravity On/Off", &gravityOn))
        setGravity(gravityOn);

      ImGui::Spacing();
      if (ImGui::Button("Reorient Target"))
        state->reorientTarget();
      ImGui::SameLine();
      if (ImGui::Button("Reset Target"))
        state->clearPick();

      // Short labels stacked under a caption so the three add-block buttons fit
      // the panel width (the full "Add ...-Joint Block" labels overflowed the
      // row and clipped).
      ImGui::TextUnformatted("Add block at target:");
      if (ImGui::Button("Weld"))
        addWeldJointBlock(world, *state, state->pickedNode, getRelTf(*state));
      ImGui::SameLine();
      if (ImGui::Button("Revolute"))
        addRevoluteJointBlock(
            world, *state, state->pickedNode, getRelTf(*state));
      ImGui::SameLine();
      if (ImGui::Button("Ball"))
        addBallJointBlock(world, *state, state->pickedNode, getRelTf(*state));

      if (ImGui::Button("Delete Block"))
        deletePick(world, *state);

      ImGui::TextWrapped(
          "Left-click a block to select it (fuchsia/red while "
          "simulating/paused). While paused: 1/2/3 attach a new "
          "Weld/Revolute/Ball-jointed block to the target's pose; Delete "
          "removes the selection and its children. While simulating, "
          "dragging the target pulls the selection with a clamped spring "
          "force (Up/Down or the slider adjust its strength). Backspace "
          "clears the selection, '`' resets the target's rotation, 'g' "
          "toggles gravity.");
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos
