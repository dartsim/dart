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

// Shared state for the Tinkertoy scene port (examples/tinkertoy). Split into
// its own header so the scene can mirror the original's
// TinkertoyWorldNode+TinkertoyWidget ownership split without widening the
// public demo-host API.

#ifndef DART_EXAMPLES_DEMOS_SCENES_TINKERTOY_TINKERTOYSTATE_HPP_
#define DART_EXAMPLES_DEMOS_SCENES_TINKERTOY_TINKERTOYSTATE_HPP_

#include <dart/gui/osg/osg.hpp>

#include <dart/dart.hpp>

#include <functional>
#include <memory>
#include <string>

namespace dart_demos {
namespace tinkertoy {

constexpr double kDefaultBlockLength = 0.5;
constexpr double kDefaultBlockWidth = 0.075;
constexpr double kDefaultJointRadius = 1.5 * kDefaultBlockWidth / 2.0;
// Literal preserved exactly from the original (`0.16 * 10e3`): 10e3 is
// 10*10^3 = 10000, not 10^3, so this evaluates to 1600 kg/m^3 rather than the
// ~160 kg/m^3 a real balsa-wood density would suggest. Kept as-is for
// behavior parity (it only affects how heavy the blocks feel to sling
// around with the force target).
constexpr double kBalsaWoodDensity = 0.16 * 10e3;
constexpr double kDefaultBlockMass = kBalsaWoodDensity * kDefaultBlockLength
                                     * kDefaultBlockWidth * kDefaultBlockWidth;
constexpr double kDefaultDamping = 0.4;

constexpr double kMaxForce = 200.0;
constexpr float kDefaultForceCoeff = 100.0f;
constexpr float kMaxForceCoeff = 1000.0f;
constexpr float kMinForceCoeff = 10.0f;
constexpr float kForceIncrement = 10.0f;

//==============================================================================
/// Per-instance state captured by this scene's preStep/postStep/renderPanel/
/// key-action lambdas and the mouse pick handler (one instance per factory()
/// call, i.e. per Rebuild/Reset too).
struct TinkertoyState
{
  dart::simulation::WorldPtr world;
  dart::gui::osg::ImGuiViewer* viewer = nullptr;

  dart::dynamics::ShapePtr weldJointShape;
  dart::dynamics::ShapePtr revoluteJointShape;
  dart::dynamics::ShapePtr ballJointShape;
  dart::dynamics::ShapePtr blockShape;
  Eigen::Isometry3d blockOffset = Eigen::Isometry3d::Identity();

  // Zero-initialized here (the original's TinkertoyWorldNode constructor left
  // these two default-constructed/uninitialized -- a documented portRisk:
  // "mPickedNode and mPickedPoint are NOT initialized in the constructor ...
  // customPreRefresh reads mPickedNode before any pick, i.e. latent UB").
  dart::dynamics::BodyNode* pickedNode = nullptr;
  Eigen::Vector3d pickedPoint = Eigen::Vector3d::Zero();

  dart::gui::osg::InteractiveFramePtr target;
  dart::dynamics::LineSegmentShapePtr forceLine;

  float forceCoeff = kDefaultForceCoeff;

  // Routes user-facing status (e.g. "pause before adding a block") to the
  // host's diagnostics log; set from onActivate. May be empty during
  // build-time setup, so always null-check before calling.
  std::function<void(const std::string&)> log;

  /// Moves the drag target to the pick location (offset outward along the
  /// picked surface normal) and records the picked body + local pick point.
  void handlePick(const dart::gui::osg::PickInfo& pick);

  /// Clears the current pick and resets the target to identity, as in the
  /// original's clearPick().
  void clearPick();

  /// Zeroes the target's rotation, keeping its current position.
  void reorientTarget();

  void incrementForceCoeff();
  void decrementForceCoeff();

  bool isSimulating() const
  {
    return viewer && viewer->isSimulating();
  }
};

//==============================================================================
/// Converts LEFT_MOUSE BUTTON_PUSH picks into TinkertoyState::handlePick(),
/// exactly as the original TinkertoyMouseHandler did. Held alive by a
/// shared_ptr captured in this scene's teardown lambda; when the scene is torn
/// down (switch, Rebuild/Reset, or app shutdown) that lambda is released and
/// the handler is deleted while the DefaultEventHandler is still alive.
///
/// Deleting it then unregisters it cleanly: DefaultEventHandler::
/// addMouseEventHandler observes the handler (addSubject(handler)), so the
/// handler's Subject destructor notifies the DefaultEventHandler, whose
/// handleDestructionNotification erases it from mMouseEventHandlers -- no
/// dangling pointer is left for the next mouse event. (That observer wiring was
/// added in the accompanying gui-osg library fix; before it, the header's
/// documented "removed automatically upon deletion" contract was unimplemented,
/// leaving a use-after-free after a scene switch.) The host destroys the
/// teardown list before the viewer (DemoHost member order), so the handler
/// never outlives the DefaultEventHandler and the MouseEventHandler suicide
/// path (delete-on-DefaultEventHandler-death) never races the shared_ptr.
class TinkertoyMouseHandler : public dart::gui::osg::MouseEventHandler
{
public:
  TinkertoyMouseHandler(
      dart::gui::osg::Viewer* viewer, std::shared_ptr<TinkertoyState> state)
    : mViewer(viewer), mState(std::move(state))
  {
  }

  void update() override;

private:
  dart::gui::osg::Viewer* mViewer;
  std::shared_ptr<TinkertoyState> mState;
};

} // namespace tinkertoy
} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_SCENES_TINKERTOY_TINKERTOYSTATE_HPP_
