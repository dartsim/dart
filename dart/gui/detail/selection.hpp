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

#ifndef DART_GUI_DETAIL_SELECTION_HPP_
#define DART_GUI_DETAIL_SELECTION_HPP_

#include <dart/gui/fwd.hpp>
#include <dart/gui/gizmo.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/dynamics/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>
#include <vector>

#include <cstddef>

struct GLFWwindow;

namespace dart::gui::detail {

struct DartScene;
struct FrameViewport;
struct IkHandle;

IkHandle* findIkHandle(
    DartScene& scene, dart::gui::RenderableId targetRenderableId);

const IkHandle* findIkHandle(
    const DartScene& scene, dart::gui::RenderableId targetRenderableId);

std::string selectionLabelForRenderable(
    const DartScene& scene, const dart::gui::RenderableDescriptor& descriptor);

bool translateRenderableAndApplyIk(
    DartScene& scene,
    const dart::gui::RenderableDescriptor& descriptor,
    const Eigen::Vector3d& worldTranslation);

class SelectionController
{
public:
  dart::gui::RenderableId selectedRenderableId() const;
  const std::string& selectedLabel() const;
  const std::optional<Eigen::Vector3d>& selectedPoint() const;
  const std::optional<Eigen::Vector3d>& selectedNormal() const;
  std::optional<dart::gui::GizmoHandleHit> highlightedGizmoHandle() const;
  dart::gui::RenderableId selectionDebugRenderableId() const;
  bool isDraggingSelection() const;

  void select(dart::gui::RenderableId renderableId, std::string label);
  void clear();

  void applyKeyboardNudge(
      GLFWwindow* window,
      const dart::gui::OrbitCamera& camera,
      DartScene& scene,
      std::vector<dart::gui::RenderableDescriptor>& descriptors,
      dart::gui::ViewerLifecycleState& lifecycle,
      double stepSize);

  bool updateMouseSelection(
      GLFWwindow* window,
      const FrameViewport& viewport,
      bool showUi,
      bool uiCapturesMouse,
      double guiScale,
      DartScene& scene,
      std::vector<dart::gui::RenderableDescriptor>& descriptors,
      dart::gui::ViewerLifecycleState& lifecycle);

private:
  enum class DragMode
  {
    Translate,
    Rotate,
    GizmoTranslateAxis,
    GizmoTranslatePlane,
    GizmoRotateAxis,
    BodyTranslate,
    BodyRotate,
    Force,
  };

  struct ActiveBodyNodeDrag
  {
    dart::dynamics::BodyNode* bodyNode = nullptr;
    dart::dynamics::InverseKinematicsPtr ik;
    Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
    Eigen::Matrix3d savedRotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d savedGlobalOffset = Eigen::Vector3d::Zero();
    Eigen::Vector3d savedLocalOffset = Eigen::Vector3d::Zero();
    Eigen::Isometry3d targetTransform = Eigen::Isometry3d::Identity();
  };

  /// State for a mouse "force-drag" (a mouse spring) begun on left-press over a
  /// renderable. `bodyNode` is non-null for legacy `World` renderables (force
  /// applied in-engine via `addExtForce`) and null for SimpleFrame-only
  /// renderables (e.g. sx mirrors), which are forwarded to
  /// `DartScene::onForceDrag`. The local offset records where on the body the
  /// grab landed so off-center grabs produce torque; the ray depth fixes the
  /// drag plane so the target follows the cursor in screen space.
  struct ActiveForceDrag
  {
    dart::gui::RenderableId renderableId = 0;
    std::string renderableName;
    dart::dynamics::BodyNode* bodyNode = nullptr;
    Eigen::Vector3d savedLocalOffset = Eigen::Vector3d::Zero();
    double rayDepth = 0.0;
  };

  bool beginBodyNodeDrag(
      GLFWwindow* window,
      const dart::gui::OrbitCamera& camera,
      DartScene& scene,
      const dart::gui::RenderableDescriptor& descriptor,
      const dart::gui::PickRay& cursorRay,
      double cursorX,
      double cursorY,
      dart::gui::ViewerLifecycleState& lifecycle);

  bool applyBodyNodeDragTranslation(const Eigen::Vector3d& worldTranslation);

  bool applyBodyNodeDragRotation(
      const Eigen::Vector3d& worldAxis, double angle);

  bool beginForceDrag(
      DartScene& scene,
      const dart::gui::RenderableDescriptor& descriptor,
      const dart::gui::PickRay& cursorRay,
      const Eigen::Vector3d& hitPointWorld,
      dart::gui::ViewerLifecycleState& lifecycle);

  void updateForceDrag(
      DartScene& scene,
      const std::vector<dart::gui::RenderableDescriptor>& descriptors,
      const dart::gui::PickRay& cursorRay);

  void endForceDrag(DartScene& scene);

  bool mWasLeftMousePressed = false;
  std::optional<std::size_t> mActivePointerPaneIndex;
  bool mLeftMouseStartedOnPanel = false;
  bool mLeftMouseStartedDrag = false;
  DragMode mSelectedDragMode = DragMode::Translate;
  double mLeftMousePressX = 0.0;
  double mLeftMousePressY = 0.0;
  double mSelectedDragLastCursorX = 0.0;
  double mSelectedDragLastCursorY = 0.0;
  dart::gui::PickRay mSelectedDragLastRay;
  bool mSelectedDragIsAxisConstrained = false;
  std::size_t mActiveGizmoIndex = 0u;
  std::optional<dart::gui::GizmoHandleHit> mHoveredGizmoHandle;
  std::optional<dart::gui::GizmoHandleHit> mActiveGizmoHandle;
  bool mSelectionBoundsVisible = true;
  Eigen::Vector3d mSelectedDragPlanePoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d mSelectedDragPlaneNormal = Eigen::Vector3d::UnitX();
  Eigen::Vector3d mSelectedDragAxisDirection = Eigen::Vector3d::UnitX();
  Eigen::Vector3d mSelectedRotationAxis = Eigen::Vector3d::UnitZ();
  std::optional<ActiveBodyNodeDrag> mActiveBodyNodeDrag;
  std::optional<ActiveForceDrag> mActiveForceDrag;
  dart::gui::RenderableId mSelectedRenderableId = 0;
  std::string mSelectedLabel = "none";
  std::optional<Eigen::Vector3d> mSelectedPoint;
  std::optional<Eigen::Vector3d> mSelectedNormal;
};

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_SELECTION_HPP_
