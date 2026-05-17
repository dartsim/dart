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

#ifndef DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SELECTION_HPP_
#define DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SELECTION_HPP_

#include <dart/gui/gizmo.hpp>
#include <dart/gui/renderable.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

#include <cstddef>

struct GLFWwindow;

namespace dart::gui::filament {

struct DartScene;
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

  void updateMouseSelection(
      GLFWwindow* window,
      const dart::gui::OrbitCamera& camera,
      int framebufferWidth,
      int framebufferHeight,
      bool showUi,
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
  };

  bool mWasLeftMousePressed = false;
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
  Eigen::Vector3d mSelectedDragPlanePoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d mSelectedDragPlaneNormal = Eigen::Vector3d::UnitX();
  Eigen::Vector3d mSelectedDragAxisDirection = Eigen::Vector3d::UnitX();
  Eigen::Vector3d mSelectedRotationAxis = Eigen::Vector3d::UnitZ();
  dart::gui::RenderableId mSelectedRenderableId = 0;
  std::string mSelectedLabel = "none";
  std::optional<Eigen::Vector3d> mSelectedPoint;
  std::optional<Eigen::Vector3d> mSelectedNormal;
};

} // namespace dart::gui::filament

#endif // DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SELECTION_HPP_
