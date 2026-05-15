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

#include <dart/gui/experimental/renderable.hpp>
#include <dart/gui/experimental/viewer.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

struct GLFWwindow;

namespace dart::gui::experimental::filament {

struct DartScene;
struct IkHandle;

IkHandle* findIkHandle(
    DartScene& scene, dart::gui::experimental::RenderableId targetRenderableId);

const IkHandle* findIkHandle(
    const DartScene& scene,
    dart::gui::experimental::RenderableId targetRenderableId);

std::string selectionLabelForRenderable(
    const DartScene& scene,
    const dart::gui::experimental::RenderableDescriptor& descriptor);

bool translateRenderableAndApplyIk(
    DartScene& scene,
    const dart::gui::experimental::RenderableDescriptor& descriptor,
    const Eigen::Vector3d& worldTranslation);

class SelectionController
{
public:
  dart::gui::experimental::RenderableId selectedRenderableId() const;
  const std::string& selectedLabel() const;
  bool isDraggingSelection() const;

  void select(
      dart::gui::experimental::RenderableId renderableId, std::string label);
  void clear();

  void applyKeyboardNudge(
      GLFWwindow* window,
      const dart::gui::experimental::OrbitCamera& camera,
      DartScene& scene,
      std::vector<dart::gui::experimental::RenderableDescriptor>& descriptors,
      dart::gui::experimental::ViewerLifecycleState& lifecycle,
      double stepSize);

  void updateMouseSelection(
      GLFWwindow* window,
      const dart::gui::experimental::OrbitCamera& camera,
      int framebufferWidth,
      int framebufferHeight,
      bool showUi,
      double guiScale,
      DartScene& scene,
      std::vector<dart::gui::experimental::RenderableDescriptor>& descriptors,
      dart::gui::experimental::ViewerLifecycleState& lifecycle);

private:
  bool mWasLeftMousePressed = false;
  bool mLeftMouseStartedOnPanel = false;
  bool mLeftMouseStartedDrag = false;
  double mLeftMousePressX = 0.0;
  double mLeftMousePressY = 0.0;
  dart::gui::experimental::PickRay mSelectedDragLastRay;
  bool mSelectedDragIsAxisConstrained = false;
  Eigen::Vector3d mSelectedDragPlanePoint = Eigen::Vector3d::Zero();
  Eigen::Vector3d mSelectedDragPlaneNormal = Eigen::Vector3d::UnitX();
  Eigen::Vector3d mSelectedDragAxisDirection = Eigen::Vector3d::UnitX();
  dart::gui::experimental::RenderableId mSelectedRenderableId = 0;
  std::string mSelectedLabel = "none";
};

} // namespace dart::gui::experimental::filament

#endif // DART_GUI_EXPERIMENTAL_DETAIL_FILAMENT_SELECTION_HPP_
