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

#ifndef DART_GUI_PANEL_HPP_
#define DART_GUI_PANEL_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>

#include <array>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include <cstddef>

namespace dart::simulation {
class World;
} // namespace dart::simulation

namespace dart::gui {

struct ViewerLifecycleState;

class DART_GUI_API PanelBuilder
{
public:
  virtual ~PanelBuilder() = default;

  virtual void text(std::string_view value) = 0;

  virtual void separator() = 0;

  virtual void sameLine() = 0;

  virtual bool button(std::string_view label) = 0;

  virtual bool checkbox(std::string_view label, bool& value) = 0;

  virtual bool slider(
      std::string_view label, double& value, double minimum, double maximum)
      = 0;

  virtual bool colorEdit(std::string_view label, Eigen::Vector4d& rgba)
  {
    (void)label;
    (void)rgba;
    return false;
  }

  virtual void colorSwatch(std::string_view label, const Eigen::Vector4d& rgba)
  {
    (void)rgba;
    text(label);
  }

  virtual void plotLines(std::string_view label, std::span<const double> values)
  {
    (void)values;
    text(label);
  }

  virtual bool beginTable(
      std::string_view label, std::span<const std::string_view> columns)
  {
    (void)label;
    (void)columns;
    return false;
  }

  virtual void tableNextRow()
  {
    // Default no-op for renderers without panel tables.
  }

  virtual bool tableNextColumn()
  {
    return false;
  }

  virtual void endTable()
  {
    // Default no-op for renderers without panel tables.
  }

  virtual bool collapsingHeader(std::string_view, bool = false)
  {
    return true;
  }

  virtual bool beginMenuBar()
  {
    return false;
  }

  virtual void endMenuBar()
  {
    // Default no-op for renderers without panel menu bars.
  }

  virtual bool beginMenu(std::string_view)
  {
    return false;
  }

  virtual void endMenu()
  {
    // Default no-op for renderers without panel menus.
  }

  virtual bool menuItem(std::string_view)
  {
    return false;
  }

  virtual void openModal(std::string_view, bool& open)
  {
    open = true;
  }

  virtual bool beginModal(std::string_view, bool& open)
  {
    return open;
  }

  virtual void endModal()
  {
    // Default no-op for renderers without panel modals.
  }
};

struct PanelContext
{
  struct CameraViewState
  {
    Eigen::Vector3d eye = Eigen::Vector3d::Zero();
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  };

  struct LightingState
  {
    bool* headlightsEnabled = nullptr;
  };

  struct RenderingState
  {
    RenderSettings* settings = nullptr;
  };

  struct UiState
  {
    Eigen::Vector2d displaySize = Eigen::Vector2d::Zero();
    Eigen::Vector2d framebufferScale = Eigen::Vector2d::Ones();
    double fontSize = 0.0;
    double fontGlobalScale = 1.0;
    double uiScale = 1.0;
    std::optional<std::array<int, 2>> fontTextureSize;
  };

  dart::simulation::World* world = nullptr;
  ViewerLifecycleState* lifecycle = nullptr;
  std::string selectedLabel;
  std::optional<Eigen::Vector3d> selectedPoint;
  std::optional<Eigen::Vector3d> selectedNormal;
  double simulationTime = 0.0;
  std::size_t contactCount = 0;
  CameraViewState camera;
  LightingState lighting;
  RenderingState rendering;
  UiState ui;
};

/// Where a panel docks in the default dock layout when docking is enabled.
///
/// Panels sharing a side become tabs in that region. `None` leaves the panel
/// floating. Ignored when the GUI build has no docking support.
enum class DockSide
{
  None,
  Left,
  Right,
  Top,
  Bottom,
  Center,
};

struct Panel
{
  std::string title;
  std::optional<std::array<double, 2>> initialPosition;
  std::optional<std::array<double, 2>> initialSize;
  std::optional<double> backgroundAlpha;
  bool autoResize = true;
  bool horizontalScrollbar = false;
  bool menuBar = false;
  DockSide dockSide = DockSide::None;
  std::function<void(PanelBuilder&)> build;
  std::function<void(PanelBuilder&, PanelContext&)> buildWithContext;
};

} // namespace dart::gui

#endif // DART_GUI_PANEL_HPP_
