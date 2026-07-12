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

namespace dart::gui {

struct ViewerLifecycleState;

/// Renderer-neutral non-color encoding for a panel block or ordered segment.
enum class PanelBlockPattern
{
  Solid,
  ForwardHatch,
  BackwardHatch,
  CrossHatch,
  Dots,
};

/// One ordered proportional segment inside a panel block.
struct PanelBlockSegment
{
  Eigen::Vector4d rgba = Eigen::Vector4d(0.4, 0.4, 0.4, 1.0);
  /// Relative width. The renderer normalizes all positive segment weights.
  double weight{1.0};
  PanelBlockPattern pattern{PanelBlockPattern::Solid};
};

/// One renderer-neutral cell in a panel block grid.
///
/// The tooltip and ordered segments are non-owning views that only need to
/// remain valid for the duration of PanelBuilder::blockGrid(). An empty
/// `segments` span uses the single-color/pattern fallback.
struct PanelBlock
{
  Eigen::Vector4d rgba = Eigen::Vector4d(0.4, 0.4, 0.4, 1.0);
  std::string_view tooltip;
  PanelBlockPattern pattern{PanelBlockPattern::Solid};
  std::span<const PanelBlockSegment> segments;
};

class DART_GUI_API PanelBuilder
{
public:
  virtual ~PanelBuilder() = default;

  virtual void text(std::string_view value) = 0;

  virtual void separator() = 0;

  virtual void sameLine() = 0;

  virtual bool button(std::string_view label) = 0;

  /// A selectable text row. Renders like a list item (text + hover/active
  /// highlight) instead of a button. Useful for navigation panels (e.g. the
  /// built-in demo catalog sidebar).
  virtual bool selectable(std::string_view label, bool selected = false)
  {
    (void)selected;
    return button(label);
  }

  /// Show a tooltip with `text` while the most recently added item (e.g. the
  /// preceding `selectable`/`button`) is hovered. Lets list rows explain
  /// themselves on hover (e.g. the demo catalog showing each scene's summary).
  /// Default is a no-op for builders without hover support.
  virtual void itemTooltip(std::string_view text)
  {
    (void)text;
  }

  /// Push/pop a horizontal indent in points. Lets callers visually nest
  /// list items under a category heading (tree-style).
  virtual void indent(double width = 12.0)
  {
    (void)width;
  }
  virtual void unindent(double width = 12.0)
  {
    (void)width;
  }

  virtual bool checkbox(std::string_view label, bool& value) = 0;

  virtual bool textInput(std::string_view label, std::string& value)
  {
    (void)label;
    (void)value;
    return false;
  }

  virtual bool slider(
      std::string_view label, double& value, double minimum, double maximum)
      = 0;

  /// A scrubber-style timeline control with optional diagnostic tracks.
  ///
  /// The primary value behaves like `slider`. Renderers with richer panel
  /// support may draw the optional tracks as lanes below the scrubber; simpler
  /// renderers fall back to regular plots through this default implementation.
  virtual bool timeline(
      std::string_view label,
      double& value,
      double minimum,
      double maximum,
      std::span<const double> valueTrack = {},
      std::span<const double> markerTrack = {},
      std::span<const double> cursorTrack = {},
      std::string_view valueTrackLabel = "Values")
  {
    const bool changed = slider(label, value, minimum, maximum);
    const std::string labelValue(label);
    if (!valueTrack.empty()) {
      plotLines(std::string(valueTrackLabel) + "##" + labelValue, valueTrack);
    }
    if (!markerTrack.empty()) {
      plotLines("Marks##" + labelValue, markerTrack);
    }
    if (!cursorTrack.empty()) {
      plotLines("Cursor##" + labelValue, cursorTrack);
    }
    return changed;
  }

  virtual bool colorEdit(std::string_view label, Eigen::Vector4d& rgba)
  {
    (void)label;
    (void)rgba;
    return false;
  }

  virtual bool select(
      std::string_view label,
      int& selectedIndex,
      std::span<const std::string_view> choices)
  {
    (void)label;
    (void)selectedIndex;
    (void)choices;
    return false;
  }

  virtual void colorSwatch(std::string_view label, const Eigen::Vector4d& rgba)
  {
    (void)rgba;
    text(label);
  }

  /// Draw a responsive two-dimensional grid of colored diagnostic blocks.
  ///
  /// Renderers may reduce the requested column count when the panel is narrow.
  /// Builders without a block-grid implementation fall back to the label.
  virtual void blockGrid(
      std::string_view label,
      std::span<const PanelBlock> blocks,
      std::size_t preferredColumns = 32)
  {
    (void)blocks;
    (void)preferredColumns;
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
    OrbitCamera orbit;
    std::function<void(const OrbitCamera&)> setOrbitCamera;
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
    double userScale = 1.0;
    double dpiScale = 1.0;
    double uiScale = 1.0;
    std::optional<std::array<int, 2>> fontTextureSize;
  };

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
  void* nativeWindow = nullptr;
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
