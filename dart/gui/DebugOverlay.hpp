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

#ifndef DART_GUI_DEBUGOVERLAY_HPP_
#define DART_GUI_DEBUGOVERLAY_HPP_

#include <dart/gui/ViewerAttachment.hpp>

#include <dart/dynamics/SmartPointer.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

#include <cstddef>

namespace dart {
namespace gui {
namespace detail {
struct OverlayAccess;
}

/// Attach this to a Viewer to draw always-on-top debug primitives.
///
/// A debug overlay collects world-space line segments and text labels and
/// renders them unlit, with depth testing disabled, after the scene, so
/// they stay legible on top of the geometry they annotate rather than being
/// buried inside opaque bodies. It is part of the renderer scene, so it appears
/// in off-screen captures as well as interactive views. Contact markers,
/// body-frame axes, velocity and force arrows, trajectories, and body names are
/// the intended content.
class DART_GUI_API DebugOverlay : public ViewerAttachment
{
public:
  /// Default constructor
  DebugOverlay();
  ~DebugOverlay() override;

  /// Sets the TrueType font file used when labels are (re)built. When empty
  /// (the default), the viewer's default font is used. Callers that
  /// know their environment can pass an absolute path to a .ttf.
  void setFont(const std::string& fontPath);

  /// Returns the configured font file, or an empty string for the default.
  const std::string& getFont() const;

  /// Sets the width, in pixels, of overlay lines.
  void setLineWidth(float width);

  /// Returns the overlay line width.
  float getLineWidth() const;

  /// Sets the character size (world units) used for labels added without an
  /// explicit size.
  void setCharacterSize(double size);

  /// Returns the default character size.
  double getCharacterSize() const;

  /// Adds a line segment in world coordinates. \c color is RGBA in [0, 1].
  /// Returns the line index.
  std::size_t addLine(
      const Eigen::Vector3d& start,
      const Eigen::Vector3d& end,
      const Eigen::Vector4d& color);

  /// Adds a label anchored at \c position with the default character size.
  /// Returns the label index. \c color is RGBA in [0, 1].
  std::size_t addLabel(
      const Eigen::Vector3d& position,
      const std::string& text,
      const Eigen::Vector4d& color);

  /// Adds a label anchored at \c position with an explicit character size.
  std::size_t addLabel(
      const Eigen::Vector3d& position,
      const std::string& text,
      const Eigen::Vector4d& color,
      double characterSize);

  /// Adds a world-space label whose character size stays constant in pixels.
  std::size_t addLabelPixels(
      const Eigen::Vector3d& position,
      const std::string& text,
      const Eigen::Vector4d& color,
      double size = 16.0);

  /// Adds a label at physical pixel coordinates with the origin at the top
  /// left.
  std::size_t addScreenLabel(
      const Eigen::Vector2d& position,
      const std::string& text,
      const Eigen::Vector4d& color,
      double size = 16.0);

  void setTextOutlineWidth(double width);
  double getTextOutlineWidth() const;
  void setTextOutlineColor(const Eigen::Vector4d& color);
  Eigen::Vector4d getTextOutlineColor() const;

  /// Removes every line and label.
  void clear();

  /// Returns the number of line segments currently held.
  std::size_t getNumLines() const;

  /// Returns the number of labels currently held.
  std::size_t getNumLabels() const;

  /// Refreshes cached annotation data. Called by the
  /// Viewer each refresh cycle.
  void refresh() override final;

private:
  friend struct detail::OverlayAccess;
  struct Impl;
  std::unique_ptr<Impl> mImpl;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_DEBUGOVERLAY_HPP_
