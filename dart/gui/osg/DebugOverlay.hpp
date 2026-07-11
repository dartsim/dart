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

#ifndef DART_GUI_OSG_DEBUGOVERLAY_HPP_
#define DART_GUI_OSG_DEBUGOVERLAY_HPP_

#include <dart/gui/osg/Viewer.hpp>

#include <Eigen/Core>
#include <osg/Geode>

#include <string>
#include <vector>

#include <cstddef>

namespace dart {
namespace gui {
namespace osg {

/// Attach this to a Viewer to draw always-on-top debug primitives.
///
/// A debug overlay collects world-space line segments and text labels and
/// renders them unlit, with depth testing disabled, in a late render bin, so
/// they stay legible on top of the geometry they annotate rather than being
/// buried inside opaque bodies. This mirrors the DART 7 core debug overlay
/// treatment and is part of the OSG scene, so it appears in off-screen captures
/// as well as interactive views. Contact markers, body-frame axes, velocity and
/// force arrows, trajectories, and body names are the intended content.
class DebugOverlay : public ViewerAttachment
{
public:
  /// Default constructor
  DebugOverlay();

  /// Sets the TrueType font file used when labels are (re)built. When empty
  /// (the default), osgText's default font resolution is used. Callers that
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

  /// Removes every line and label.
  void clear();

  /// Returns the number of line segments currently held.
  std::size_t getNumLines() const;

  /// Returns the number of labels currently held.
  std::size_t getNumLabels() const;

  /// Rebuilds the drawables when the primitive set has changed. Called by the
  /// Viewer each refresh cycle.
  void refresh() override final;

protected:
  /// One world-space line segment.
  struct Line
  {
    Eigen::Vector3d start;
    Eigen::Vector3d end;
    Eigen::Vector4d color;
  };

  /// One world-anchored label. \c characterSize is in world units.
  struct Label
  {
    Eigen::Vector3d position;
    std::string text;
    Eigen::Vector4d color;
    double characterSize;
  };

  /// Rebuilds the geode's drawables from \c mLines and \c mLabels.
  void rebuild();

  /// Font file used for labels, or empty for osgText's default.
  std::string mFontPath;

  /// Width of overlay lines in pixels.
  float mLineWidth;

  /// Default character size (world units).
  double mCharacterSize;

  /// The line segments to draw.
  std::vector<Line> mLines;

  /// The labels to draw.
  std::vector<Label> mLabels;

  /// Geode holding the line geometry and one osgText drawable per label.
  ::osg::ref_ptr<::osg::Geode> mGeode;

  /// Dirty flag: the drawables need rebuilding.
  bool mNeedUpdate;
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_DEBUGOVERLAY_HPP_
