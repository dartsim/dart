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

#ifndef DART_GUI_OSG_TEXTOVERLAY_HPP_
#define DART_GUI_OSG_TEXTOVERLAY_HPP_

#include <dart/gui/osg/Viewer.hpp>

#include <Eigen/Core>
#include <osg/Geode>

#include <string>
#include <vector>

#include <cstddef>

namespace dart {
namespace gui {
namespace osg {

/// Attach this to a Viewer to draw world-anchored, screen-facing text labels.
///
/// Each label is a piece of text pinned to a world-space position and rendered
/// with osgText so it is part of the OSG scene (it appears in off-screen
/// captures as well as interactive views). Labels always face the camera and
/// are drawn unlit and depth-test disabled so they stay legible over the
/// geometry they annotate. Names for bodies, frames, or contacts are the
/// intended use.
class TextOverlay : public ViewerAttachment
{
public:
  /// Default constructor
  TextOverlay();

  /// Sets the TrueType font file used when labels are (re)built. When empty
  /// (the default), osgText's default font resolution is used. Callers that
  /// know their environment can pass an absolute path to a .ttf.
  void setFont(const std::string& fontPath);

  /// Returns the configured font file, or an empty string for the default.
  const std::string& getFont() const;

  /// Sets the character size (screen pixels) used for labels added without an
  /// explicit size. Labels are sized in screen space so they stay legible
  /// regardless of the camera distance to the anchor.
  void setCharacterSize(double size);

  /// Returns the default character size.
  double getCharacterSize() const;

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

  /// Removes every label.
  void clear();

  /// Returns the number of labels currently held.
  std::size_t getNumLabels() const;

  /// Rebuilds the text drawables when the label set has changed. Called by the
  /// Viewer each refresh cycle.
  void refresh() override final;

protected:
  /// One world-anchored label. \c characterSize is in screen pixels.
  struct Label
  {
    Eigen::Vector3d position;
    std::string text;
    Eigen::Vector4d color;
    double characterSize;
  };

  /// Rebuilds the geode's drawables from \c mLabels.
  void rebuild();

  /// Font file used for labels, or empty for osgText's default.
  std::string mFontPath;

  /// Default character size (screen pixels).
  double mCharacterSize;

  /// The labels to draw.
  std::vector<Label> mLabels;

  /// Geode holding one osgText drawable per label.
  ::osg::ref_ptr<::osg::Geode> mGeode;

  /// Dirty flag: the drawables need rebuilding.
  bool mNeedUpdate;
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_TEXTOVERLAY_HPP_
