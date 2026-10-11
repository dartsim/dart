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

#ifndef DART_GUI_GRIDVISUAL_HPP_
#define DART_GUI_GRIDVISUAL_HPP_

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

/// Attach this to a Viewer in order to visualize grid.
class DART_GUI_API GridVisual : public ViewerAttachment
{
public:
  enum class PlaneType : unsigned char
  {
    XY = 0u,
    YZ = 1u,
    ZX = 2u,
  };

  /// Default constructor
  GridVisual();
  ~GridVisual() override;

  /// Sets the number of cells along each axis
  void setNumCells(std::size_t cells);

  /// Returns the number of cells along each axis
  std::size_t getNumCells() const;

  /// Sets the step size of minor lines in meters
  void setMinorLineStepSize(double size);

  /// Returns the step size of minor lines in meters
  double getMinorLineStepSize() const;

  /// Sets the number of minor lines per major line
  void setNumMinorLinesPerMajorLine(std::size_t size);

  /// Returns the number of minor lines per major line
  std::size_t getNumMinorLinesPerMajorLine() const;

  /// Set the plane type among XY, YZ, ZX planes
  void setPlaneType(PlaneType type);

  /// Returns the plane type among XY, YZ, ZX planes
  PlaneType getPlaneType() const;

  /// Changes the offset at which the grid is displayed
  void setOffset(const Eigen::Vector3d& offset);

  /// Returns the elevation of display for the support polygon
  const Eigen::Vector3d& getOffset() const;

  /// Displays the support polygon
  void display(bool display);

  /// Returns true if the support polygon is being displayed
  bool isDisplayed() const;

  /// Sets the color of major lines
  void setMajorLineColor(const Eigen::Vector4d& color);

  /// Returns the color of major lines
  Eigen::Vector4d getMajorLineColor() const;

  /// Sets the color of minor lines
  void setMinorLineColor(const Eigen::Vector4d& color);

  /// Returns the color of minor lines
  Eigen::Vector4d getMinorLineColor() const;

  /// Sets line width for axis lines
  void setAxisLineWidth(float width);

  /// Returns line width for axis lines
  float getAxisLineWidth() const;

  /// Sets line width for major lines
  void setMajorLineWidth(float width);

  /// Returns line width for major lines
  float getMajorLineWidth() const;

  /// Sets line width for minor lines
  void setMinorLineWidth(float width);

  /// Returns line width for minor lines
  float getMinorLineWidth() const;

  /// Updates the support polygon visual
  void refresh() override final;

private:
  friend struct detail::OverlayAccess;
  struct Impl;
  std::unique_ptr<Impl> mImpl;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_GRIDVISUAL_HPP_
