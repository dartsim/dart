/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_DYNAMICS_POINTCLOUDSHAPE_HPP_
#define DART_DYNAMICS_POINTCLOUDSHAPE_HPP_

#include "dart/dynamics/Shape.hpp"

#if HAVE_OCTOMAP
#  include <octomap/Pointcloud.h>
#endif

namespace dart {
namespace dynamics {

/// The PointCloudShape represents point cloud data.
class PointCloudShape : public Shape
{
public:
  enum ColorMode
  {
    /// Use the color specified by the ShapeAspect. This is the default.
    USE_SHAPE_COLOR = 0,

    /// Use one color for all the points
    BIND_OVERALL,

    /// Use one color per point
    BIND_PER_POINT,
  };

  enum PointShapeType
  {
    BOX = 0,          ///< 3D volumetric box
    BILLBOARD_SQUARE, ///< 2D square always facing the screen
    BILLBOARD_CIRCLE, ///< 2D circle always facing the screen
    POINT,            ///< Vertex
  };

  /// Constructor
  ///
  /// \param[in] visualSize The size of cube that represents each point.
  explicit PointCloudShape(double visualSize = 0.01);

  /// Destructor
  ~PointCloudShape() override = default;

  // Documentation inherited.
  const std::string& getType() const override;

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  /// Reserves the point list by \c size.
  void reserve(std::size_t size);

  /// Adds a point to this point cloud.
  void addPoint(const Eigen::Vector3d& point);

  /// Adds points to this point cloud.
  void addPoint(const std::vector<Eigen::Vector3d>& points);

  /// Replaces points with \c points.
  void setPoint(const std::vector<Eigen::Vector3d>& points);

#if HAVE_OCTOMAP
  /// Replaces points with \c pointCloud.
  void setPoints(::octomap::Pointcloud& pointCloud);

  /// Adds points from Octomap PointCloud.
  void addPoints(::octomap::Pointcloud& pointCloud);
#endif

  /// Returns the list of points.
  const std::vector<Eigen::Vector3d>& getPoints() const;

  /// Returns the number of points.
  std::size_t getNumPoints() const;

  /// Removes all the points.
  void removeAllPoints();

  /// Sets the point shape type.
  void setPointShapeType(PointShapeType type);

  /// Returns the point shape type.
  PointShapeType getPointShapeType() const;

  /// Sets the color mode.
  void setColorMode(ColorMode mode);

  /// Returns the color mode.
  ColorMode getColorMode() const;

  /// Sets the overall color.
  ///
  /// This function resizes the colors to one.
  void setOverallColor(const Eigen::Vector4d& color);

  /// Returns the overall color.
  Eigen::Vector4d getOverallColor() const;

  /// Sets the point cloud colors.
  ///
  /// The count of colors should be the same with points. It's undefined
  /// behavior, otherwise.
  void setColors(const std::vector<
                 Eigen::Vector4d,
                 Eigen::aligned_allocator<Eigen::Vector4d>>& colors);

  /// Returns the point cloud colors.
  const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>&
  getColors() const;

  /// Sets size of visual object that represents each point.
  void setVisualSize(double size);

  /// Returns size of visual object that represents each point.
  double getVisualSize() const;

  // Documentation inherited.
  void notifyColorUpdated(const Eigen::Vector4d& color) override;

protected:
  // Documentation inherited.
  void updateVolume() const override;

  // Documentation inherited.
  void updateBoundingBox() const override;

  /// List of points
  std::vector<Eigen::Vector3d> mPoints;

  /// The point shape type.
  PointShapeType mPointShapeType;

  /// The color mode
  ColorMode mColorMode;

  /// List of colors
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      mColors;

  /// The size of visual object that represents each point.
  double mVisualSize;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_POINTCLOUDSHAPE_HPP_
