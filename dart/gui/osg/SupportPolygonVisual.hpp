/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_GUI_OSG_SUPPORTPOLYGONVISUAL_HPP_
#define DART_GUI_OSG_SUPPORTPOLYGONVISUAL_HPP_

#include <osg/Geode>

#include "dart/dynamics/SmartPointer.hpp"

#include "dart/gui/osg/Viewer.hpp"
#include "dart/gui/osg/ShapeFrameNode.hpp"

namespace dart {
namespace gui {
namespace osg {

/// Attach this to a Viewer in order to visualize the support polygon of a
/// Skeleton
class SupportPolygonVisual : public ViewerAttachment
{
public:

  /// Visualize the support polygon of an entire Skeleton
  SupportPolygonVisual(const dart::dynamics::SkeletonPtr& skeleton = nullptr,
                double elevation = 0.02);

  /// Visualize the support polygon of a specific tree in a Skeleton
  SupportPolygonVisual(const dart::dynamics::SkeletonPtr& skeleton, std::size_t treeIndex,
                double elevation = 0.02);

  /// Change the Skeleton that is being visualized
  void setSkeleton(const dart::dynamics::SkeletonPtr& skeleton);

  /// Get the Skeleton associated with this visual
  dart::dynamics::SkeletonPtr getSkeleton() const;

  /// Visualize the entire Skeleton
  void visualizeWholeSkeleton();

  /// Visualize a specific tree in the Skeleton
  void visualizeTree(std::size_t treeIndex);

  /// Change the elevation height at which the polygon is displayed
  void setDisplayElevation(double elevation);

  /// Get the elevation of display for the support polygon
  double getDisplayElevation() const;

  /// Display the support polygon
  void displayPolygon(bool display);

  /// Returns true if the support polygon is being displayed
  bool isPolygonDisplayed() const;

  /// Set the color of the support polygon
  void setPolygonColor(const Eigen::Vector4d& color);

  /// Get the color of the support polygon
  Eigen::Vector4d getPolygonColor() const;

  /// Display the centroid
  void displayCentroid(bool display);

  /// Returns true if the centroid is being displayed
  bool isCentroidDisplayed() const;

  /// Set the radius of the centroid visualization
  void setCentroidRadius(double radius);

  /// Get the radius of the centroid visualization
  double getCentroidRadius() const;

  /// Display the center of mass
  void displayCenterOfMass(bool display);

  /// Returns true if the center of mass is being displayed
  bool isCenterOfMassDisplayed() const;

  /// Set the radius of the center of mass visualization
  void setCenterOfMassRadius(double radius);

  /// Get the radius of the center of mass visualization
  double getCenterOfMassRadius() const;

  /// Set the color that will be used for the center of mass if its projection
  /// is on the support polygon
  void setValidCOMColor(const Eigen::Vector4d& color);

  /// Get the color that will be used for the center of mass if its projection
  /// is on the support polygon
  const Eigen::Vector4d& getValidCOMColor() const;

  /// Set the color that will be used for the center of mass if its projection
  /// is NOT on the support polygon
  void setInvalidCOMColor(const Eigen::Vector4d& color);

  /// Get the color that will be used for the center of mass if its projection
  /// is NOT on the support polygon
  const Eigen::Vector4d& getInvalidCOMColor() const;

  /// Update the support polygon visual
  void refresh() override final;

protected:

  /// Initialize the memory used by this visual
  void initialize();

  /// Skeleton for this visual
  dart::dynamics::WeakSkeletonPtr mSkeleton;

  /// Tree index for this visual
  std::size_t mTreeIndex;

  /// Elevation that this visual should use
  double mElevation;

  /// Whether to display the polygon
  bool mDisplayPolygon;

  /// Whether to display the centroid
  bool mDisplayCentroid;

  /// SimpleFrame for the centroid
  dart::dynamics::SimpleFramePtr mCentroid;

  /// Radius to be used by the centroid
  double mCentroidRadius;

  /// Whether to display the center of mass
  bool mDisplayCOM;

  /// SimpleFrame for the center of mass
  dart::dynamics::SimpleFramePtr mCom;

  /// Radius to be used by the center of mass
  double mComRadius;

  /// Color to be used when COM is valid
  Eigen::Vector4d mValidColor;

  /// Color to be used when COM is invalid
  Eigen::Vector4d mInvalidColor;

  /// Color for the polygon
  ::osg::ref_ptr<::osg::Vec4Array> mPolygonColor;

  /// Geode to hold the polygon
  ::osg::ref_ptr<::osg::Geode> mPolygonGeode;

  /// Geometry to describe the polygon
  ::osg::ref_ptr<::osg::Geometry> mPolygonGeom;

  /// Vertices of the polygon
  ::osg::ref_ptr<::osg::Vec3Array> mVertices;

  /// Faces of the polygon
  ::osg::ref_ptr<::osg::DrawElementsUShort> mFaces;

  /// Node to render the centroid
  ::osg::ref_ptr<ShapeFrameNode> mCentroidNode;

  /// Node to render the COM
  ::osg::ref_ptr<ShapeFrameNode> mComNode;
};

} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_SUPPORTPOLYGONVISUAL_HPP_
