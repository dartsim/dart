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

#ifndef DART_GUI_POLYHEDRONVISUAL_HPP_
#define DART_GUI_POLYHEDRONVISUAL_HPP_

#include <dart/gui/export.hpp>
#include <dart/gui/viewer.hpp>

#include <Eigen/Core>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>

#include <span>
#include <vector>

namespace dart {
namespace gui {

/// Visualizes a convex polyhedron built from an arbitrary set of input
/// vertices. The vertices are converted into a convex hull before being sent to
/// OpenSceneGraph, which allows callers to provide a V-representation of the
/// desired shape.
class DART_GUI_API PolyhedronVisual : public ViewerAttachment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Default constructor. Creates an empty visual with a semi-transparent fill
  /// and a dark wireframe outline.
  PolyhedronVisual();

  /// Sets the vertices by taking ownership of a vector.
  void setVertices(std::vector<Eigen::Vector3d>&& vertices);

  /// Sets the vertices by copying from a contiguous span. At least three points
  /// are required, and four non-coplanar points are required to display a 3D
  /// hull.
  void setVertices(std::span<const Eigen::Vector3d> vertices);

  /// Sets the vertices from a matrix. The matrix can either be 3xN (each column
  /// is a vertex) or Nx3 (each row is a vertex).
  void setVertices(const Eigen::Ref<const Eigen::MatrixXd>& vertices);

  /// Returns the raw vertices that were provided most recently.
  std::span<const Eigen::Vector3d> getVertices() const;

  /// Removes all vertices and clears the rendered geometry.
  void clear();

  /// Sets the color of the filled faces.
  void setSurfaceColor(const Eigen::Vector4d& color);

  /// Returns the color of the filled faces.
  Eigen::Vector4d getSurfaceColor() const;

  /// Sets the color of the wireframe overlay.
  void setWireframeColor(const Eigen::Vector4d& color);

  /// Returns the color of the wireframe overlay.
  Eigen::Vector4d getWireframeColor() const;

  /// Sets the OpenGL line width that is used for the wireframe overlay.
  void setWireframeWidth(float width);

  /// Returns the OpenGL line width used for the wireframe overlay.
  float getWireframeWidth() const;

  /// Shows or hides the entire visual.
  void display(bool display);

  /// Returns true if the entire visual is displayed.
  bool isDisplayed() const;

  /// Shows or hides the filled faces.
  void displaySurface(bool display);

  /// Returns true if the filled faces are displayed.
  bool isSurfaceDisplayed() const;

  /// Shows or hides the wireframe overlay.
  void displayWireframe(bool display);

  /// Returns true if the wireframe overlay is displayed.
  bool isWireframeDisplayed() const;

  /// Updates the underlying OpenSceneGraph geometry.
  void refresh() override final;

private:
  void initialize();
  void updateGeometry();
  void clearGeometry();
  void updateWireframeWidth();

  bool mDisplay;
  bool mDisplaySurface;
  bool mDisplayWireframe;
  bool mDirty;

  std::vector<Eigen::Vector3d> mVertices;

  ::osg::ref_ptr<::osg::Geode> mGeode;

  ::osg::ref_ptr<::osg::Geometry> mSurfaceGeom;
  ::osg::ref_ptr<::osg::Vec3Array> mSurfaceVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mSurfaceNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mSurfaceColor;
  ::osg::ref_ptr<::osg::DrawElementsUInt> mSurfaceIndices;

  ::osg::ref_ptr<::osg::Geometry> mWireframeGeom;
  ::osg::ref_ptr<::osg::DrawElementsUInt> mWireframeIndices;
  ::osg::ref_ptr<::osg::Vec4Array> mWireframeColor;
  ::osg::ref_ptr<::osg::LineWidth> mWireframeWidth;

  float mWireWidth;
};

} // namespace gui
} // namespace dart

#endif // DART_GUI_POLYHEDRONVISUAL_HPP_
