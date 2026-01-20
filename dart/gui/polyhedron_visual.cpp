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

#include "dart/gui/polyhedron_visual.hpp"

#include "dart/common/logging.hpp"
#include "dart/math/geometry.hpp"

#include <osg/StateSet>

#include <limits>
#include <set>
#include <span>
#include <tuple>
#include <utility>

namespace dart {
namespace gui {

namespace {

Eigen::Vector3d computeSafeNormal(const Eigen::Vector3d& normal)
{
  if (normal.norm() < 1e-9)
    return Eigen::Vector3d::UnitZ();

  return normal.normalized();
}

std::pair<std::size_t, std::size_t> makeOrderedEdge(
    std::size_t a, std::size_t b)
{
  if (a < b)
    return std::make_pair(a, b);

  return std::make_pair(b, a);
}

} // namespace

//==============================================================================
PolyhedronVisual::PolyhedronVisual()
  : mDisplay(true),
    mDisplaySurface(true),
    mDisplayWireframe(true),
    mDirty(true),
    mWireWidth(2.0f)
{
  initialize();
}

//==============================================================================
void PolyhedronVisual::setVertices(std::vector<Eigen::Vector3d>&& vertices)
{
  mVertices = std::move(vertices);
  mDirty = true;
}

//==============================================================================
void PolyhedronVisual::setVertices(std::span<const Eigen::Vector3d> vertices)
{
  mVertices.assign(vertices.begin(), vertices.end());
  mDirty = true;
}

//==============================================================================
void PolyhedronVisual::setVertices(
    const Eigen::Ref<const Eigen::MatrixXd>& vertices)
{
  if (vertices.size() == 0) {
    clear();
    return;
  }

  std::vector<Eigen::Vector3d> converted;

  if (vertices.rows() == 3) {
    converted.reserve(vertices.cols());
    for (int i = 0; i < vertices.cols(); ++i)
      converted.emplace_back(vertices(0, i), vertices(1, i), vertices(2, i));
  } else if (vertices.cols() == 3) {
    converted.reserve(vertices.rows());
    for (int i = 0; i < vertices.rows(); ++i)
      converted.emplace_back(vertices(i, 0), vertices(i, 1), vertices(i, 2));
  } else {
    DART_WARN(
        "[PolyhedronVisual::setVertices] Expected either a 3xN or Nx3 matrix. "
        "Received {}x{}; ignoring the request.",
        vertices.rows(),
        vertices.cols());
    return;
  }

  mVertices = std::move(converted);
  mDirty = true;
}

//==============================================================================
std::span<const Eigen::Vector3d> PolyhedronVisual::getVertices() const
{
  return std::span<const Eigen::Vector3d>(mVertices);
}

//==============================================================================
void PolyhedronVisual::clear()
{
  mVertices.clear();
  mDirty = true;
}

//==============================================================================
void PolyhedronVisual::setSurfaceColor(const Eigen::Vector4d& color)
{
  (*mSurfaceColor)[0] = ::osg::Vec4(color[0], color[1], color[2], color[3]);
  mSurfaceGeom->setColorArray(mSurfaceColor, ::osg::Array::BIND_OVERALL);
}

//==============================================================================
Eigen::Vector4d PolyhedronVisual::getSurfaceColor() const
{
  const ::osg::Vec4& c = (*mSurfaceColor)[0];
  return Eigen::Vector4d(c[0], c[1], c[2], c[3]);
}

//==============================================================================
void PolyhedronVisual::setWireframeColor(const Eigen::Vector4d& color)
{
  (*mWireframeColor)[0] = ::osg::Vec4(color[0], color[1], color[2], color[3]);
  mWireframeGeom->setColorArray(mWireframeColor, ::osg::Array::BIND_OVERALL);
}

//==============================================================================
Eigen::Vector4d PolyhedronVisual::getWireframeColor() const
{
  const ::osg::Vec4& c = (*mWireframeColor)[0];
  return Eigen::Vector4d(c[0], c[1], c[2], c[3]);
}

//==============================================================================
void PolyhedronVisual::setWireframeWidth(float width)
{
  if (mWireWidth == width)
    return;

  mWireWidth = width;
  updateWireframeWidth();
}

//==============================================================================
float PolyhedronVisual::getWireframeWidth() const
{
  return mWireWidth;
}

//==============================================================================
void PolyhedronVisual::display(bool display)
{
  if (mDisplay == display)
    return;

  mDisplay = display;

  if (mDisplay)
    addChild(mGeode);
  else
    removeChild(mGeode);
}

//==============================================================================
bool PolyhedronVisual::isDisplayed() const
{
  return mDisplay;
}

//==============================================================================
void PolyhedronVisual::displaySurface(bool display)
{
  if (mDisplaySurface == display)
    return;

  mDisplaySurface = display;

  if (mDisplaySurface)
    mGeode->addDrawable(mSurfaceGeom);
  else
    mGeode->removeDrawable(mSurfaceGeom);
}

//==============================================================================
bool PolyhedronVisual::isSurfaceDisplayed() const
{
  return mDisplaySurface;
}

//==============================================================================
void PolyhedronVisual::displayWireframe(bool display)
{
  if (mDisplayWireframe == display)
    return;

  mDisplayWireframe = display;

  if (mDisplayWireframe)
    mGeode->addDrawable(mWireframeGeom);
  else
    mGeode->removeDrawable(mWireframeGeom);
}

//==============================================================================
bool PolyhedronVisual::isWireframeDisplayed() const
{
  return mDisplayWireframe;
}

//==============================================================================
void PolyhedronVisual::refresh()
{
  if (!mDirty)
    return;

  updateGeometry();
}

//==============================================================================
void PolyhedronVisual::initialize()
{
  mGeode = new ::osg::Geode;
  addChild(mGeode);

  mSurfaceGeom = new ::osg::Geometry;
  mSurfaceGeom->setDataVariance(::osg::Object::DYNAMIC);
  mSurfaceVertices = new ::osg::Vec3Array;
  mSurfaceGeom->setVertexArray(mSurfaceVertices);
  mSurfaceNormals = new ::osg::Vec3Array;
  mSurfaceGeom->setNormalArray(mSurfaceNormals, ::osg::Array::BIND_PER_VERTEX);
  mSurfaceColor = new ::osg::Vec4Array;
  mSurfaceColor->resize(1);
  (*mSurfaceColor)[0] = ::osg::Vec4(0.1, 0.7, 0.9, 0.6);
  mSurfaceGeom->setColorArray(mSurfaceColor, ::osg::Array::BIND_OVERALL);
  mSurfaceIndices = new ::osg::DrawElementsUInt(GL_TRIANGLES, 0);
  mSurfaceGeom->addPrimitiveSet(mSurfaceIndices);

  mGeode->addDrawable(mSurfaceGeom);

  mWireframeGeom = new ::osg::Geometry;
  mWireframeGeom->setDataVariance(::osg::Object::DYNAMIC);
  mWireframeGeom->setVertexArray(mSurfaceVertices);
  mWireframeIndices = new ::osg::DrawElementsUInt(GL_LINES, 0);
  mWireframeGeom->addPrimitiveSet(mWireframeIndices);
  mWireframeColor = new ::osg::Vec4Array;
  mWireframeColor->resize(1);
  (*mWireframeColor)[0] = ::osg::Vec4(0.05, 0.05, 0.05, 1.0);
  mWireframeGeom->setColorArray(mWireframeColor, ::osg::Array::BIND_OVERALL);
  mWireframeWidth = new ::osg::LineWidth(mWireWidth);
  mWireframeGeom->getOrCreateStateSet()->setAttributeAndModes(
      mWireframeWidth, ::osg::StateAttribute::ON);

  mGeode->addDrawable(mWireframeGeom);
}

//==============================================================================
void PolyhedronVisual::updateGeometry()
{
  mDirty = false;

  if (mVertices.size() < 3u) {
    clearGeometry();
    return;
  }

  auto result = dart::math::computeConvexHull3D<double, std::size_t>(
      std::span<const Eigen::Vector3d>(mVertices), true);

  auto hullVertices = std::move(std::get<0>(result));
  auto hullTriangles = std::move(std::get<1>(result));

  if (hullVertices.size() < 3u || hullTriangles.empty()) {
    if (!mVertices.empty()) {
      DART_WARN(
          "[PolyhedronVisual::updateGeometry] Unable to compute a "
          "non-degenerate convex hull from {} vertices. Provide at least four "
          "non-coplanar points.",
          mVertices.size());
    }
    clearGeometry();
    return;
  }

  std::vector<Eigen::Vector3d> vertexNormals(
      hullVertices.size(), Eigen::Vector3d::Zero());

  mSurfaceVertices->resize(hullVertices.size());
  mSurfaceNormals->resize(hullVertices.size());
  for (std::size_t i = 0; i < hullVertices.size(); ++i) {
    const Eigen::Vector3d& v = hullVertices[i];
    (*mSurfaceVertices)[i] = ::osg::Vec3(v[0], v[1], v[2]);
  }

  mSurfaceIndices->clear();
  mWireframeIndices->clear();

  std::set<std::pair<std::size_t, std::size_t>> edges;

  for (const auto& tri : hullTriangles) {
    const std::size_t i0 = static_cast<std::size_t>(tri[0]);
    const std::size_t i1 = static_cast<std::size_t>(tri[1]);
    const std::size_t i2 = static_cast<std::size_t>(tri[2]);

    const Eigen::Vector3d& v0 = hullVertices[i0];
    const Eigen::Vector3d& v1 = hullVertices[i1];
    const Eigen::Vector3d& v2 = hullVertices[i2];
    const Eigen::Vector3d faceNormal = (v1 - v0).cross(v2 - v0);

    vertexNormals[i0] += faceNormal;
    vertexNormals[i1] += faceNormal;
    vertexNormals[i2] += faceNormal;

    mSurfaceIndices->push_back(static_cast<unsigned int>(i0));
    mSurfaceIndices->push_back(static_cast<unsigned int>(i1));
    mSurfaceIndices->push_back(static_cast<unsigned int>(i2));

    edges.insert(makeOrderedEdge(i0, i1));
    edges.insert(makeOrderedEdge(i1, i2));
    edges.insert(makeOrderedEdge(i2, i0));
  }

  for (std::size_t i = 0; i < vertexNormals.size(); ++i) {
    const Eigen::Vector3d normal = computeSafeNormal(vertexNormals[i]);
    (*mSurfaceNormals)[i] = ::osg::Vec3(normal[0], normal[1], normal[2]);
  }

  for (const auto& edge : edges) {
    mWireframeIndices->push_back(static_cast<unsigned int>(edge.first));
    mWireframeIndices->push_back(static_cast<unsigned int>(edge.second));
  }

  mSurfaceGeom->dirtyDisplayList();
  mSurfaceGeom->dirtyBound();
  mWireframeGeom->dirtyDisplayList();
  mWireframeGeom->dirtyBound();
}

//==============================================================================
void PolyhedronVisual::clearGeometry()
{
  mSurfaceVertices->clear();
  mSurfaceNormals->clear();
  mSurfaceIndices->clear();
  mWireframeIndices->clear();

  mSurfaceGeom->dirtyDisplayList();
  mSurfaceGeom->dirtyBound();
  mWireframeGeom->dirtyDisplayList();
  mWireframeGeom->dirtyBound();
}

//==============================================================================
void PolyhedronVisual::updateWireframeWidth()
{
  if (mWireframeWidth)
    mWireframeWidth->setWidth(mWireWidth);
}

} // namespace gui
} // namespace dart
