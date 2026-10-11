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

#include <dart/gui/SupportPolygonVisual.hpp>
#include <dart/gui/detail/geometry.hpp>
#include <dart/gui/detail/overlay_adapter.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/Geometry.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <cmath>

namespace dart::gui {
struct SupportPolygonVisual::Impl
{
  dynamics::WeakSkeletonPtr skeleton;
  std::size_t tree = std::numeric_limits<std::size_t>::max();
  double elevation = 0.02, centroidRadius = 0.12, comRadius = 0.12;
  bool polygonShown = true, centroidShown = true, comShown = true;
  Eigen::Vector4d polygonColor{0.1, 0.9, 0.1, 1}, validColor{0, 0, 1, 1},
      invalidColor{1, 0, 0, 1};
  DebugScene scene;
  MeshGeometry centroidMesh, comMesh;
  bool dirty = true;
  math::SupportPolygon lastPolygon;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> lastAxes;
  Eigen::Vector3d lastCOM = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastCentroid = Eigen::Vector3d::Zero();
};
SupportPolygonVisual::SupportPolygonVisual(
    const dynamics::SkeletonPtr& skeleton, double elevation)
  : mImpl(std::make_unique<Impl>())
{
  setSkeleton(skeleton);
  setDisplayElevation(elevation);
  setCentroidRadius(mImpl->centroidRadius);
  setCenterOfMassRadius(mImpl->comRadius);
}
SupportPolygonVisual::SupportPolygonVisual(
    const dynamics::SkeletonPtr& skeleton, std::size_t tree, double elevation)
  : SupportPolygonVisual(skeleton, elevation)
{
  visualizeTree(tree);
}
SupportPolygonVisual::~SupportPolygonVisual() = default;
void SupportPolygonVisual::setSkeleton(const dynamics::SkeletonPtr& skeleton)
{
  mImpl->skeleton = skeleton;
  mImpl->dirty = true;
}
dynamics::SkeletonPtr SupportPolygonVisual::getSkeleton() const
{
  return mImpl->skeleton.lock();
}
void SupportPolygonVisual::visualizeWholeSkeleton()
{
  mImpl->tree = std::numeric_limits<std::size_t>::max();
  mImpl->dirty = true;
}
void SupportPolygonVisual::visualizeTree(std::size_t tree)
{
  mImpl->tree = tree;
  mImpl->dirty = true;
}
void SupportPolygonVisual::setDisplayElevation(double value)
{
  if (!std::isfinite(value))
    throw std::invalid_argument("Support elevation must be finite");
  mImpl->elevation = value;
  mImpl->dirty = true;
}
double SupportPolygonVisual::getDisplayElevation() const
{
  return mImpl->elevation;
}
void SupportPolygonVisual::displayPolygon(bool value)
{
  mImpl->polygonShown = value;
  mImpl->dirty = true;
}
bool SupportPolygonVisual::isPolygonDisplayed() const
{
  return mImpl->polygonShown;
}
void SupportPolygonVisual::setPolygonColor(const Eigen::Vector4d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("Support color must be finite");
  mImpl->polygonColor = value;
  mImpl->dirty = true;
}
Eigen::Vector4d SupportPolygonVisual::getPolygonColor() const
{
  return mImpl->polygonColor;
}
void SupportPolygonVisual::displayCentroid(bool value)
{
  mImpl->centroidShown = value;
  mImpl->dirty = true;
}
bool SupportPolygonVisual::isCentroidDisplayed() const
{
  return mImpl->centroidShown;
}
void SupportPolygonVisual::setCentroidRadius(double value)
{
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(
        "Support marker radius must be finite and positive");
  mImpl->centroidRadius = value;
  mImpl->dirty = true;
  mImpl->centroidMesh
      = makeEllipsoidMeshGeometry(Eigen::Vector3d::Constant(value / 4));
}
double SupportPolygonVisual::getCentroidRadius() const
{
  return mImpl->centroidRadius;
}
void SupportPolygonVisual::displayCenterOfMass(bool value)
{
  mImpl->comShown = value;
  mImpl->dirty = true;
}
bool SupportPolygonVisual::isCenterOfMassDisplayed() const
{
  return mImpl->comShown;
}
void SupportPolygonVisual::setCenterOfMassRadius(double value)
{
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(
        "Support marker radius must be finite and positive");
  mImpl->comRadius = value;
  mImpl->dirty = true;
  mImpl->comMesh
      = makeEllipsoidMeshGeometry(Eigen::Vector3d::Constant(value / 4));
}
double SupportPolygonVisual::getCenterOfMassRadius() const
{
  return mImpl->comRadius;
}
void SupportPolygonVisual::setValidCOMColor(const Eigen::Vector4d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("Support color must be finite");
  mImpl->validColor = value;
  mImpl->dirty = true;
}
const Eigen::Vector4d& SupportPolygonVisual::getValidCOMColor() const
{
  return mImpl->validColor;
}
void SupportPolygonVisual::setInvalidCOMColor(const Eigen::Vector4d& value)
{
  if (!value.allFinite())
    throw std::invalid_argument("Support color must be finite");
  mImpl->invalidColor = value;
  mImpl->dirty = true;
}
const Eigen::Vector4d& SupportPolygonVisual::getInvalidCOMColor() const
{
  return mImpl->invalidColor;
}
void SupportPolygonVisual::refresh()
{
  auto& d = *mImpl;
  const auto skeleton = d.skeleton.lock();
  const bool whole = d.tree == std::numeric_limits<std::size_t>::max();
  if (!skeleton || skeleton->getNumBodyNodes() == 0 || skeleton->getMass() <= 0
      || (!whole && d.tree >= skeleton->getNumTrees())) {
    d.scene.triangles.clear();
    d.dirty = true;
    return;
  }
  const auto& polygon = whole ? skeleton->getSupportPolygon()
                              : skeleton->getSupportPolygon(d.tree);
  const auto& axes
      = whole ? skeleton->getSupportAxes() : skeleton->getSupportAxes(d.tree);
  const Eigen::Vector3d offset = axes.first.cross(axes.second) * d.elevation;
  auto point = [&](const Eigen::Vector2d& p) -> Eigen::Vector3d {
    return axes.first * p.x() + axes.second * p.y() + offset;
  };
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  if (whole)
    com = skeleton->getCOM();
  else {
    double mass = 0;
    for (const auto* body : skeleton->getTreeBodyNodes(d.tree)) {
      com += body->getCOM() * body->getMass();
      mass += body->getMass();
    }
    if (mass <= 0) {
      d.scene.triangles.clear();
      d.dirty = true;
      return;
    }
    com /= mass;
  }
  const Eigen::Vector3d centroid
      = polygon.empty() ? skeleton->getCOM()
                        : point(
                            whole ? skeleton->getSupportCentroid()
                                  : skeleton->getSupportCentroid(d.tree));
  const bool samePolygon
      = polygon.size() == d.lastPolygon.size()
        && std::equal(
            polygon.begin(),
            polygon.end(),
            d.lastPolygon.begin(),
            [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
              return a.isApprox(b, 0.0);
            });
  if (!d.dirty && samePolygon && axes.first.isApprox(d.lastAxes.first, 0.0)
      && axes.second.isApprox(d.lastAxes.second, 0.0)
      && com.isApprox(d.lastCOM, 0.0) && centroid.isApprox(d.lastCentroid, 0.0))
    return;
  d.lastPolygon = polygon;
  d.lastAxes = axes;
  d.lastCOM = com;
  d.lastCentroid = centroid;
  d.dirty = false;
  d.scene.triangles.clear();
  auto triangle = [&](const Eigen::Vector3d& a,
                      const Eigen::Vector3d& b,
                      const Eigen::Vector3d& c,
                      const Eigen::Vector4d& rgba) {
    DebugTriangleDescriptor t;
    t.a = a;
    t.b = b;
    t.c = c;
    t.rgba = rgba;
    t.alwaysOnTop = false;
    d.scene.triangles.push_back(std::move(t));
  };
  if (d.polygonShown)
    for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
      triangle(
          point(polygon.front()),
          point(polygon[i]),
          point(polygon[i + 1]),
          d.polygonColor);
  auto sphere = [&](const MeshGeometry& mesh,
                    const Eigen::Vector3d& center,
                    const Eigen::Vector4d& rgba) {
    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
      triangle(
          mesh.vertices[mesh.indices[i]].position.cast<double>() + center,
          mesh.vertices[mesh.indices[i + 1]].position.cast<double>() + center,
          mesh.vertices[mesh.indices[i + 2]].position.cast<double>() + center,
          rgba);
  };
  if (d.centroidShown) {
    sphere(d.centroidMesh, centroid, d.polygonColor);
  }
  if (d.comShown) {
    const Eigen::Vector2d projected(com.dot(axes.first), com.dot(axes.second));
    sphere(
        d.comMesh,
        point(projected),
        math::isInsideSupportPolygon(projected, polygon) ? d.validColor
                                                         : d.invalidColor);
  }
}
void detail::OverlayAccess::append(
    const SupportPolygonVisual& visual, DebugScene& scene)
{
  scene.triangles.insert(
      scene.triangles.end(),
      visual.mImpl->scene.triangles.begin(),
      visual.mImpl->scene.triangles.end());
}
} // namespace dart::gui
