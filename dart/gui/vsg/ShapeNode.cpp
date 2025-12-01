/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/gui/vsg/ShapeNode.hpp"

#include <dart/common/Console.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <utility>

namespace dart::gui::vsg {

namespace {

Eigen::Vector3d sanitizeExtent(const Eigen::Vector3d& extent)
{
  const double min_extent = 1e-4;
  return extent.cwiseMax(Eigen::Vector3d::Constant(min_extent));
}

} // namespace

ShapeNode::ShapeNode(
    dart::dynamics::ShapeFrame* frame, ::vsg::ref_ptr<::vsg::Builder> builder)
  : mFrame(frame),
    mBuilder(std::move(builder)),
    mTransform(::vsg::MatrixTransform::create()),
    mUtilized(false)
{
  if (!mBuilder) {
    mBuilder = ::vsg::Builder::create();
  }
}

::vsg::ref_ptr<::vsg::MatrixTransform> ShapeNode::getNode() const
{
  return mTransform;
}

void ShapeNode::clearUtilization()
{
  mUtilized = false;
}

bool ShapeNode::wasUtilized() const
{
  return mUtilized;
}

void ShapeNode::refresh()
{
  mUtilized = true;

  if (!mFrame) {
    return;
  }

  const auto shape = mFrame->getShape();
  if (!shape) {
    mTransform->children.clear();
    return;
  }

  const auto* visualAspect
      = mFrame->hasVisualAspect() ? mFrame->getVisualAspect() : nullptr;
  if (visualAspect && visualAspect->isHidden()) {
    mTransform->children.clear();
    return;
  }

  const Eigen::Vector4d rgba = visualAspect
                                   ? visualAspect->getRGBA()
                                   : dart::dynamics::VisualAspect::getDefaultRGBA();

  const bool shapeChanged = mCachedShape.lock() != shape;
  const bool colorChanged = !mLastColor.isApprox(rgba);
  if (shapeChanged || colorChanged || !mGeometry) {
    mGeometry = buildGeometry(shape, rgba);
    mTransform->children.clear();
    if (mGeometry) {
      mTransform->addChild(mGeometry);
    }
    mCachedShape = shape;
    mLastColor = rgba;
  }

  mTransform->matrix = convertTransform(mFrame->getWorldTransform());
}

::vsg::ref_ptr<::vsg::Node> ShapeNode::buildGeometry(
    const std::shared_ptr<dart::dynamics::Shape>& shape,
    const Eigen::Vector4d& rgba)
{
  if (!shape || !mBuilder) {
    return {};
  }

  ::vsg::GeometryInfo info;
  ::vsg::StateInfo state;
  info.color = toVec4(rgba);
  state.blending = rgba[3] < 0.999;

  if (auto box = std::dynamic_pointer_cast<dart::dynamics::BoxShape>(shape)) {
    const Eigen::Vector3d extent = sanitizeExtent(box->getSize());
    info.dx.set(static_cast<float>(extent[0]), 0.0f, 0.0f);
    info.dy.set(0.0f, static_cast<float>(extent[1]), 0.0f);
    info.dz.set(0.0f, 0.0f, static_cast<float>(extent[2]));
    return mBuilder->createBox(info, state);
  }

  if (auto sphere
      = std::dynamic_pointer_cast<dart::dynamics::SphereShape>(shape)) {
    const float diameter = static_cast<float>(sphere->getRadius() * 2.0);
    info.dx.set(diameter, 0.0f, 0.0f);
    info.dy.set(0.0f, diameter, 0.0f);
    info.dz.set(0.0f, 0.0f, diameter);
    return mBuilder->createSphere(info, state);
  }

  if (auto cylinder
      = std::dynamic_pointer_cast<dart::dynamics::CylinderShape>(shape)) {
    const float diameter = static_cast<float>(cylinder->getRadius() * 2.0);
    const float height = static_cast<float>(cylinder->getHeight());
    info.dx.set(diameter, 0.0f, 0.0f);
    info.dy.set(0.0f, diameter, 0.0f);
    info.dz.set(0.0f, 0.0f, height);
    return mBuilder->createCylinder(info, state);
  }

  if (auto capsule
      = std::dynamic_pointer_cast<dart::dynamics::CapsuleShape>(shape)) {
    const float diameter = static_cast<float>(capsule->getRadius() * 2.0);
    const float height = static_cast<float>(capsule->getHeight());
    info.dx.set(diameter, 0.0f, 0.0f);
    info.dy.set(0.0f, diameter, 0.0f);
    info.dz.set(0.0f, 0.0f, height);
    return mBuilder->createCapsule(info, state);
  }

  if (auto ellipsoid
      = std::dynamic_pointer_cast<dart::dynamics::EllipsoidShape>(shape)) {
    const Eigen::Vector3d extent = sanitizeExtent(ellipsoid->getDiameters());
    info.dx.set(static_cast<float>(extent[0]), 0.0f, 0.0f);
    info.dy.set(0.0f, static_cast<float>(extent[1]), 0.0f);
    info.dz.set(0.0f, 0.0f, static_cast<float>(extent[2]));
    return mBuilder->createSphere(info, state);
  }

  const auto& bbox = shape->getBoundingBox();
  const Eigen::Vector3d extent
      = sanitizeExtent(bbox.getMax() - bbox.getMin());
  DART_WARN(
      "VSG GUI backend does not implement Shape type [{}]; rendering a "
      "bounding box instead.",
      shape->getType());
  info.dx.set(static_cast<float>(extent[0]), 0.0f, 0.0f);
  info.dy.set(0.0f, static_cast<float>(extent[1]), 0.0f);
  info.dz.set(0.0f, 0.0f, static_cast<float>(extent[2]));
  return mBuilder->createBox(info, state);
}

} // namespace dart::gui::vsg
