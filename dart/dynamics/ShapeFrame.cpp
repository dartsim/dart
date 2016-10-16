/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/ShapeFrame.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
VisualAspectProperties::VisualAspectProperties(const Eigen::Vector4d& color,
                                             const bool hidden)
  : mRGBA(color),
    mHidden(hidden)
{
  // Do nothing
}

//==============================================================================
CollisionAspectProperties::CollisionAspectProperties(
    const bool collidable)
  : mCollidable(collidable)
{
  // Do nothing
}

//==============================================================================
DynamicsAspectProperties::DynamicsAspectProperties(
    const double frictionCoeff,
    const double restitutionCoeff)
  :
    mFrictionCoeff(frictionCoeff),
    mRestitutionCoeff(restitutionCoeff)
{
  // Do nothing
}

//==============================================================================
ShapeFrameProperties::ShapeFrameProperties(const ShapePtr& shape)
  : mShape(shape)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
VisualAspect::VisualAspect(const PropertiesData& properties)
  : VisualAspect::Base(properties)
{
  // Do nothing
}

//==============================================================================
void VisualAspect::setRGBA(const Eigen::Vector4d& color)
{
  mProperties.mRGBA = color;

  notifyPropertiesUpdate();

  mComposite->getShape()->notifyColorUpdate(color);
}

//==============================================================================
void VisualAspect::setColor(const Eigen::Vector3d& color)
{
  setRGB(color);
}

//==============================================================================
void VisualAspect::setColor(const Eigen::Vector4d& color)
{
  setRGBA(color);
}

//==============================================================================
void VisualAspect::setRGB(const Eigen::Vector3d& rgb)
{
  Eigen::Vector4d rgba = getRGBA();
  rgba.head<3>() = rgb;

  setRGBA(rgba);
}

//==============================================================================
void VisualAspect::setAlpha(const double alpha)
{
  mProperties.mRGBA[3] = alpha;

  notifyPropertiesUpdate();

  mComposite->getShape()->notifyAlphaUpdate(alpha);
}

//==============================================================================
Eigen::Vector3d VisualAspect::getColor() const
{
  return getRGB();
}

//==============================================================================
Eigen::Vector3d VisualAspect::getRGB() const
{
  return getRGBA().head<3>();
}

//==============================================================================
double VisualAspect::getAlpha() const
{
  return getRGBA()[3];
}

//==============================================================================
void VisualAspect::hide()
{
  setHidden(true);
}

//==============================================================================
void VisualAspect::show()
{
  setHidden(false);
}

//==============================================================================
bool VisualAspect::isHidden() const
{
  return getHidden();
}

//==============================================================================
CollisionAspect::CollisionAspect(
    const PropertiesData& properties)
  : AspectImplementation(properties)
{
  // Do nothing
}

//==============================================================================
bool CollisionAspect::isCollidable() const
{
  return getCollidable();
}

//==============================================================================
DynamicsAspect::DynamicsAspect(
    const PropertiesData& properties)
  : Base(properties)
{
  // Do nothing
}

//==============================================================================
void ShapeFrame::setProperties(const ShapeFrame::UniqueProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void ShapeFrame::setAspectProperties(const AspectProperties& properties)
{
  setShape(properties.mShape);
}

//==============================================================================
const ShapeFrame::AspectProperties& ShapeFrame::getAspectProperties() const
{
  return mAspectProperties;
}

//==============================================================================
void ShapeFrame::setShape(const ShapePtr& shape)
{
  if (shape == ShapeFrame::mAspectProperties.mShape)
    return;

  ShapePtr oldShape = ShapeFrame::mAspectProperties.mShape;

  ShapeFrame::mAspectProperties.mShape = shape;

  mShapeUpdatedSignal.raise(this, oldShape, ShapeFrame::mAspectProperties.mShape);
}

//==============================================================================
ShapePtr ShapeFrame::getShape()
{
  return ShapeFrame::mAspectProperties.mShape;
}

//==============================================================================
ConstShapePtr ShapeFrame::getShape() const
{
  return ShapeFrame::mAspectProperties.mShape;
}

//==============================================================================
ShapeFrame* ShapeFrame::asShapeFrame()
{
  return this;
}

//==============================================================================
const ShapeFrame* ShapeFrame::asShapeFrame() const
{
  return this;
}

//==============================================================================
bool ShapeFrame::isShapeNode() const
{
  return mAmShapeNode;
}

//==============================================================================
ShapeNode* ShapeFrame::asShapeNode()
{
  return nullptr;
}

//==============================================================================
const ShapeNode* ShapeFrame::asShapeNode() const
{
  return nullptr;
}

//==============================================================================
ShapeFrame::ShapeFrame(Frame* parent, const Properties& properties)
  : common::Composite(),
    Entity(ConstructFrame),
    Frame(parent),
    mAmShapeNode(false),
    mShapeUpdatedSignal(),
    mRelativeTransformUpdatedSignal(),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  createAspect<Aspect>();
  mAmShapeFrame = true;
  setProperties(properties);
}

//==============================================================================
ShapeFrame::ShapeFrame(Frame* parent,
                       const ShapePtr& shape)
  : common::Composite(),
    Entity(ConstructFrame),
    Frame(parent),
    mAmShapeNode(false),
    mShapeUpdatedSignal(),
    mRelativeTransformUpdatedSignal(),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  createAspect<Aspect>();
  mAmShapeFrame = true;
  setShape(shape);
}

//==============================================================================
ShapeFrame::ShapeFrame(const std::tuple<Frame*, Properties>& args)
  : ShapeFrame(std::get<0>(args), std::get<1>(args))
{
  // Delegating constructor
}

} // namespace dynamics
} // namespace dart

