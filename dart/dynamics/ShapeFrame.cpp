/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/dynamics/ShapeFrame.h"

#include "dart/renderer/OpenGLRenderInterface.h"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
void VisualShapeFrameAddonUpdate(VisualAddon* /*addon*/)
{
  // Do nothing
}

//==============================================================================
void CollisionShapeFrameAddonUpdate(CollisionAddon* /*addon*/)
{
  // Do nothing
}

//==============================================================================
void DynamicsShapeFrameAddonUpdate(DynamicsAddon* /*addon*/)
{
  // Do nothing
}

//==============================================================================
VisualDataProperties::VisualDataProperties(const Eigen::Vector4d& color,
                                           const bool hidden)
  : mRGBA(color),
    mHidden(hidden)
{
  // Do nothing
}

//==============================================================================
CollisionDataProperties::CollisionDataProperties(
    const bool collidable)
  : mCollidable(collidable)
{
  // Do nothing
}

//==============================================================================
DynamicsDataProperties::DynamicsDataProperties(
    const double frictionCoeff,
    const double restitutionCoeff)
  :
    mFrictionCoeff(frictionCoeff),
    mRestitutionCoeff(restitutionCoeff)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
VisualAddon::VisualAddon(common::AddonManager* mgr,
                         const PropertiesData& properties)
  : AddonWithProtectedProperties<
      VisualAddon,
      detail::VisualDataProperties,
      ShapeFrame,
      &detail::VisualShapeFrameAddonUpdate>(mgr, properties),
    mShapeFrame(dynamic_cast<ShapeFrame*>(mgr))
{
  assert(mShapeFrame);
}

//==============================================================================
void VisualAddon::setRGBA(const Eigen::Vector4d& color)
{
  mProperties.mRGBA = color;

  UpdateProperties(this);

  mShapeFrame->getShape()->notifyColorUpdate(color);
}

//==============================================================================
void VisualAddon::setColor(const Eigen::Vector3d& color)
{
  setRGB(color);
}

//==============================================================================
void VisualAddon::setColor(const Eigen::Vector4d& color)
{
  setRGBA(color);
}

//==============================================================================
void VisualAddon::setRGB(const Eigen::Vector3d& rgb)
{
  Eigen::Vector4d rgba = getRGBA();
  rgba.head<3>() = rgb;

  setRGBA(rgba);
}

//==============================================================================
void VisualAddon::setAlpha(const double alpha)
{
  Eigen::Vector4d rgba = getRGBA();
  rgba[3] = alpha;

  setRGBA(rgba);
}

//==============================================================================
Eigen::Vector3d VisualAddon::getColor() const
{
  return getRGB();
}

//==============================================================================
Eigen::Vector3d VisualAddon::getRGB() const
{
  return getRGBA().head<3>();
}

//==============================================================================
const double VisualAddon::getAlpha() const
{
  return getRGBA()[3];
}

//==============================================================================
void VisualAddon::hide()
{
  setHidden(false);
}

//==============================================================================
void VisualAddon::show()
{
  setHidden(true);
}

//==============================================================================
bool VisualAddon::isHidden() const
{
  return getHidden();
}

//==============================================================================
CollisionAddon::CollisionAddon(
    common::AddonManager* mgr,
    const AddonType::PropertiesData& properties)
  : AddonType(mgr, properties)
{
//  mIsShapeNode = dynamic_cast<ShapeNode*>(mgr);
}

//==============================================================================
bool CollisionAddon::isCollidable() const
{
  return getCollidable();
}

//==============================================================================
ShapeFrame::UniqueProperties::UniqueProperties(const ShapePtr& shape)
  : mShape(shape)
{
  // Do nothing
}

//==============================================================================
ShapeFrame::UniqueProperties::UniqueProperties(ShapePtr&& shape)
  : mShape(std::move(shape))
{
  // Do nothing
}

//==============================================================================
ShapeFrame::Properties::Properties(
    const Entity::Properties& entityProperties,
    const UniqueProperties& shapeFrameProperties,
    const ShapeFrame::AddonProperties& addonProperties)
  : Entity::Properties(entityProperties),
    UniqueProperties(shapeFrameProperties),
    mAddonProperties(addonProperties)
{
  // Do nothing
}

//==============================================================================
void ShapeFrame::setProperties(const ShapeFrame::AddonProperties& properties)
{
  setAddonProperties(properties);
}

//==============================================================================
void ShapeFrame::setProperties(const Properties& properties)
{
  Entity::setProperties(static_cast<const Entity::Properties&>(properties));
  setProperties(static_cast<const UniqueProperties&>(properties));
  setProperties(properties.mAddonProperties);
}

//==============================================================================
void ShapeFrame::setProperties(const ShapeFrame::UniqueProperties& properties)
{
  setShape(properties.mShape);
}

//==============================================================================
const ShapeFrame::Properties ShapeFrame::getShapeFrameProperties() const
{
  return Properties(getEntityProperties(), mShapeFrameP, getAddonProperties());
}

//==============================================================================
void ShapeFrame::copy(const ShapeFrame& other)
{
  if (this == &other)
    return;

  setProperties(other.getShapeFrameProperties());
}

//==============================================================================
void ShapeFrame::copy(const ShapeFrame* other)
{
  if (nullptr == other)
    return;

  copy(*other);
}

//==============================================================================
ShapeFrame& ShapeFrame::operator=(const ShapeFrame& other)
{
  copy(other);
  return *this;
}

//==============================================================================
void ShapeFrame::setShape(const ShapePtr& shape)
{
  if (nullptr == shape)
  {
    dtwarn << "[ShapeFrame::setShape] Attempting to add a nullptr as a "
           << " shape of ShapeNode '" << getName()
           << "', which is not allowed. "
           << "The shape of ShapeNode '" << getName() << "' hasn't changed.\n";
    return;
  }

  if (shape == mShapeFrameP.mShape)
    return;

  ShapePtr oldShape = mShapeFrameP.mShape;

  mShapeFrameP.mShape = shape;

  mShapeUpdatedSignal.raise(this, oldShape, mShapeFrameP.mShape);
}

//==============================================================================
ShapePtr ShapeFrame::getShape()
{
  return mShapeFrameP.mShape;
}

//==============================================================================
ConstShapePtr ShapeFrame::getShape() const
{
  return mShapeFrameP.mShape;
}

//==============================================================================
VisualAddon* ShapeFrame::getVisualAddon(const bool createIfNull)
{
  VisualAddon* visualData = getVisualAddon();

  if (createIfNull && nullptr == visualData)
    return createVisualAddon();

  return visualData;
}

//==============================================================================
CollisionAddon* ShapeFrame::getCollisionAddon(const bool createIfNull)
{
  CollisionAddon* collisionData = getCollisionAddon();

  if (createIfNull && nullptr == collisionData)
    return createCollisionAddon();

  return collisionData;
}

//==============================================================================
DynamicsAddon* ShapeFrame::getDynamicsShapeNode(const bool createIfNull)
{
  DynamicsAddon* dynamicsData = getDynamicsAddon();

  if (createIfNull && nullptr == dynamicsData)
    return createDynamicsAddon();

  return dynamicsData;
}

//==============================================================================
void ShapeFrame::draw(renderer::RenderInterface* ri,
                     const Eigen::Vector4d& color,
                     bool useDefaultColor) const
{
  auto visualData = getVisualAddon();

  if (!visualData || visualData->isHidden())
    return;

  ri->pushMatrix();
  ri->transform(getRelativeTransform());

  if (useDefaultColor)
    mShapeFrameP.mShape->draw(ri, visualData->getRGBA());
  else
    mShapeFrameP.mShape->draw(ri, color);

  ri->popMatrix();
}

//==============================================================================
ShapeFrame::ShapeFrame(Frame* parent, const Properties& properties)
  : common::AddonManager(),
    Entity(ConstructFrame),
    Frame(parent, ""),
    mShapeUpdatedSignal(),
    mRelativeTransformUpdatedSignal(),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  setProperties(properties);
}

//==============================================================================
ShapeFrame::ShapeFrame(Frame* parent,
                       const std::string& name,
                       const ShapePtr& shape)
  : common::AddonManager(),
    Entity(ConstructFrame),
    Frame(parent, name),
    mShapeUpdatedSignal(),
    mRelativeTransformUpdatedSignal(),
    onShapeUpdated(mShapeUpdatedSignal),
    onRelativeTransformUpdated(mRelativeTransformUpdatedSignal)
{
  setName(name);
  setShape(shape);
}

} // namespace dynamics
} // namespace dart

