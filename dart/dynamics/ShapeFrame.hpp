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

#ifndef DART_DYNAMICS_SHAPEFRAME_HPP_
#define DART_DYNAMICS_SHAPEFRAME_HPP_

#include <Eigen/Dense>

#include "dart/common/Signal.hpp"
#include "dart/common/AddonWithVersion.hpp"
#include "dart/common/SpecializedForAddon.hpp"
#include "dart/dynamics/FixedFrame.hpp"
#include "dart/dynamics/TemplatedJacobianNode.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"

namespace dart {
namespace dynamics {

class VisualAddon;
class CollisionAddon;
class DynamicsAddon;
class ShapeFrame;

namespace detail {

struct VisualAddonProperties
{
  /// Color for the primitive shape
  Eigen::Vector4d mRGBA;

  bool mUseDefaultColor;

  /// True if this shape node should be kept from rendering
  bool mHidden;

  /// Constructor
  VisualAddonProperties(
      const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 1.0, 1.0),
      const bool hidden = false);

  /// Destructor
  virtual ~VisualAddonProperties() = default;
};

struct CollisionAddonProperties
{
  /// This object is collidable if true
  bool mCollidable;

  /// Constructor
  CollisionAddonProperties(const bool collidable = true);

  /// Destructor
  virtual ~CollisionAddonProperties() = default;
};

struct DynamicsAddonProperties
{
  /// Coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Constructor
  DynamicsAddonProperties(const double frictionCoeff = 1.0,
                          const double restitutionCoeff = 0.0);

  /// Destructor
  virtual ~DynamicsAddonProperties() = default;
};

} // namespace detail

class VisualAddon final :
    public common::AddonWithVersionedProperties<
        VisualAddon,
        detail::VisualAddonProperties,
        ShapeFrame>
{
public:

  using BaseClass = common::AddonWithVersionedProperties<
      VisualAddon, detail::VisualAddonProperties, ShapeFrame>;

  /// Constructor
  VisualAddon(common::AddonManager* mgr,
              const PropertiesData& properties = PropertiesData());

  VisualAddon(const VisualAddon&) = delete;

  /// Set RGBA color
  void setRGBA(const Eigen::Vector4d& color);

  DART_COMMON_GET_ADDON_PROPERTY( Eigen::Vector4d, RGBA )
  // void setRGBA(const Eigen::Vector4d& value);
  // const Eigen::Vector4d& getRGBA() const;

  DART_COMMON_SET_GET_ADDON_PROPERTY( bool, Hidden )
  // void setHidden(const Eigen::Vector4d& value);
  // const Eigen::Vector4d& getHidden() const;

  /// Identical to setRGB(const Eigen::Vector3d&)
  void setColor(const Eigen::Vector3d& color);

  /// Identical to setRGBA(const Eigen::Vector4d&)
  void setColor(const Eigen::Vector4d& color);

  /// Set RGB color components (leave alpha alone)
  void setRGB(const Eigen::Vector3d& rgb);

  /// Set the transparency of the Shape
  void setAlpha(const double alpha);

  /// Get color
  Eigen::Vector3d getColor() const;

  /// Get RGB color components
  Eigen::Vector3d getRGB() const;

  /// Get the transparency of the Shape
  double getAlpha() const;

  /// Hide the ShapeNode
  void hide();

  /// Show the ShapeNode
  void show();

  /// True iff the ShapeNode is set to be hidden. Use hide(bool) to change this
  /// setting
  bool isHidden() const;

};

class CollisionAddon final :
    public common::AddonWithVersionedProperties<
        CollisionAddon,
        detail::CollisionAddonProperties,
        ShapeFrame>
{
public:

  CollisionAddon(const CollisionAddon &) = delete;
  CollisionAddon(dart::common::AddonManager* mgr,
                      const PropertiesData& properties = PropertiesData());

  DART_COMMON_SET_GET_ADDON_PROPERTY( bool, Collidable )
  // void setCollidable(const bool& value);
  // const bool& getCollidable() const;

  /// Return true if this body can collide with others bodies
  bool isCollidable() const;

};

class DynamicsAddon final :
    public common::AddonWithVersionedProperties<
        DynamicsAddon,
        detail::DynamicsAddonProperties,
        ShapeFrame>
{
public:

  using BaseClass = common::AddonWithVersionedProperties<
      DynamicsAddon, detail::DynamicsAddonProperties, ShapeFrame>;

  DynamicsAddon(const DynamicsAddon&) = delete;

  DynamicsAddon(dart::common::AddonManager* mgr,
                const PropertiesData& properties = PropertiesData());

  DART_COMMON_SET_GET_ADDON_PROPERTY( double, FrictionCoeff )
  // void setFrictionCoeff(const double& value);
  // const double& getFrictionCoeff() const;
  DART_COMMON_SET_GET_ADDON_PROPERTY( double, RestitutionCoeff )
  // void setRestitutionCoeff(const double& value);
  // const double& getRestitutionCoeff() const;

};

class ShapeFrame :
    public virtual common::VersionCounter,
    public common::SpecializedForAddon<
        VisualAddon, CollisionAddon, DynamicsAddon>,
    public virtual Frame
{
public:

  friend class BodyNode;

  using ShapeUpdatedSignal
      = common::Signal<void(const ShapeFrame* thisShapeFrame,
                            const ShapePtr& oldShape,
                            const ShapePtr& newShape)>;

  using RelativeTransformUpdatedSignal
      = common::Signal<void(const ShapeFrame* thisShapeFrame,
                            const Eigen::Isometry3d& oldTransform,
                            const Eigen::Isometry3d& newTransform)>;

  using AddonProperties = common::AddonManager::Properties;

  struct UniqueProperties
  {
    /// Shape pointer
    ShapePtr mShape;

    size_t mVersion;

    /// Composed constructor
    UniqueProperties(const ShapePtr& shape = nullptr);

    /// Composed move constructor
    UniqueProperties(ShapePtr&& shape);

    virtual ~UniqueProperties() = default;
  };

  /// Composition of Entity and ShapeFrame properties
  struct Properties : Entity::Properties, UniqueProperties
  {
    /// Composed constructor
    Properties(const Entity::Properties& entityProperties
                   = Entity::Properties("ShapeFrame"),
               const UniqueProperties& shapeFrameProperties
                   = UniqueProperties(),
               const AddonProperties& addonProperties
                   = AddonProperties());

    virtual ~Properties() = default;

    /// Properties of all the Addons attached to this ShapeFrame
    AddonProperties mAddonProperties;
  };

  /// Destructor
  virtual ~ShapeFrame() = default;

  /// Same as setAddonProperties()
  void setProperties(const AddonProperties& properties);

  /// Set the Properties of this ShapeFrame
  void setProperties(const Properties& properties);

  /// Set the UniqueProperties of this ShapeFrame
  void setProperties(const UniqueProperties& properties);

  /// Get the Properties of this ShapeFrame
  const Properties getShapeFrameProperties() const;

  /// Copy the properties of another ShapeFrame
  void copy(const ShapeFrame& other);

  /// Copy the properties of another ShapeFrame
  void copy(const ShapeFrame* other);

  /// Set shape
  void setShape(const ShapePtr& shape);

  /// Return shape
  ShapePtr getShape();

  /// Return (const) shape
  ConstShapePtr getShape() const;

  DART_BAKE_SPECIALIZED_ADDON(VisualAddon)

  DART_BAKE_SPECIALIZED_ADDON(CollisionAddon)

  DART_BAKE_SPECIALIZED_ADDON(DynamicsAddon)

  /// Render this Entity
  virtual void draw(renderer::RenderInterface* ri = nullptr,
                    const Eigen::Vector4d& color = Eigen::Vector4d::Ones(),
                    bool useDefaultColor = true) const;

  // Documentation inherited
  size_t incrementVersion() override;

  // Documentation inherited
  size_t getVersion() const override;

  // Documentation inherited
  ShapeFrame* asShapeFrame() override;

  // Documentation inherited
  const ShapeFrame* asShapeFrame() const override;

  /// Returns true if this Frame is a ShapeNode
  bool isShapeNode() const;

  /// Convert 'this' into a ShapeNode pointer if ShapeFrame is a ShapeNode,
  /// otherwise return nullptr
  virtual ShapeNode* asShapeNode();

  /// Convert 'const this' into a ShapeNode pointer if ShapeFrame is a ShapeNode,
  /// otherwise return nullptr
  virtual const ShapeNode* asShapeNode() const;

protected:

  /// Constructor
  ShapeFrame(Frame* parent, const Properties& properties = Properties());

  /// Constructor
  ShapeFrame(Frame* parent,
             const std::string& name,
             const ShapePtr& shape = nullptr);

  /// ShapeFrame properties
  Properties mShapeFrameP;

  /// Contains whether or not this is a ShapeNode
  bool mAmShapeNode;

  /// Shape updated signal
  ShapeUpdatedSignal mShapeUpdatedSignal;

  /// Relative transformation updated signal
  RelativeTransformUpdatedSignal mRelativeTransformUpdatedSignal;

public:

  /// Slot register for shape updated signal
  common::SlotRegister<ShapeUpdatedSignal> onShapeUpdated;

  /// Slot register for relative transformation updated signal
  common::SlotRegister<RelativeTransformUpdatedSignal>
      onRelativeTransformUpdated;

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SHAPEFRAME_HPP_
