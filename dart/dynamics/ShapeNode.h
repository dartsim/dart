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

#ifndef DART_DYNAMICS_SHAPENODE_H_
#define DART_DYNAMICS_SHAPENODE_H_

#include <Eigen/Dense>

#include "dart/common/Signal.h"
#include "dart/common/SpecializedAddonManager.h"
#include "dart/dynamics/Addon.h"
#include "dart/dynamics/Node.h"
#include "dart/dynamics/FixedFrame.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace dynamics {

class VisualData;
class CollisionData;
class DynamicsData;
class ShapeNode;

//==============================================================================
// Addon data
//==============================================================================

namespace detail
{

void VisualUpdate(VisualData* visualData);
void CollisionUpdate(CollisionData* collisionData);
void DynamicsUpdate(DynamicsData* dynamicsData);

struct VisualDataProperties
{
  /// Color for the primitive shape
  Eigen::Vector4d mRGBA;

  /// True if this shape node should be kept from rendering
  bool mHidden;

  /// Constructor
  VisualDataProperties(
      const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 1.0, 1.0),
      const bool hidden = false);

  /// Destructor
  virtual ~VisualDataProperties() = default;
};

struct CollisionDataProperties
{
  /// This object is collidable if true
  bool mCollidable;

  /// Constructor
  CollisionDataProperties(const bool collidable = true);

  /// Destructor
  virtual ~CollisionDataProperties() = default;
};

struct DynamicsDataProperties
{
  /// Coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Constructor
  DynamicsDataProperties(const double frictionCoeff = 1.0,
                         const double restitutionCoeff = 0.0);

  /// Destructor
  virtual ~DynamicsDataProperties() = default;
};

} // namespace detail


//==============================================================================
// Addons
//==============================================================================

class VisualData final :
    public AddonWithProtectedPropertiesInSkeleton<
        VisualData,
        detail::VisualDataProperties,
        ShapeNode,
        &detail::VisualUpdate,
        true>
{
public:

  DART_DYNAMICS_ADDON_PROPERTY_CONSTRUCTOR( VisualData, &detail::VisualUpdate )

  // This macro defines following setter/getter of the addon property obeying
  // the skeleton version increment mechanism.
  DART_DYNAMICS_SET_GET_ADDON_PROPERTY( Eigen::Vector4d, RGBA )
  // void setRGBA(const Eigen::Vector4d& value);
  // const Eigen::Vector4d& getRGBA() const;
  DART_DYNAMICS_SET_GET_ADDON_PROPERTY( bool, Hidden )
  // void setRGBA(const Eigen::Vector4d& value);
  // const Eigen::Vector4d& getRGBA() const;

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
  const double getAlpha() const;

  /// Hide the ShapeNode
  void hide();

  /// Show the ShapeNode
  void show();

  /// True iff the ShapeNode is set to be hidden. Use hide(bool) to change this
  /// setting
  bool isHidden() const;

};

class CollisionData final :
    public AddonWithProtectedPropertiesInSkeleton<
        CollisionData,
        detail::CollisionDataProperties,
        ShapeNode,
        &detail::CollisionUpdate,
        true>
{
public:

  DART_DYNAMICS_ADDON_PROPERTY_CONSTRUCTOR( CollisionData,
                                            &detail::CollisionUpdate )

  // This macro defines following setter/getter of the addon property obeying
  // the skeleton version increment mechanism.
  DART_DYNAMICS_SET_GET_ADDON_PROPERTY( bool, Collidable )
  // void setCollisionMode(const bool& value);
  // const bool& getCollisionMode() const;

  /// Return true if this body can collide with others bodies
  bool isCollidable() const;

};

class DynamicsData final :
    public AddonWithProtectedPropertiesInSkeleton<
        DynamicsData,
        detail::DynamicsDataProperties,
        ShapeNode,
        &detail::DynamicsUpdate,
        true>
{
public:

  DART_DYNAMICS_ADDON_PROPERTY_CONSTRUCTOR( DynamicsData,
                                            &detail::DynamicsUpdate )

  // This macro defines following setter/getter of the addon property obeying
  // the skeleton version increment mechanism.
  DART_DYNAMICS_SET_GET_ADDON_PROPERTY( double, FrictionCoeff )
  // void setFrictionCoeff(const double& value);
  // const double& getFrictionCoeff() const;
  DART_DYNAMICS_SET_GET_ADDON_PROPERTY( double, RestitutionCoeff )
  // void setRestitutionCoeff(const double& value);
  // const double& getRestitutionCoeff() const;

};

//==============================================================================
// ShapeNode
//==============================================================================

class ShapeNode final :
    public virtual common::SpecializedAddonManager<
        VisualData, CollisionData, DynamicsData>,
    public FixedFrame,
    public AccessoryNode<ShapeNode>,
    public TemplatedJacobianNode<ShapeNode>
{
public:

  friend class BodyNode;

  using ShapeUpdatedSignal
      = common::Signal<void(const ShapeNode* thisShapeNode,
                            const ShapePtr& oldShape,
                            const ShapePtr& newShape)>;

  using RelativeTransformUpdatedSignal
      = common::Signal<void(const ShapeNode* thisShapeNode,
                            const Eigen::Isometry3d& oldTransform,
                            const Eigen::Isometry3d& newTransform)>;

  struct UniqueProperties
  {
    ShapePtr mShape;

    /// Transformation of this shape node relative to the parent frame
    Eigen::Isometry3d mTransform;

    /// Constructor
    UniqueProperties(const ShapePtr& shape = nullptr,  // TODO(JS)
               const Eigen::Isometry3d tf = Eigen::Isometry3d::Identity(),
               const bool hidden = false);

    virtual ~UniqueProperties() = default;

    // To get byte-aligned Eigen vectors
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using AddonProperties = common::AddonManager::Properties;

  struct Properties : Frame::Properties, UniqueProperties
  {
    /// Composed constructor
    Properties(
        const UniqueProperties& standardProperties = UniqueProperties(),
        const AddonProperties& addonProperties = AddonProperties());

    /// Composed move constructor
    Properties(
        const UniqueProperties&& standardProperties,
        const AddonProperties&& addonProperties);

    /// The properties of the ShapeNode's Addons
    AddonProperties mAddonProperties;
  };

  /// Destructor
  virtual ~ShapeNode() = default;

  /// Set the Properties of this ShapeNode
  void setProperties(const UniqueProperties& properties);

  /// Get the Properties of this ShapeNode
  const UniqueProperties getProperties() const;

  /// Copy the properties of another ShapeNode
  void copy(const ShapeNode& other);

  /// Copy the properties of another ShapeNode
  void copy(const ShapeNode* other);

  /// Same as copy(const ShapeNode&)
  ShapeNode& operator=(const ShapeNode& other);

  // Documentation inherited
  const std::string& setName(const std::string& newName) override;

  /// Set shape
  void setShape(const ShapePtr& shape);

  /// Return shape
  ShapePtr getShape();

  /// Return (const) shape
  ConstShapePtr getShape() const;

  /// Set transformation of this shape node relative to the parent frame
  void setRelativeTransform(const Eigen::Isometry3d& transform);

  /// Get transformation of this shape node relative to the parent frame
  const Eigen::Isometry3d& getRelativeTransform() const;

  /// Set rotation of this shape node relative to the parent frame
  void setRelativeRotation(const Eigen::Matrix3d& rotation);

  /// Get rotation of this shape node relative to the parent frame
  Eigen::Matrix3d getRelativeRotation() const;

  /// Set translation of this shape node relative to the parent frame
  void setRelativeTranslation(const Eigen::Vector3d& translation);

  /// Same as setRelativeTranslation(offset)
  void setOffset(const Eigen::Vector3d& offset);

  /// Get translation of this shape node relative to the parent frame
  Eigen::Vector3d getRelativeTranslation() const;

  /// Same as getRelativeTranslation()
  Eigen::Vector3d getOffset() const;

  // Documentation inherited
  std::shared_ptr<Skeleton> getSkeleton() override;

  // Documentation inherited
  std::shared_ptr<const Skeleton> getSkeleton() const override;

  // Documentation inherited
  bool dependsOn(size_t genCoordIndex) const override;

  // Documentation inherited
  size_t getNumDependentGenCoords() const override;

  // Documentation inherited
  size_t getDependentGenCoordIndex(size_t arrayIndex) const override;

  // Documentation inherited
  const std::vector<size_t>& getDependentGenCoordIndices() const override;

  // Documentation inherited
  size_t getNumDependentDofs() const override;

  // Documentation inherited
  DegreeOfFreedom* getDependentDof(size_t index) override;

  // Documentation inherited
  const DegreeOfFreedom* getDependentDof(size_t index) const override;

  // Documentation inherited
  const std::vector<DegreeOfFreedom*>& getDependentDofs() override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*>& getDependentDofs() const override;

  // Documentation inherited
  const std::vector<const DegreeOfFreedom*> getChainDofs() const override;

  //----------------------------------------------------------------------------
  /// \{ \name Jacobian Functions
  //----------------------------------------------------------------------------

  // Documentation inherited
  const math::Jacobian& getJacobian() const override final;

  // Prevent the inherited getJacobian functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getJacobian;

  // Documentation inherited
  const math::Jacobian& getWorldJacobian() const override final;

  // Prevent the inherited getWorldJacobian functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getWorldJacobian;

  // Documentation inherited
  const math::Jacobian& getJacobianSpatialDeriv() const override final;

  // Prevent the inherited getJacobianSpatialDeriv functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getJacobianSpatialDeriv;

  // Documentation inherited
  const math::Jacobian& getJacobianClassicDeriv() const override final;

  // Prevent the inherited getJacobianClassicDeriv functions from being shadowed
  using TemplatedJacobianNode<ShapeNode>::getJacobianClassicDeriv;

  /// \}

  DART_BAKE_SPECIALIZED_ADDON(VisualData)

  DART_BAKE_SPECIALIZED_ADDON(CollisionData)

  DART_BAKE_SPECIALIZED_ADDON(DynamicsData)

  /// Get a pointer to the VisualData Addon for this ShapeNode. If
  /// _createIfNull is true, then the VisualData will be generated if one does
  /// not already exist.
  VisualData* getVisualData(const bool createIfNull);

  /// Get a pointer to the CollisionData Addon for this ShapeNode. If
  /// _createIfNull is true, then the CollisionData will be generated if one
  /// does not already exist.
  CollisionData* getCollisionData(const bool createIfNull);

  /// Get a pointer to the DynamicsData Addon for this ShapeNode. If
  /// _createIfNull is true, then the DynamicsData will be generated if one
  /// does not already exist.
  DynamicsData* getDynamicsData(const bool createIfNull);

  /// Render this Entity
  virtual void draw(renderer::RenderInterface* ri = nullptr,
                    const Eigen::Vector4d& color = Eigen::Vector4d::Ones(),
                    bool useDefaultColor = true) const;

protected:
  /// Constructor used by the Skeleton class
  ShapeNode(BodyNode* bodyNode, const Properties& properties);

  /// Create a clone of this ShapeNode. This may only be called by the Skeleton
  /// class.
  Node* cloneNode(BodyNode* parent) const override;

  /// Update the Jacobian of this ShapeNode. getJacobian() calls this function
  /// if mIsShapeNodeJacobianDirty is true.
  void updateShapeNodeJacobian() const;

  /// Update the World Jacobian cache.
  void updateWorldJacobian() const;

  /// Update the spatial time derivative of the ShapeNode Jacobian.
  /// getJacobianSpatialDeriv() calls this function if
  /// mIsShapeNodeJacobianSpatialDerivDirty is true.
  void updateShapeNodeJacobianSpatialDeriv() const;

  /// Update the classic time derivative of the ShapeNode Jacobian.
  /// getJacobianClassicDeriv() calls this function if
  /// mIsWorldJacobianClassicDerivDirty is true.
  void updateWorldJacobianClassicDeriv() const;

protected:

  /// ShapeNode properties
  UniqueProperties mShapeNodeProp;

  /// Cached Jacobian of this ShapeNode
  ///
  /// Do not use directly! Use getJacobian() to access this quantity
  mutable math::Jacobian mShapeNodeJacobian;

  /// Cached World Jacobian of this ShapeNode
  ///
  /// Do not use directly! Use getWorldJacobian() to access this quantity
  mutable math::Jacobian mWorldJacobian;

  /// Spatial time derivative of ShapeNode Jacobian
  ///
  /// Do not use directly! Use getJacobianSpatialDeriv() to access this quantity
  mutable math::Jacobian mShapeNodeJacobianSpatialDeriv;

  /// Classic time derivative of the ShapeNode Jacobian
  ///
  /// Do not use directly! Use getJacobianClassicDeriv() to access this quantity
  mutable math::Jacobian mWorldJacobianClassicDeriv;

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

#endif // DART_DYNAMICS_SHAPENODE_H_
