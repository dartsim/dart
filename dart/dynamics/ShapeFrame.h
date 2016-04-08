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

#ifndef DART_DYNAMICS_SHAPEFRAME_H_
#define DART_DYNAMICS_SHAPEFRAME_H_

#include <Eigen/Dense>

#include "dart/common/Signal.h"
#include "dart/common/AspectWithVersion.h"
#include "dart/common/SpecializedForAspect.h"
#include "dart/dynamics/FixedFrame.h"
#include "dart/dynamics/TemplatedJacobianNode.h"
#include "dart/dynamics/EllipsoidShape.h"

namespace dart {
namespace dynamics {

class VisualAspect;
class CollisionAspect;
class DynamicsAspect;
class ShapeFrame;

namespace detail {

struct VisualAspectProperties
{
  /// Color for the primitive shape
  Eigen::Vector4d mRGBA;

  bool mUseDefaultColor;

  /// True if this shape node should be kept from rendering
  bool mHidden;

  /// Constructor
  VisualAspectProperties(
      const Eigen::Vector4d& color = Eigen::Vector4d(0.5, 0.5, 1.0, 1.0),
      const bool hidden = false);

  /// Destructor
  virtual ~VisualAspectProperties() = default;
};

struct CollisionAspectProperties
{
  /// This object is collidable if true
  bool mCollidable;

  /// Constructor
  CollisionAspectProperties(const bool collidable = true);

  /// Destructor
  virtual ~CollisionAspectProperties() = default;
};

struct DynamicsAspectProperties
{
  /// Coefficient of friction
  double mFrictionCoeff;

  /// Coefficient of restitution
  double mRestitutionCoeff;

  /// Constructor
  DynamicsAspectProperties(const double frictionCoeff = 1.0,
                           const double restitutionCoeff = 0.0);

  /// Destructor
  virtual ~DynamicsAspectProperties() = default;
};

struct ShapeFrameProperties
{
  /// Pointer to a shape
  ShapePtr mShape;

  /// Constructor
  ShapeFrameProperties(const ShapePtr& shape = nullptr);

  /// Virtual destructor
  virtual ~ShapeFrameProperties() = default;
};

} // namespace detail

class VisualAspect final :
    public common::AspectWithVersionedProperties<
        VisualAspect,
        detail::VisualAspectProperties,
        ShapeFrame>
{
public:

  using BaseClass = common::AspectWithVersionedProperties<
      VisualAspect, detail::VisualAspectProperties, ShapeFrame>;

  /// Constructor
  VisualAspect(common::Composite* comp,
              const PropertiesData& properties = PropertiesData());

  VisualAspect(const VisualAspect&) = delete;

  /// Set RGBA color
  void setRGBA(const Eigen::Vector4d& color);

  DART_COMMON_GET_ASPECT_PROPERTY( Eigen::Vector4d, RGBA )
  // void setRGBA(const Eigen::Vector4d& value);
  // const Eigen::Vector4d& getRGBA() const;

  DART_COMMON_SET_GET_ASPECT_PROPERTY( bool, Hidden )
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

class CollisionAspect final :
    public common::AspectWithVersionedProperties<
        CollisionAspect,
        detail::CollisionAspectProperties,
        ShapeFrame>
{
public:

  CollisionAspect(const CollisionAspect &) = delete;
  CollisionAspect(dart::common::Composite* comp,
                      const PropertiesData& properties = PropertiesData());

  DART_COMMON_SET_GET_ASPECT_PROPERTY( bool, Collidable )
  // void setCollidable(const bool& value);
  // const bool& getCollidable() const;

  /// Return true if this body can collide with others bodies
  bool isCollidable() const;

};

class DynamicsAspect final :
    public common::AspectWithVersionedProperties<
        DynamicsAspect,
        detail::DynamicsAspectProperties,
        ShapeFrame>
{
public:

  using BaseClass = common::AspectWithVersionedProperties<
      DynamicsAspect, detail::DynamicsAspectProperties, ShapeFrame>;

  DynamicsAspect(const DynamicsAspect&) = delete;

  DynamicsAspect(dart::common::Composite* comp,
                const PropertiesData& properties = PropertiesData());

  DART_COMMON_SET_GET_ASPECT_PROPERTY( double, FrictionCoeff )
  // void setFrictionCoeff(const double& value);
  // const double& getFrictionCoeff() const;
  DART_COMMON_SET_GET_ASPECT_PROPERTY( double, RestitutionCoeff )
  // void setRestitutionCoeff(const double& value);
  // const double& getRestitutionCoeff() const;

};

class ShapeFrame :
    public virtual common::VersionCounter,
    public common::EmbedPropertiesOnTopOf<
        ShapeFrame, detail::ShapeFrameProperties,
        common::SpecializedForAspect<
            VisualAspect, CollisionAspect, DynamicsAspect> >,
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

  using UniqueProperties = AspectProperties;
  using Properties = UniqueProperties;

  /// Destructor
  virtual ~ShapeFrame() = default;

  /// Set the UniqueProperties of this ShapeFrame
  void setProperties(const UniqueProperties& properties);

  /// Set the AspectProperties of this ShapeFrame
  void setAspectProperties(const AspectProperties& properties);

  const AspectProperties& getAspectProperties() const;

  /// Set shape
  void setShape(const ShapePtr& shape);

  /// Return shape
  ShapePtr getShape();

  /// Return (const) shape
  ConstShapePtr getShape() const;

  DART_BAKE_SPECIALIZED_ASPECT(VisualAspect)

  DART_BAKE_SPECIALIZED_ASPECT(CollisionAspect)

  DART_BAKE_SPECIALIZED_ASPECT(DynamicsAspect)

  /// Render this Entity
  virtual void draw(renderer::RenderInterface* ri = nullptr,
                    const Eigen::Vector4d& color = Eigen::Vector4d::Ones(),
                    bool useDefaultColor = true) const;

protected:

  /// Constructor
  ShapeFrame(Frame* parent, const Properties& properties);

  /// Constructor
  ShapeFrame(Frame* parent, const ShapePtr& shape = nullptr);

  /// Delegating constructor
  ShapeFrame(const std::tuple<Frame*, Properties>& args);

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

#endif // DART_DYNAMICS_SHAPEFRAME_H_
