/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_SHAPE_H_
#define DART_DYNAMICS_SHAPE_H_

#include <memory>

#include <Eigen/Dense>

#include "dart/math/Geometry.h"
#include "dart/common/Deprecated.h"
#include "dart/common/Subject.h"
#include "dart/dynamics/SmartPointer.h"
#include "dart/common/Deprecated.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {
/// \brief
class Shape : public virtual common::Subject
{
public:
  // TODO(JS): We should not use ShapeType because this is not extendable.
  /// \brief
  enum ShapeType {
    BOX,
    ELLIPSOID,
    CYLINDER,
    PLANE,
    MESH,
    SOFT_MESH,
    LINE_SEGMENT
  };

  /// DataVariance can be used by renderers to determine whether it should
  /// expect data for this shape to change during each update.
  enum DataVariance {
    STATIC=0,                   /// No data will ever change
    DYNAMIC_TRANSFORM = 1 << 1, /// The relative transform of the Shape might change
    DYNAMIC_PRIMITIVE = 1 << 2, /// The primitive properties (such as x/y/z scaling) of the shape might change
    DYNAMIC_COLOR     = 1 << 3, /// The coloring or textures of the shape might change
    DYNAMIC_VERTICES  = 1 << 4, /// Vertex positions of a mesh might change (this does not include adding or removing vertices) (this enum is not relevant for primitive shapes)
    DYNAMIC_ELEMENTS  = 1 << 5, /// The number of elements and/or arrangement of elements might change (this includes adding and removing vertices)  (this enum is not relevant for primitive shapes)
    DYNAMIC           = 0xFF    /// All data is subject to changing
  };

  /// \brief Constructor
  explicit Shape(ShapeType _type);

  /// \brief Destructor
  virtual ~Shape();

  /// \brief Set RGB color components (leave alpha alone). Identical to
  /// setRGB(const Eigen::Vector3d&)
  void setColor(const Eigen::Vector3d& _color);

  /// \brief Set RGBA color components
  void setColor(const Eigen::Vector4d& _color);

  /// \brief Set RGB color components (leave alpha alone)
  void setRGB(const Eigen::Vector3d& _rgb);

  /// \brief Set RGBA color components
  virtual void setRGBA(const Eigen::Vector4d& _rgba);

  /// \brief Get color.
  Eigen::Vector3d getColor() const;

  /// \brief Get RGB color components
  Eigen::Vector3d getRGB() const;

  /// \brief Get RGBA color components
  const Eigen::Vector4d& getRGBA() const;

  /// \brief Set the transparency of this Shape
  virtual void setAlpha(double _alpha);


  /// \brief Get the bounding box of the shape in its local coordinate frame.
  ///        The dimension will be automatically determined by the sub-classes
  ///        such as BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  const math::BoundingBox& getBoundingBox() const;

  /// \brief Get dimensions of bounding box.
  ///        The dimension will be automatically determined by the sub-classes
  ///        such as BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  /// \deprecated Please use getBoundingBox() instead
  DEPRECATED(5.2)
  Eigen::Vector3d getBoundingBoxDim() const;

  /// \brief Set local transformation of the shape w.r.t. parent frame.
  void setLocalTransform(const Eigen::Isometry3d& _Transform);

  /// \brief Get local transformation of the shape w.r.t. parent frame.
  const Eigen::Isometry3d& getLocalTransform() const;

  /// \brief Set local transformation of the shape
  ///        The offset is translational part of the local transformation.
  /// \sa setLocalTransform()
  void setOffset(const Eigen::Vector3d& _offset);

  /// \brief Get local offset of the shape w.r.t. parent frame.
  ///        The offset is translational part of the local transformation.
  /// \sa getLocalTransform()
  Eigen::Vector3d getOffset() const;

  /// \brief
  virtual Eigen::Matrix3d computeInertia(double mass) const = 0;

  Eigen::Matrix3d computeInertiaFromDensity(double density) const;

  Eigen::Matrix3d computeInertiaFromMass(double mass) const;

  /// \brief Get volume of this shape.
  ///        The volume will be automatically calculated by the sub-classes
  ///        such as BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  double getVolume() const;

  /// \brief
  int getID() const;

  /// \brief
  ShapeType getShapeType() const;

  /// Set the data variance of this shape. Use the DataVariance to indicate what
  /// kind of shape information might change during run time so that renderers
  /// can optimize reliably.
  void setDataVariance(unsigned int _variance);

  /// Add a type of variance to this shape. All other variance types will remain
  /// the same.
  void addDataVariance(unsigned int _variance);

  /// Remove a type of variance from this shape. All other variance types will
  /// remain the same.
  void removeDataVariance(unsigned int _variance);

  /// Get the data variance of this shape
  unsigned int getDataVariance() const;

  /// True iff this Shape has the specified type of DataVariance
  bool checkDataVariance(DataVariance type) const;

  /// Instruct this shape to update its data
  virtual void refreshData();

  /// \brief
  virtual void draw(renderer::RenderInterface* _ri = nullptr,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true) const = 0;

  /// Pass in true to prevent this shape from being rendered. Pass in false to
  /// allow it to render again.
  void setHidden(bool _hide=true);

  /// True iff this Shape is set to be hidden. Use hide(bool) to change this
  /// setting
  bool isHidden() const;

protected:
  DEPRECATED(6.0)
  virtual void computeVolume() { updateVolume(); }

  /// \brief Update volume
  virtual void updateVolume() = 0;

  DEPRECATED(6.0)
  virtual void initMeshes() {}

  /// \brief The bounding box (in the local coordinate frame) of the shape.
  math::BoundingBox mBoundingBox;

  /// \brief Volume enclosed by the geometry.
  double mVolume;

  /// \brief Unique id.
  int mID;

  /// \brief Color for the primitive.
  Eigen::Vector4d mColor;

  /// \brief Local geometric transformation of the Shape w.r.t. parent frame.
  Eigen::Isometry3d mTransform;

  /// The DataVariance of this Shape
  unsigned int mVariance;

  /// True if this shape should be kept from rendering
  bool mHidden;

  /// \brief
  static int mCounter;

private:
  /// \brief Type of primitive
  ShapeType mType;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SHAPE_H_
