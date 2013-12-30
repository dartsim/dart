/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
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

#include <Eigen/Dense>

#include "dart/math/Geometry.h"

namespace dart {
namespace renderer {
class RenderInterface;
}  // namespace renderer
}  // namespace dart

namespace dart {
namespace dynamics {

/// \brief
class Shape {
public:
  // TODO(JS): We should not use ShapeType because this is not extendable.
  /// \brief
  enum ShapeType {
    BOX,
    ELLIPSOID,
    CYLINDER,
    PLANE,
    MESH,
    SOFT_MESH
  };

  /// \brief Constructor
  explicit Shape(ShapeType _type);

  /// \brief Destructor
  virtual ~Shape();

  /// \brief Set color.
  void setColor(const Eigen::Vector3d& _color);

  /// \brief Get color.
  const Eigen::Vector3d& getColor() const;

  /// \brief Get dimensions of bounding box.
  ///        The dimension will be automatically determined by the sub-classes
  ///        such as BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  // TODO(JS): Single Vector3d does not fit to represent bounding box for
  //           biased mesh shape. Two Vector3ds might be better; one is for
  //           minimum verterx, and the other is for maximum verterx of the
  //           bounding box.
  const Eigen::Vector3d& getBoundingBoxDim() const;

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
  virtual Eigen::Matrix3d computeInertia(double _mass) const = 0;

  /// \brief Get volume of this shape.
  ///        The volume will be automatically calculated by the sub-classes
  ///        such as BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  double getVolume() const;

  /// \brief
  int getID() const;

  /// \brief
  ShapeType getShapeType() const;

  /// \brief
  virtual void draw(renderer::RenderInterface* _ri = NULL,
                    const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                    bool _useDefaultColor = true) const = 0;

protected:
  /// \brief
  virtual void computeVolume() = 0;

  /// \brief
  virtual void initMeshes() {}

  /// \brief Dimensions for bounding box.
  Eigen::Vector3d mBoundingBoxDim;

  /// \brief Volume enclosed by the geometry.
  double mVolume;

  /// \brief Unique id.
  int mID;

  /// \brief Color for the primitive.
  Eigen::Vector3d mColor;

  /// \brief Local geometric transformation of the Shape w.r.t. parent frame.
  Eigen::Isometry3d mTransform;

  /// \brief
  static int mCounter;

private:
  /// \brief Type of primitive
  ShapeType mType;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_SHAPE_H_
