/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_DYNAMICS_SHAPE_HPP_
#define DART_DYNAMICS_SHAPE_HPP_

#include <memory>

#include <Eigen/Dense>

#include "dart/common/Deprecated.hpp"
#include "dart/common/Signal.hpp"
#include "dart/common/Subject.hpp"
#include "dart/common/VersionCounter.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/dynamics/SmartPointer.hpp"

namespace dart {
namespace dynamics {

class Shape
    : public virtual common::Subject,
      public virtual common::VersionCounter
{
public:

  using VersionChangedSignal
      = common::Signal<void(Shape* shape, std::size_t version)>;

  /// \deprecated Deprecated in 6.1. Please use getType() instead.
  enum ShapeType {
    SPHERE,
    BOX,
    ELLIPSOID,
    CYLINDER,
    CAPSULE,
    CONE,
    PLANE,
    MULTISPHERE,
    MESH,
    SOFT_MESH,
    LINE_SEGMENT,
    HEIGHTMAP,
    UNSUPPORTED
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
  /// \deprecated Deprecated in 6.1. Please use getType() instead.
  explicit Shape(ShapeType _type);

  /// \brief Constructor
  Shape();

  /// \brief Destructor
  virtual ~Shape();

  /// Returns a string representing the shape type
  /// \sa is()
  virtual const std::string& getType() const = 0;

  /// Get true if the types of this Shape and the template parameter (a shape
  /// class) are identical. This function is a syntactic sugar, which is
  /// identical to: (getType() == ShapeType::getStaticType()).
  ///
  /// Example code:
  /// \code
  /// auto shape = bodyNode->getShapeNode(0)->getShape();
  /// if (shape->is<BoxShape>())
  ///   std::cout << "The shape type is box!\n";
  /// \endcode
  ///
  /// \sa getType()
  template <typename ShapeT>
  bool is() const;

  /// \brief Get the bounding box of the shape in its local coordinate frame.
  ///        The dimension will be automatically determined by the sub-classes
  ///        such as BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  const math::BoundingBox& getBoundingBox() const;

  /// Computes the inertia.
  virtual Eigen::Matrix3d computeInertia(double mass) const = 0;

  Eigen::Matrix3d computeInertiaFromDensity(double density) const;

  Eigen::Matrix3d computeInertiaFromMass(double mass) const;

  /// Returns volume of this shape.
  ///
  /// The volume will be automatically calculated by the sub-classes such as
  /// BoxShape, EllipsoidShape, CylinderShape, and MeshShape.
  double getVolume() const;

  /// \brief
  std::size_t getID() const;

  /// \deprecated Deprecated in 6.1. Please use getType() instead.
  DART_DEPRECATED(6.1)
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

  /// Notify that the alpha of this shape has updated
  DART_DEPRECATED(6.2)
  virtual void notifyAlphaUpdate(double alpha);

  /// Notify that the alpha of this shape has updated
  virtual void notifyAlphaUpdated(double alpha);

  /// Notify that the color (rgba) of this shape has updated
  DART_DEPRECATED(6.2)
  virtual void notifyColorUpdate(const Eigen::Vector4d& color);

  /// Notify that the color (rgba) of this shape has updated
  virtual void notifyColorUpdated(const Eigen::Vector4d& color);

  /// Increment the version of this Shape and notify its subscribers
  std::size_t incrementVersion() override final;

protected:
  /// Updates volume
  virtual void updateVolume() const = 0;

  /// Updates bounding box
  virtual void updateBoundingBox() const = 0;

  /// \brief The bounding box (in the local coordinate frame) of the shape.
  mutable math::BoundingBox mBoundingBox;

  /// Whether bounding box needs update
  mutable bool mIsBoundingBoxDirty;

  /// Volume enclosed by the geometry.
  mutable double mVolume;

  /// Whether volume needs update
  mutable bool mIsVolumeDirty;

  /// \brief Unique id.
  const std::size_t mID;

  /// The DataVariance of this Shape
  unsigned int mVariance;

  /// \brief
  static std::atomic_int mCounter;

  /// \deprecated Deprecated in 6.1. Please use getType() instead.
  /// Type of primitive shpae.
  ShapeType mType;

private:
  /// Triggered by incrementVersion()
  VersionChangedSignal mVersionChangedSignal;

public:
  /// Use this to subscribe to version change signals
  common::SlotRegister<VersionChangedSignal> onVersionChanged;

};

}  // namespace dynamics
}  // namespace dart

#include "dart/dynamics/detail/Shape.hpp"

#endif  // DART_DYNAMICS_SHAPE_HPP_
