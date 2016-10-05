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

#ifndef DART_DYNAMICS_MULTISPHERESHAPE_HPP_
#define DART_DYNAMICS_MULTISPHERESHAPE_HPP_

#include <vector>

#include "dart/dynamics/Shape.hpp"

namespace dart {
namespace dynamics {

/// MultiSphereShape represents the convex hull of a collection of spheres.
class MultiSphereShape : public Shape
{
public:

  using Sphere = std::pair<double, Eigen::Vector3d>;
  using Spheres = std::vector<Sphere>;

  /// Constructor.
  explicit MultiSphereShape(const Spheres& spheres);

  /// Destructor.
  virtual ~MultiSphereShape();

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  /// Add a list of spheres
  void addSpheres(const Spheres& spheres);

  /// Add a sphere
  void addSphere(const Sphere& sphere);

  /// Add a sphere
  void addSphere(double radius, const Eigen::Vector3d& position);

  /// Remove all spheres
  void removeAllSpheres();

  /// Get the number of spheres
  std::size_t getNumSpheres() const;

  /// Get the set of spheres
  const Spheres& getSpheres() const;

  /// Compute the inertia of this MultiSphereShape.
  ///
  /// \note The return value is an approximated inertia that is the inertia of
  /// the axis-alinged bounding box of this MultiSphereShape.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:

  /// Update the volume of this MultiSphereShape.
  ///
  /// \note The result volume is an approximated volumen that is the volume of
  /// the axis-alinged bounding box of this MultiSphereShape.
  void updateVolume() override;

private:

  /// Update bounding box (in the local coordinate frame) of the shape.
  void updateBoundingBoxDim();

  /// Spheres
  Spheres mSpheres;

};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MULTISPHERESHAPE_HPP_
