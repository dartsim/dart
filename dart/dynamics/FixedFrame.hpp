/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_FIXEDFRAME_HPP_
#define DART_DYNAMICS_FIXEDFRAME_HPP_

#include "dart/dynamics/Frame.hpp"
#include "dart/common/EmbeddedAspect.hpp"
#include "dart/common/VersionCounter.hpp"
#include "dart/dynamics/detail/FixedFrameAspect.hpp"

namespace dart {
namespace dynamics {

/// The FixedFrame class represents a Frame with zero relative velocity and
/// zero relative acceleration. It does not move within its parent Frame after
/// its relative transform is set. However, classes that inherit the FixedFrame
/// class may alter its relative transform or change what its parent Frame is.
class FixedFrame :
    public virtual Frame,
    public virtual common::VersionCounter,
    public common::EmbedProperties<FixedFrame, detail::FixedFrameProperties>
{
public:
  /// Constructor
  explicit FixedFrame(Frame* refFrame,
      const Eigen::Isometry3d& relativeTransform =
          Eigen::Isometry3d::Identity());

  /// Destructor
  virtual ~FixedFrame();

  /// Set the AspectProperties of this FixedFrame
  void setAspectProperties(const AspectProperties& properties);

  /// Set the relative transform of this FixedFrame
  virtual void setRelativeTransform(const Eigen::Isometry3d& transform);

  // Documentation inherited
  const Eigen::Isometry3d& getRelativeTransform() const override;

  /// Always returns a zero vector
  const Eigen::Vector6d& getRelativeSpatialVelocity() const override;

  /// Always returns a zero vector
  const Eigen::Vector6d& getRelativeSpatialAcceleration() const override;

  /// Always returns a zero vector
  const Eigen::Vector6d& getPrimaryRelativeAcceleration() const override;

  /// Always returns a zero vector
  const Eigen::Vector6d& getPartialAcceleration() const override;

protected:

  /// Default constructor -- calls the Abstract constructor
  FixedFrame();

  /// Abstract constructor
  explicit FixedFrame(ConstructAbstractTag);

  /// Used for Relative Velocity and Relative Acceleration of this Frame
  static const Eigen::Vector6d mZero;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_FIXEDFRAME_HPP_
