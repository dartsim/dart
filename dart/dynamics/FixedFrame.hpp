/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_DYNAMICS_FIXEDFRAME_H_
#define DART_DYNAMICS_FIXEDFRAME_H_

#include "Frame.hpp"

namespace dart {
namespace dynamics {

/// The FixedFrame class represents a Frame with zero relative velocity and
/// zero relative acceleration. It does not move within its parent Frame after
/// its relative transform is set. However, classes that inherit the FixedFrame
/// class may alter its relative transform or change what its parent Frame is.
class FixedFrame : public virtual Frame
{
public:
  /// Constructor
  explicit FixedFrame(Frame* _refFrame, const std::string& _name,
                      const Eigen::Isometry3d& _relativeTransform =
                                        Eigen::Isometry3d::Identity());

  /// Destructor
  virtual ~FixedFrame();

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
  /// Relative Transform of this Frame
  Eigen::Isometry3d mRelativeTf;

  /// Used for Relative Velocity and Relative Acceleration of this Frame
  static const Eigen::Vector6d mZero;
};

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_FIXEDFRAME_H_
