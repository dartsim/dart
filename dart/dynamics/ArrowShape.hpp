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

#ifndef DART_DYNAMICS_ARROWSHAPE_HPP_
#define DART_DYNAMICS_ARROWSHAPE_HPP_

#include "dart/dynamics/MeshShape.hpp"

namespace dart {
namespace dynamics {

class ArrowShape : public MeshShape
{
public:

  struct Properties
  {
    /// _radius affects the thickness of the arrow. _headRadiusScale can be
    /// [1,INFINITY) and is a multiplier that affects the wideness of the
    /// beginning of the arrow head. _headLengthScale can be [0,1] and indicates
    /// what fraction of the arrow length is to be the conical head.
    /// _minHeadLength will prevent the arrow head from being shorter than the
    /// given value; _maxHeadLength will prevent it from being longer than the
    /// given value. Set _minHeadLength and _maxHeadLength to the same value to
    /// fix the size of the arrow head.
    Properties(double _radius=0.01, double _headRadiusScale=2.0,
               double _headLengthScale=0.15,
               double _minHeadLength=0,  double _maxHeadLength=INFINITY,
               bool _doubleArrow=false);

    double mRadius;
    double mHeadRadiusScale;
    double mHeadLengthScale;
    double mMinHeadLength;
    double mMaxHeadLength;
    bool mDoubleArrow;
  };

  /// This will produce an arrow that reaches from _tail to _head with the given
  /// properties.
  ArrowShape(const Eigen::Vector3d& _tail, const Eigen::Vector3d& _head,
             const Properties& _properties = Properties(),
             const Eigen::Vector4d& _color=Eigen::Vector4d(0.5,0.5,1.0,1.0),
             std::size_t _resolution=10);

  /// Set the positions of the tail and head of the arrow without changing any
  /// settings
  void setPositions(const Eigen::Vector3d& _tail, const Eigen::Vector3d& _head);

  /// Get the location of the tail of this arrow
  const Eigen::Vector3d& getTail() const;

  /// Get the location of the head of this arrow
  const Eigen::Vector3d& getHead() const;

  /// Set the properties of this arrow
  void setProperties(const Properties& _properties);

  /// Set the color of this arrow
  void notifyColorUpdated(const Eigen::Vector4d& _color) override;

  /// Get the properties of this arrow
  const Properties& getProperties() const;

  void configureArrow(const Eigen::Vector3d& _tail,
                      const Eigen::Vector3d& _head,
                      const Properties& _properties);

protected:

  void instantiate(std::size_t resolution);

  Eigen::Vector3d mTail;
  Eigen::Vector3d mHead;

  Properties mProperties;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_ARROWSHAPE_HPP_
