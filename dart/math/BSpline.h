/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_MATH_BSPLINE_H_
#define DART_MATH_BSPLINE_H_

#include <Eigen/Dense>

#include "dart/math/MathTypes.h"

namespace dart {
namespace math {

/// BSpline represents non-uniform (non-rational) b-spline curve with arbitrary
/// degree and control points.
// TODO: remove unnecessary static member functions
// - Implement getVelocity(), getAcceleration(), getDerivative(degree)
// - Implement spline fitting
// - Implement degree elevation/reduction
// - Implement nun-uniform rational b-spline
class BSpline
{
public:
  /// Constructor.
  /// \param[in] _degree Degree of spline, should be eqaul or greater than 2
  /// \param[in] _ctrlPts Control points of spline
  /// \param[in]
  BSpline(size_t _degree,
          const Eigen::VectorXd& _ctrlPts,
          double _firstKnot = 0.0,
          double _lastKnot = 1.0,
          bool _knotMultiplicity = true);

  /// Constructor.
  /// \param[in] _degree Degree of spline, should be eqaul or greater than 2
  /// \param[in] _ctrlPts Control points of spline
  /// \param[in]
  BSpline(size_t _degree,
          size_t _numCtrlPts,
          double _beginTime = 0.0,
          double _endTime = 1.0,
          bool _knotMultiplicity = true);

  /// Destructor.
  ~BSpline();

  /// Set control point.
  void setControlPoint(size_t _index, double _point);

  /// Set control points.
  void setControlPoints(const Eigen::VectorXd& _controlPoints);

  /// Get control point.
  double getControlPoint(size_t _index) const;

  /// Get control points.
  const Eigen::VectorXd& getControlPoints() const;

  /// Get number of control points.
  size_t getNumControlPoints() const;

  /// Set knot.
  void setKnot(size_t _index, double _value);

  /// Set knot vector.
  void setKnots(const Eigen::VectorXd& _knots);

  /// Set uniformly distributed knot vector.
  void setUniformKnots(double _firstKnot,
                       double _lastKnot,
                       bool _isMultipleKnots = true);

  /// Get knot.
  double getKnot(size_t _index) const;

  /// Get knot vector.
  const Eigen::VectorXd& getKnots() const;

  /// Get number of knots.
  size_t getNumKnots() const;

  /// Get knot index \f$ I \f$ such that \f$ u_{I} <= t < u_{I+1} \f$ where
  /// \f$u_i\f$ are knot elements and \f$t\f$ is parameter of curve.
  size_t getKnotIndex(double _t);

  /// Get point on the curve.
  double getPoint(double _t);

  /// Function call operator to get point on the curve.
  double operator()(double _t);

private:
  /// Get number of knots given degree and number of control points.
  int computeNumKnots(size_t _degree, size_t _numCtrlPts);

  /// Find index of knots of _t when the knots are unformly distributed.
  ///
  /// This function is usually faster than findKnotIndex.
  ///
  /// \warning If the number of control points is too large compare to the range
  /// of the parameter [_beginTime, _endTime], then it could cause numerical
  /// error to find knot index.
  size_t getUniformKnotIndex(double _t);

  /// Find the index of the range that contains _t using binary search
  size_t getNonuniformKnotIndex(double _t);

  /// Compute de Boor algorithm to get a point on curve.
  double computeDeBoor(double _t);

  /// Get segment from source vector to destination vector with given offset and
  /// size.
  ///
  /// If the indecies of elements of destination are out of source vector, the
  /// element will be set to zero.
  ///
  /// <pre>
  /// Case 1:
  ///          |---->|  offset: 2
  ///              [ s2 s3 s4 s5 s6 s7 s8 ]
  ///                |<--------------->|  size: 7
  ///        [ s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 ]  <-- source vector
  ///
  /// Case 2:
  ///    |<----|  offset: -2
  ///  [ 0  0  s0 s1 s2 s3 s4 ]
  ///    |<--------------->|  size: 7
  ///        [ s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 ]  <-- source vector
  ///
  /// Case 3:
  ///          |------------------------------>|  offset: 10
  ///                                        [ 0  0  0  0  0  0  0  ]
  ///                                          |<--------------->|  size: 7
  ///        [ s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 ]  <-- source vector
  /// </pre>
  void segment(Eigen::VectorXd& _dest,
               const Eigen::VectorXd& _src,
               int _offset,
               size_t _size);

  /// Return tru if the knot vector is monotone increasing.
  bool isMonoIncreasingVector(const Eigen::VectorXd& _val);

  /// Degree of this B-spline
  size_t mDegree;

  /// B-Spline Control points, n + 1
  Eigen::VectorXd mCtrlPts;

  /// Knot, No. of Knot = 2(k - 1 - 1) + (n + 1) = n + 2k -3, order = degree + 1
  Eigen::VectorXd mKnots;

  /// True if the knot vector is uniformly distributed
  bool mIsUniformKnots;

  /// True if the knot elements at the boundaries are repeated with number of
  /// degree + 1.
  bool mIsMultipleKnots;

  /// Local knots using in the computation of de Boor algorithm.
  Eigen::VectorXd mLocalKnots;

  /// Local control points using in the computation of de Boor algorithm.
  Eigen::VectorXd mLocalCtrlPts;
};

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_BSPLINE_H_

