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
#include <unsupported/Eigen/Splines>

namespace dart {
namespace math {

using namespace Eigen;

/// BSpline is a wrapper class of Eigen::Spline with useful interface for DART.
template <typename _Scalar, int _Dim, int _Degree>
class BSpline : public Spline<_Scalar, _Dim, _Degree>
{
public:
  using Scalar = _Scalar; ///< The spline curve's scalar type.
  enum { Dimension = _Dim }; ///< The spline curve's dimension.
  enum { Degree = _Degree }; ///< The spline curve's degree.

  /// The point type the spline is representing.
  using PointType
      = typename Spline<_Scalar, _Dim, _Degree>::PointType;

  /// The data type used to store knot vectors.
  using KnotVectorType
      = typename Spline<_Scalar, _Dim, _Degree>::KnotVectorType;

  /// The data type used to store non-zero basis functions.
  using BasisVectorType
      = typename Spline<_Scalar, _Dim, _Degree>::BasisVectorType;

  /// The data type representing the spline's control points.
  using ControlPointVectorType
      = typename Spline<_Scalar, _Dim, _Degree>::ControlPointVectorType;

  /// Creates a (constant) zero spline.
  ///
  /// For Splines with dynamic degree, the resulting degree will be 0.
  BSpline() : Spline<_Scalar, _Dim, _Degree>() {}

  /// Creates a spline from degree, control points, knot range, and knot vector
  /// type.
  BSpline(DenseIndex _degree,
          const ControlPointVectorType& _ctrls,
          Scalar _firstKnot = 0.0,
          Scalar _lastKnot = 1.0,
          bool _isOpenKnots = true)
  {
    auto numCtrls = _ctrls.cols();
    auto& ctrls_ = const_cast<ControlPointVectorType&>(
                     Spline<_Scalar, _Dim, _Degree>::ctrls());
    ctrls_ = _ctrls;

    auto numKnots = _degree + numCtrls + 1;
    auto& knots_ = const_cast<KnotVectorType&>(
                     Spline<_Scalar, _Dim, _Degree>::knots());
    knots_.resize(numKnots);
    setUniformKnots(knots_, _firstKnot, _lastKnot, _isOpenKnots);
  }

  /// Creates a spline from degree, number of control points, knot range, and
  /// knot vector type.
  BSpline(DenseIndex _degree,
          DenseIndex _numCtrlPts,
          Scalar _firstKnot = 0.0,
          Scalar _lastKnot = 1.0,
          bool _isOpenKnots = true)
    : BSpline(_degree,
              ControlPointVectorType(_Dim, _numCtrlPts),
              _firstKnot,
              _lastKnot,
              _isOpenKnots) {}

  /// Creates a spline from a knot vector and control points.
  /// \param[in] _knots The spline's knot vector.
  /// \param[in] _ctrls The spline's control point vector.
  template <typename OtherVectorType, typename OtherArrayType>
  BSpline(const OtherVectorType& _knots, const OtherArrayType& _ctrls)
    : Spline<_Scalar, _Dim, _Degree>(_knots, _ctrls) {}

  /// Copy constructor for splines.
  /// \param[in] _spline The input spline.
  template <int OtherDegree>
  BSpline(const BSpline<Scalar, Dimension, OtherDegree>& _spline)
    : Spline<_Scalar, _Dim, _Degree>(_spline) {}

  /// Set an element of control points at (i, j) to val.
  void setControlPoint(DenseIndex _i, DenseIndex _j, Scalar _val)
  {
    if (_i < 0 || _i >= Spline<_Scalar, _Dim, _Degree>::ctrls().rows())
      return;

    if (_j < 0 || _j >= Spline<_Scalar, _Dim, _Degree>::ctrls().cols())
      return;

    auto& ctrls_ = getControlPoints();

    ctrls_(_i,_j) = _val;
  }

  /// Set control point.
  void setControlPoints(const ControlPointVectorType& _ctrls)
  {
    auto& ctrls_ = getControlPoints();

    if (ctrls_.rows() != _ctrls.rows())
      return;

    if (ctrls_.cols() != _ctrls.cols())
      return;

    ctrls_ = _ctrls;
  }

  /// Get an element of control points at (i, j) to val.
  Scalar getControlPoint(DenseIndex _i, DenseIndex _j) const
  {
    return Spline<_Scalar, _Dim, _Degree>::ctrls()(_i, _j);
  }

  /// Get (const) control points.
  const ControlPointVectorType& getControlPoints() const
  {
    return Spline<_Scalar, _Dim, _Degree>::ctrls();
  }

  /// Get control points.
  ControlPointVectorType& getControlPoints()
  {
    return const_cast<ControlPointVectorType&>(
          Spline<_Scalar, _Dim, _Degree>::ctrls());
  }

  /// Get number of elements of control points.
  DenseIndex getNumControlPoints() const
  {
    return Spline<_Scalar, _Dim, _Degree>::ctrls().cols();
  }

  /// Set an element of knot vector at i to val.
  void setKnot(DenseIndex _i, Scalar _val)
  {
    if (_i < 0 || _i >= Spline<_Scalar, _Dim, _Degree>::knots().size())
      return;

    auto& knots_ = getKnots();

    knots_[_i] = _val;
  }

  /// Set knot vector.
  void setKnots(const KnotVectorType& _knots)
  {
    if (_knots.size() != Spline<_Scalar, _Dim, _Degree>::knots().size())
      return;

    auto& knots_ = getKnots();

    knots_ = _knots;
  }

  /// Set knot vector with evenly spaced values.
  void setUniformKnots(KnotVectorType& _knots,
                       Scalar _firstKnot,
                       Scalar _lastKnot,
                       bool _isOpenKnots)
  {
    const auto degree_ = Spline<_Scalar, _Dim, _Degree>::degree();
    const auto& ctrls_ = Spline<_Scalar, _Dim, _Degree>::ctrls();
    const auto numCtrls = ctrls_.cols();
    const auto numKnots = _knots.size();

    if (_isOpenKnots)
    {
      // Number of middle knots (not repeating knots), which should be always
      // equal or greater than 2. If (2 <= order <= numCtrlPts) is satisfied,
      // then it is always equal or greater than 2.
      const auto numMidKnots = numCtrls - degree_ + 1;
      eigen_assert(numMidKnots >= 2);

      _knots.segment(0, degree_).setConstant(_firstKnot);
      _knots.segment(degree_, numMidKnots).setLinSpaced(
            numMidKnots, _firstKnot, _lastKnot);

      _knots[numCtrls] = _lastKnot;
      // TODO: The last value of _knots is not always _endTime because
      // Eigen::VectorXd::setLinSpaced() doesn't do that.

      _knots.segment(numCtrls + 1, degree_).setConstant(_lastKnot);
    }
    else
    {
      _knots.setLinSpaced(numKnots, _firstKnot, _lastKnot);

      _knots[numKnots - 1] = _lastKnot;
      // TODO: The last value of _knots is not always _endTime because
      // Eigen::VectorXd::setLinSpaced() doesn't do that.
    }
  }

  /// Get an element of knot vector given index.
  Scalar getKnot(DenseIndex _i) const
  {
    return Spline<_Scalar, _Dim, _Degree>::knots()[_i];
  }

  /// Get (const) knot vector.
  const KnotVectorType& getKnots() const
  {
    return Spline<_Scalar, _Dim, _Degree>::knots();
  }

  /// Get knot vector.
  KnotVectorType& getKnots()
  {
    return const_cast<KnotVectorType&>(Spline<_Scalar, _Dim, _Degree>::knots());
  }

  /// Get number of elements of knot vector.
  DenseIndex getNumKnots() const
  {
    return Spline<_Scalar, _Dim, _Degree>::knots().size();
  }

  /// Returns the spline point at a given site \f$u\f$.
  ///
  /// The function returns
  /// \f{align*}
  ///   C(u) & = \sum_{i=0}^{n}N_{i,p}(u) P_i
  /// \f}
  /// for i raning between 0 and order.
  ///
  /// \param[in] _u Parameter \f$u \in [0;1]\f$ at which the spline is
  /// evaluated.
  /// \return The spline point at the given location \f$u\f$.
  PointType getPosition(Scalar _u) const
  {
    return *this(_u);
  }

  /// Returns the spline velocity at a given site \f$u\f$.
  ///
  /// The function returns
  /// \f{align*}
  ///   C'(u) & = \sum_{i=0}^{n}N'_{i,p}(u) P_i
  /// \f}
  /// for i raning between 0 and order.
  ///
  /// \param[in] _u Parameter \f$u \in [0;1]\f$ at which the spline is
  /// evaluated.
  /// \return The spline velocity at the given location \f$u\f$.
  PointType getVelocity(Scalar _u) const
  {
    return Spline<_Scalar, _Dim, _Degree>::derivatives(_u, 1);
  }

  /// Returns the spline acceleration at a given site \f$u\f$.
  ///
  /// The function returns
  /// \f{align*}
  ///   C''(u) & = \sum_{i=0}^{n}N''_{i,p}(u) P_i
  /// \f}
  /// for i raning between 0 and order.
  ///
  /// \param[in] u Parameter \f$u \in [0;1]\f$ at which the spline is evaluated.
  /// \return The spline acceleration at the given location \f$u\f$.
  PointType getAcceleration(Scalar _u) const
  {
    return Spline<_Scalar, _Dim, _Degree>::derivatives(_u, 2);
  }
};

/// 1D float B-spline with dynamic degree.
using BSpline1f = BSpline<float,1,Dynamic>;

/// 2D float B-spline with dynamic degree.
using BSpline2f = BSpline<float,2,Dynamic>;

/// 3D float B-spline with dynamic degree.
using BSpline3f = BSpline<float,3,Dynamic>;

/// 1D double B-spline with dynamic degree.
using BSpline1d = BSpline<double,1,Dynamic>;

/// 2D double B-spline with dynamic degree.
using BSpline2d = BSpline<double,2,Dynamic>;

/// 3D double B-spline with dynamic degree.
using BSpline3d = BSpline<double,3,Dynamic>;

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_BSPLINE_H_

