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

#include "dart/math/BSpline.h"

#include "dart/common/Console.h"

#include <cmath>
#include <iostream>

namespace dart {
namespace math {

//==============================================================================
BSpline::BSpline(size_t _degree,
                 const Eigen::VectorXd& _ctrlPts,
                 double _firstKnot,
                 double _lastKnot,
                 bool _knotMultiplicity)
  : mDegree(_degree),
    mCtrlPts(_ctrlPts),
    mKnots(Eigen::VectorXd()),
    mIsUniformKnots(false),
    mIsMultipleKnots(false)
{
  // 2 <= (degree + 1) <= (n + 1), where (degree + 1) is order and (n + 1) is
  // number of control points.
  assert(2 <= _degree + 1);
  assert(_degree <= static_cast<size_t>(_ctrlPts.size()));

  setUniformKnots(_firstKnot, _lastKnot, _knotMultiplicity);

  mLocalCtrlPts.resize(mDegree + 1);
  mLocalKnots.resize(2 * (mDegree + 1));
}

//==============================================================================
BSpline::BSpline(size_t _degree,
                 size_t _numCtrlPts,
                 double _beginTime,
                 double _endTime,
                 bool _knotMultiplicity)
  : BSpline(_degree,
            Eigen::VectorXd::Zero(_numCtrlPts),
            _beginTime,
            _endTime,
            _knotMultiplicity)
{
}

//==============================================================================
BSpline::~BSpline()
{
}

//==============================================================================
void BSpline::setControlPoint(size_t _index, double _point)
{
  assert(_index < static_cast<size_t>(mCtrlPts.size()));

  mCtrlPts[_index] = _point;
}

//==============================================================================
void BSpline::setControlPoints(const Eigen::VectorXd& _controlPoints)
{
  assert(_controlPoints.size() == mCtrlPts.size());

  mCtrlPts = _controlPoints;

  // TODO: Need to consider the case that the number of control points is
  // changed. In that case, knot vector needs to be updated as well.
}

//==============================================================================
const Eigen::VectorXd& BSpline::getControlPoints() const
{
  return mCtrlPts;
}

//==============================================================================
size_t BSpline::getNumControlPoints() const
{
  return mCtrlPts.size();
}

//==============================================================================
void BSpline::setKnot(size_t _index, double _val)
{
  // Check index
  assert(_index < static_cast<size_t>(mKnots.size()));

  // Check monotonicity of knot vector
  // Assume that the size of knot vector is at least 2.
  bool isMonotonic = true;
  if (_index != 0)
  {
    if (mKnots[_index - 1] > _val)
      isMonotonic = false;
  }
  else if (_index != static_cast<size_t>(mKnots.size()) - 1)
  {
    if (_val > mKnots[_index + 1])
      isMonotonic = false;
  }

  if (!isMonotonic)
  {
    dtwarn << "BSpline::setKnot(): "
           << "The input knot breaks monotonicity of the knot vector. "
           << "Ignoring the input knot." << std::endl;
    return;
  }

  // Update knot vector and its properties
  if (mKnots[_index] != _val)
  {
    mKnots[_index] = _val;

    mIsUniformKnots  = false;
    mIsMultipleKnots = false;
  }
}

//==============================================================================
void BSpline::setKnots(const Eigen::VectorXd& _knots)
{
  assert(_knots.size() == mKnots.size());

  if (!isMonoIncreasingVector(_knots))
  {
    dtwarn << "BSpline::setKnots(): "
           << "The knot vector is not monotone increasing. "
           << "Ignoring the knot vector." << std::endl;
  }

  // Update knot vector
  mKnots = _knots;

  // TODO: Do we need to update the knot vector properties (mIsUniformKnots,
  // mIsMultipleKnots)?
}

//==============================================================================
double BSpline::getKnot(size_t _index) const
{
  assert(_index < static_cast<size_t>(mKnots.size()));

  return mKnots[_index];
}

//==============================================================================
const Eigen::VectorXd& BSpline::getKnots() const
{
  return mKnots;
}

//==============================================================================
size_t BSpline::getNumKnots() const
{
  return mKnots.size();
}

//==============================================================================
double BSpline::getPoint(double _t)
{
  // Return 0.0 for the paramter out of the range of knot vector.
  if (_t < mKnots[0] || mKnots[mKnots.size() - 1] < _t)
    return 0.0;

  return computeDeBoor(_t);
}

//==============================================================================
double BSpline::operator()(double _t)
{
  return getPoint(_t);
}

//==============================================================================
int BSpline::computeNumKnots(size_t _degree, size_t _numCtrlPts)
{
  return _degree + 1 + _numCtrlPts;
}

//==============================================================================
void BSpline::setUniformKnots(double _firstKnot,
                              double _lastKnot,
                              bool _isMultipleKnots)
{
  // Number of knots: order + (n + 1), where (n + 1) is the number of control
  // points
  const int nKnots = computeNumKnots(mDegree, mCtrlPts.size());

  if (_isMultipleKnots)
  {
    // Number of middle knots (not repeating knots), which should be always
    // equal or greater than 2. If (2 <= order <= numCtrlPts) is satisfied, then
    // it is always equal or greater than 2.
    const int nMidKnots = mCtrlPts.size() - mDegree + 1;
    assert(nMidKnots >= 2);
    const int index = mDegree;

    mKnots.resize(nKnots);
    mKnots.segment(0, index).setConstant(_firstKnot);
    mKnots.segment(index, nMidKnots).setLinSpaced(
          nMidKnots, _firstKnot, _lastKnot);

    mKnots[mCtrlPts.size()] = _lastKnot;
    // TODO(JS): The last value of _knots is not always _endTime becaseu
    // Eigen::VectorXd::setLinSpaced() doesn't do that.

    mKnots.segment(mCtrlPts.size() + 1, index).setConstant(_lastKnot);
  }
  else
  {
    mKnots.setLinSpaced(nKnots, _firstKnot, _lastKnot);

    mKnots[mKnots.size() - 1] = _lastKnot;
    // TODO(JS): The last value of _knots is not always _endTime becaseu
    // Eigen::VectorXd::setLinSpaced() doesn't do that.
  }

  mIsUniformKnots  = true;
  mIsMultipleKnots = _isMultipleKnots;
}

//==============================================================================
size_t BSpline::getUniformKnotIndex(double _t)
{
  assert(mKnots[0] < mKnots[mKnots.size() - 1]);

  // Number of knots: (degree + 1) + (n + 1), where (n + 1) is the number of
  // control points
  const int nKnots = computeNumKnots(mDegree,  mCtrlPts.size());

  if (mIsMultipleKnots)
  {
    const int nMidKnots = mCtrlPts.size() - mDegree + 1;
    const double interval = (mKnots[mKnots.size() - 1] - mKnots[0])
                            / (nMidKnots - 1);

    return std::floor((_t - mKnots[0]) / interval) + mDegree;
  }
  else
  {
    const double interval = (mKnots[mKnots.size() - 1] - mKnots[0])
                            / (nKnots - 1);

    return std::floor((_t - mKnots[0]) / interval);
  }

}

//==============================================================================
size_t BSpline::getNonuniformKnotIndex(double _t)
{
  assert(mKnots[0] <= _t);
  assert(_t <= mKnots[mKnots.size() - 1]);

  size_t left = 0;
  size_t right = static_cast<size_t>(mKnots.size()) - 1;

  while (left < right - 1)
  {
    const size_t middle = left + 0.5 * (right - left);

    if (_t < mKnots[middle])
      right = middle; // take left section
    else
      left = middle;  // take right section
  }

  return left;
}

//==============================================================================
size_t BSpline::getKnotIndex(double _t)
{
  if (mIsUniformKnots)
    return getUniformKnotIndex(_t);
  else
    return getNonuniformKnotIndex(_t);
}

//==============================================================================
double BSpline::computeDeBoor(double _t)
{
  //----------------------------------------------------------------------------
  // Compute de Boor algorithm
  //----------------------------------------------------------------------------

  // Find knot index I such that u_{I} <= _t < u_{I+1}.
  const size_t knotIndex = getKnotIndex(_t);
  const size_t order     = mDegree + 1;

  // Segments of control points and knots that are influenced by the single
  // first-order basis function N_{I, 0}.
  segment(mLocalCtrlPts, mCtrlPts, knotIndex - mDegree,     order);
  segment(mLocalKnots  , mKnots  , knotIndex - mDegree, 2 * order);
  // TODO: Copying knot vector to local knot vector decrese the performance.

  // Compute the point on the curve using de Boor algorithm with the local
  // control points and local knots.
  for (size_t k = 0; k < mDegree; ++k)
  {
    const size_t j = mDegree - k - 1;

    for (size_t i = mDegree; i >= k + 1; --i)
    {
      // If the denominator of a fraction is zero, then it is zero.
      if ((mLocalKnots[i + j + 1] - mLocalKnots[i]) > DART_EPSILON)
      {
        const double alpha = (mLocalKnots[i + j + 1] - _t)
                             / (mLocalKnots[i + j + 1] - mLocalKnots[i]);

        mLocalCtrlPts[i] = alpha * mLocalCtrlPts[i - 1]
                           + (1 - alpha) * mLocalCtrlPts[i];
      }
    }
  }

  // The point on the curve is stored at the last element of the local control
  // points.
  return mLocalCtrlPts[mLocalCtrlPts.size() - 1];
}

//==============================================================================
void BSpline::segment(Eigen::VectorXd& _dest,
                      const Eigen::VectorXd& _src,
                      int _offset,
                      size_t _size)
{
  assert(static_cast<size_t>(_dest.size()) == _size);

  _dest.setZero();

  for (size_t i = 0; i < _size; ++i)
  {
    const int srcIndex = i + _offset;
    if (0 <= srcIndex && srcIndex < _src.size())
      _dest[i] = _src[srcIndex];
  }
}

//==============================================================================
bool BSpline::isMonoIncreasingVector(const Eigen::VectorXd& _val)
{
  for (int i = 1; i < _val.size(); ++i)
  {
    if (_val[i - 1] > _val[i])
      return false;
  }

  return true;
}

}  // namespace math
}  // namespace dart

