/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
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

#ifndef DART_MATH_HELPERS_H_
#define DART_MATH_HELPERS_H_

// Standard Libraries
#include <cassert>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <vector>

// External Libraries
#include <Eigen/Dense>
// Local Headers
#include "dart/common/Console.h"
#include "dart/math/MathTypes.h"

namespace dart {
namespace math {

/// \brief a cross b = (CR*a) dot b
/// const Matd CR(2,2,0.0,-1.0,1.0,0.0);
const Eigen::Matrix2d CR((Eigen::Matrix2d() << 0.0, -1.0, 1.0, 0.0).finished());

inline int delta(int _i, int _j) {
  if (_i == _j)
    return 1;
  return 0;
}

inline int sgn(double _a) {
  if (_a < 0)
    return -1;
  else if (_a == 0)
    return 0;
  else
    return 1;
}

inline double sqr(double _x) {
  return _x*_x;
}

inline double Tsinc(double _theta) {
  return 0.5-sqrt(_theta)/48;
}

inline bool isZero(double _theta) {
  return (fabs(_theta) < DART_EPSILON);
}

inline double asinh(double _X) {
  return log(_X + sqrt(_X * _X + 1));
}

inline double acosh(double _X) {
  return log(_X + sqrt(_X * _X - 1));
}

inline double atanh(double _X) {
  return log((1 + _X)/(1 - _X))/ 2;
}

inline double asech(double _X) {
  return log((sqrt(-_X * _X + 1) + 1) / _X);
}

inline double acosech(double _X) {
  return log((sgn(_X) * sqrt(_X * _X + 1) +1) / _X);
}

inline double acotanh(double _X) {
  return log((_X + 1) / (_X - 1)) / 2;
}

inline double round(double _x) {
  return floor(_x + 0.5);
}

inline double round2(double _x) {
  int gintx = static_cast<int>(std::floor(_x));
  if (_x - gintx < 0.5)
    return static_cast<double>(gintx);
  else
    return static_cast<double>(gintx + 1.0);
}

inline bool isEqual(double _x, double _y) {
  return (std::fabs(_x - _y) < DART_EPSILON);
}

// check if it is an integer
inline bool isInt(double _x) {
  if (isEqual(round(_x), _x))
    return true;
  return false;
}

/// \brief Returns whether _v is a NaN (Not-A-Number) value
inline bool isNan(double _v) {
#ifdef WIN32
  return _isnan(_v);
#else
  return std::isnan(_v);
#endif
}

/// \brief Returns whether _m is a NaN (Not-A-Number) matrix
inline bool isNan(const Eigen::MatrixXd& _m) {
  for (int i = 0; i < _m.rows(); ++i)
    for (int j = 0; j < _m.cols(); ++j)
      if (isNan(_m(i, j)))
        return true;

  return false;
}

/// \brief Returns whether _v is an infinity value (either positive infinity or
/// negative infinity).
inline bool isInf(double _v) {
#ifdef WIN32
  return !_finite(_v);
#else
  return std::isinf(_v);
#endif
}

/// \brief Returns whether _m is an infinity matrix (either positive infinity or
/// negative infinity).
inline bool isInf(const Eigen::MatrixXd& _m) {
  for (int i = 0; i < _m.rows(); ++i)
    for (int j = 0; j < _m.cols(); ++j)
      if (isInf(_m(i, j)))
        return true;

  return false;
}

/// \brief Returns whether _m is symmetric or not
inline bool isSymmetric(const Eigen::MatrixXd& _m, double _tol = 1e-6) {
  size_t rows = _m.rows();
  size_t cols = _m.cols();

  if (rows != cols)
    return false;

  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = i + 1; j < cols; ++j) {
      if (std::fabs(_m(i, j) - _m(j, i)) > _tol) {
        std::cout << "A: " << std::endl;
        for (size_t k = 0; k < rows; ++k) {
          for (size_t l = 0; l < cols; ++l)
            std::cout << std::setprecision(4) << _m(k, l) << " ";
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << _m(i, j) << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << _m(i, j) << std::endl;
        return false;
      }
    }
  }

  return true;
}

inline unsigned seedRand() {
  time_t now = time(0);
  unsigned char* p = reinterpret_cast<unsigned char*>(&now);
  unsigned seed = 0;
  size_t i;

  for (i = 0; i < sizeof(now); i++)
    seed = seed * (UCHAR_MAX + 2U) + p[i];

  srand(seed);
  return seed;
}

inline double random(double _min, double _max) {
  return _min + ((static_cast<double>(rand()) / (RAND_MAX + 1.0))
                * (_max - _min));
}

inline int castUIntToInt(size_t _x)
{
  if (_x <= INT_MAX)
    return static_cast<int>(_x);

  if (_x >= INT_MIN)
    return static_cast<int>(_x - INT_MIN) + INT_MIN;

  dterr << "x is out of range." << std::endl;

  throw _x; // Or whatever else you like
}

}  // namespace math
}  // namespace dart

#endif  // DART_MATH_HELPERS_H_
