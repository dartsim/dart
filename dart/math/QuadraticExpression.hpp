/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_MATH_QUADRATICEXPRESSION_HPP_
#define DART_MATH_QUADRATICEXPRESSION_HPP_

#include "dart/math/AffineExpression.hpp"

namespace dart {
namespace math {

/// Mathematical expression of an affine form of `x^T A x + b^T x + c` where
/// this class stores `A`, `b`, and `c`.
class QuadraticExpression
{
public:
  QuadraticExpression();

  /// Constructs an quadratic expression given coefficients and constant.
  QuadraticExpression(
      const Eigen::MatrixXd& mat, const Eigen::VectorXd& vec, double constant);

  QuadraticExpression(
      const Eigen::MatrixXd& mat, const AffineExpression& affinePart);

  /// Sets the matrix coefficient component.
  void setMatrixCoefficient(const Eigen::MatrixXd& mat);

  /// Returns the matrix coefficient component.
  const Eigen::MatrixXd& getMatrixCoefficient() const;

  /// Sets the vector coefficient component.
  void setVectorCoefficients(const Eigen::VectorXd& vec);

  /// Returns the vector coefficient component.
  const Eigen::VectorXd& getVectorCoefficients() const;

  /// Sets the constant component.
  void setConstant(double constant);

  /// Returns the constant component.
  double getConstant() const;

  /// Returns the number of variables where it is identical to the size of the
  /// vector coefficient and the size of the squared-matrix coefficient.
  std::size_t getNumVariables() const;

  /// Returns evaluated value of this quadratic expression given x.
  double eval(const Eigen::VectorXd& x) const;

  /// Assignment operator
  const QuadraticExpression& operator=(const QuadraticExpression& other);

  /// Assignment operator
  const QuadraticExpression& operator=(const AffineExpression& other);

  /// Addition operator
  QuadraticExpression operator+(const QuadraticExpression& other) const;

  /// Addition operator
  QuadraticExpression operator+(const AffineExpression& other) const;

  /// Subtraction operator
  QuadraticExpression operator-(const QuadraticExpression& other) const;

  /// Subtraction operator
  QuadraticExpression operator-(const AffineExpression& other) const;

  /// Unary plus operator
  const QuadraticExpression& operator+() const;

  /// Unary minus operator
  QuadraticExpression operator-() const;

  /// Addition assignment operator
  const QuadraticExpression& operator+=(const QuadraticExpression& other);

  /// Addition assignment operator
  const QuadraticExpression& operator+=(const AffineExpression& other);

  /// Subtraction assignment operator
  const QuadraticExpression& operator-=(const QuadraticExpression& other);

  /// Subtraction assignment operator
  const QuadraticExpression& operator-=(const AffineExpression& other);

protected:
  /// Matrix coefficient component of this affine expression.
  Eigen::MatrixXd mMatrixCoefficient;

  /// Affine part of this affine expression (i.e., `b^T x + c`).
  AffineExpression mAffinePart;
};

} // namespace math
} // namespace dart

#endif // DART_MATH_QUADRATICEXPRESSION_HPP_

