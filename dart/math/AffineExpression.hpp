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

#ifndef DART_MATH_AFFINEEXPRESSION_HPP_
#define DART_MATH_AFFINEEXPRESSION_HPP_

#include <Eigen/Core>

namespace dart {
namespace math {

/// Mathematical expression of an affine form of `a^T x + b` where this class
/// stores `a` and `b`.
class AffineExpression
{
public:
  /// Constructs an affine expression with zero-sized coefficients and zero
  /// constant.
  AffineExpression();

  /// Constructs an affine expression given coefficients and constant.
  AffineExpression(const Eigen::VectorXd& coeffs, double constant);

  /// Sets the constants component.
  void setCoefficients(std::size_t index, double coeff);

  /// Sets the constants component.
  void setCoefficients(const Eigen::VectorXd& coeffs);

  /// Returns the constants component.
  double getCoefficient(std::size_t index) const;

  /// Returns the constants component.
  const Eigen::VectorXd& getCoefficients() const;

  /// Sets the constant component.
  void setConstant(double constant);

  /// Returns the constant component.
  double getConstant() const;

  /// Returns the number of variables where it's identical to the size of the
  /// vector coefficient.
  std::size_t getNumVariables() const;

  /// Returns evaluated value of this affine expression given x.
  double eval(const Eigen::VectorXd& x) const;

  /// Assignment operator
  const AffineExpression& operator=(const AffineExpression& other);

  /// Addition operator
  AffineExpression operator+(const AffineExpression& other) const;

  /// Subtraction operator
  AffineExpression operator-(const AffineExpression& other) const;

  /// Unary plus operator
  const AffineExpression& operator+() const;

  /// Unary minus operator
  AffineExpression operator-() const;

  /// Addition assignment operator
  const AffineExpression& operator+=(const AffineExpression& other);

  /// Subtraction assignment operator
  const AffineExpression& operator-=(const AffineExpression& other);

protected:
  /// Coefficients component of this affine expression.
  Eigen::VectorXd mCoefficients;

  /// Constant component of this affine expression.
  double mConstant;
};

} // namespace math
} // namespace dart

#endif // DART_MATH_AFFINEEXPRESSION_HPP_

