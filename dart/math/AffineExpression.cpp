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

#include "dart/math/AffineExpression.hpp"

namespace dart {
namespace math {

//==============================================================================
AffineExpression::AffineExpression()
  : mCoefficients(Eigen::VectorXd()), mConstant(0)
{
  // Do nothing
}

//==============================================================================
AffineExpression::AffineExpression(
    const Eigen::VectorXd& coeffs, double constant)
  : mCoefficients(coeffs), mConstant(constant)
{
  // Do nothing
}

//==============================================================================
void AffineExpression::setCoefficients(std::size_t index, double coeff)
{
  assert(index < static_cast<std::size_t>(mCoefficients.size()));
  mCoefficients[index] = coeff;
}

//==============================================================================
void AffineExpression::setCoefficients(const Eigen::VectorXd& coeffs)
{
  mCoefficients = coeffs;
}

//==============================================================================
double AffineExpression::getCoefficient(std::size_t index) const
{
  assert(index < static_cast<std::size_t>(mCoefficients.size()));
  return mCoefficients[index];
}

//==============================================================================
const Eigen::VectorXd& AffineExpression::getCoefficients() const
{
  return mCoefficients;
}

//==============================================================================
void AffineExpression::setConstant(double constant)
{
  mConstant = constant;
}

//==============================================================================
double AffineExpression::getConstant() const
{
  return mConstant;
}

//==============================================================================
std::size_t AffineExpression::getNumVariables() const
{
  return static_cast<std::size_t>(mCoefficients.size());
}

//==============================================================================
double AffineExpression::eval(const Eigen::VectorXd& x) const
{
  assert(mCoefficients.size() == x.size());
  return mCoefficients.dot(x) + mConstant;
}

//==============================================================================
const AffineExpression& AffineExpression::operator=(
    const AffineExpression& other)
{
  mCoefficients = other.mCoefficients;
  mConstant = other.mConstant;

  return *this;
}

//==============================================================================
AffineExpression AffineExpression::operator+(
    const AffineExpression& other) const
{
  assert(getNumVariables() == other.getNumVariables());

  return AffineExpression(
      mCoefficients + other.mCoefficients, mConstant + other.mConstant);
}

//==============================================================================
AffineExpression AffineExpression::operator-(
    const AffineExpression& other) const
{
  assert(getNumVariables() == other.getNumVariables());

  return AffineExpression(
        mCoefficients - other.mCoefficients, mConstant - other.mConstant);
}

//==============================================================================
const AffineExpression& AffineExpression::operator+() const
{
  return *this;
}

//==============================================================================
AffineExpression AffineExpression::operator-() const
{
  return AffineExpression(-mCoefficients, -mConstant);
}
//==============================================================================
const AffineExpression& AffineExpression::operator+=(
    const AffineExpression& other)
{
  assert(getNumVariables() == other.getNumVariables());

  mCoefficients += other.mCoefficients;
  mConstant += other.mConstant;

  return *this;
}

//==============================================================================
const AffineExpression& AffineExpression::operator-=(
    const AffineExpression& other)
{
  assert(getNumVariables() == other.getNumVariables());

  mCoefficients -= other.mCoefficients;
  mConstant -= other.mConstant;

  return *this;
}

} // namespace math
} // namespace dart
