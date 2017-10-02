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

#include "dart/math/QuadraticExpression.hpp"

namespace dart {
namespace math {

//==============================================================================
QuadraticExpression::QuadraticExpression()
  : mMatrixCoefficient(Eigen::MatrixXd()), mAffinePart(AffineExpression())
{
  // Do nothing
}

//==============================================================================
QuadraticExpression::QuadraticExpression(
    const Eigen::MatrixXd& mat, const Eigen::VectorXd& vec, double constant)
  : mMatrixCoefficient(mat), mAffinePart(AffineExpression(vec, constant))
{
  assert(mat.rows() == mat.cols());
  assert(mat.rows() == vec.size());
}

//==============================================================================
QuadraticExpression::QuadraticExpression(
    const Eigen::MatrixXd& mat, const AffineExpression& affinePart)
  : mMatrixCoefficient(mat), mAffinePart(affinePart)
{
  assert(mat.rows() == mat.cols());
  assert(mat.rows() == mAffinePart.getCoefficients().size());
}

//==============================================================================
void QuadraticExpression::setMatrixCoefficient(const Eigen::MatrixXd& mat)
{
  mMatrixCoefficient = mat;
}

//==============================================================================
const Eigen::MatrixXd& QuadraticExpression::getMatrixCoefficient() const
{
  return mMatrixCoefficient;
}

//==============================================================================
void QuadraticExpression::setVectorCoefficients(const Eigen::VectorXd& vec)
{
  mAffinePart.setCoefficients(vec);
}

//==============================================================================
const Eigen::VectorXd& QuadraticExpression::getVectorCoefficients() const
{
  return mAffinePart.getCoefficients();
}

//==============================================================================
void QuadraticExpression::setConstant(double constant)
{
  mAffinePart.setConstant(constant);
}

//==============================================================================
double QuadraticExpression::getConstant() const
{
  return mAffinePart.getConstant();
}

//==============================================================================
std::size_t QuadraticExpression::getNumVariables() const
{
  return mAffinePart.getNumVariables();
}

//==============================================================================
double QuadraticExpression::eval(const Eigen::VectorXd& x) const
{
  return x.dot(mMatrixCoefficient*x) + mAffinePart.eval(x);
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator=(
    const QuadraticExpression& other)
{
  mMatrixCoefficient = other.mMatrixCoefficient;
  mAffinePart = other.mAffinePart;

  return *this;
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator=(
    const AffineExpression& other)
{
  mMatrixCoefficient = Eigen::MatrixXd::Zero(
      other.getNumVariables(), other.getNumVariables());
  mAffinePart = other;

  return *this;
}

//==============================================================================
QuadraticExpression QuadraticExpression::operator+(
    const QuadraticExpression& other) const
{
  assert(getNumVariables() == other.getNumVariables());

  return QuadraticExpression(
      mMatrixCoefficient + other.mMatrixCoefficient,
        mAffinePart + other.mAffinePart);
}

//==============================================================================
QuadraticExpression QuadraticExpression::operator+(
    const AffineExpression& other) const
{
  assert(getNumVariables() == other.getNumVariables());

  return QuadraticExpression(mMatrixCoefficient, mAffinePart + other);
}

//==============================================================================
QuadraticExpression QuadraticExpression::operator-(
    const QuadraticExpression& other) const
{
  assert(getNumVariables() == other.getNumVariables());

  return QuadraticExpression(
      mMatrixCoefficient - other.mMatrixCoefficient,
        mAffinePart - other.mAffinePart);
}

//==============================================================================
QuadraticExpression QuadraticExpression::operator-(
    const AffineExpression& other) const
{
  assert(getNumVariables() == other.getNumVariables());

  return QuadraticExpression(mMatrixCoefficient, mAffinePart - other);
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator+() const
{
  return *this;
}

//==============================================================================
QuadraticExpression QuadraticExpression::operator-() const
{
  return QuadraticExpression(mMatrixCoefficient, mAffinePart);
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator+=(
    const QuadraticExpression& other)
{
  assert(getNumVariables() == other.getNumVariables());

  mMatrixCoefficient += other.mMatrixCoefficient;
  mAffinePart += other.mAffinePart;

  return *this;
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator+=(
    const AffineExpression& other)
{
  assert(getNumVariables() == other.getNumVariables());

  mAffinePart += other;

  return *this;
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator-=(
    const QuadraticExpression& other)
{
  assert(getNumVariables() == other.getNumVariables());

  mMatrixCoefficient -= other.mMatrixCoefficient;
  mAffinePart -= other.mAffinePart;

  return *this;
}

//==============================================================================
const QuadraticExpression& QuadraticExpression::operator-=(
    const AffineExpression& other)
{
  assert(getNumVariables() == other.getNumVariables());

  mAffinePart -= other;

  return *this;
}

} // namespace math
} // namespace dart
