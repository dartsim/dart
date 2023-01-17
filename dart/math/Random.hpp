/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_MATH_RANDOM_HPP_
#define DART_MATH_RANDOM_HPP_

#include <dart/math/Fwd.hpp>

#include <random>

namespace dart {
namespace math {

class DART_MATH_API Random final
{
public:
  using GeneratorType = std::mt19937;

  template <typename FloatType>
  using UniformRealDist = std::uniform_real_distribution<FloatType>;

  template <typename IntType>
  using UniformIntDist = std::uniform_int_distribution<IntType>;

  template <typename FloatType>
  using NormalRealDist = std::normal_distribution<FloatType>;

  /// Returns a mutable reference to the random generator
  static GeneratorType& getGenerator();

  /// Sets the seed value.
  ///
  /// The same seed gives the same sequence of random values so that you can
  /// regenerate the same sequencial random values as long as you knot the seed
  /// value.
  static void setSeed(unsigned int seed);

  /// Generates a seed value using the default random device.
  ///
  /// \param[in] applyGeneratedSeed Whether to apply the generated seed.
  /// \return The new seed value.
  static unsigned int generateSeed(bool applyGeneratedSeed = false);

  /// \return The current seed value.
  static unsigned int getSeed();

  /// Returns a random number from an uniform distribution.
  ///
  ///
  /// This template function can generate different scalar types of random
  /// numbers as:
  /// - Floating-point number: \c float, \c double, \c long double
  /// - Integer number: [\c unsigned] \c short, [\c unsigned] \c int,
  ///   [\c unsigned] \c long, [\c unsigned] \c long \c long
  ///
  /// and vectors and matrices as:
  /// - Fixed-size: Vector3i, Vector3d, Matrix4d, and so
  ///   on.
  /// - Dynamic-size: VectorXi, VectorXd, MatrixXd, and so
  ///   on.
  ///
  /// Example:
  /// \code
  /// // Generate a random int in [0, 10]
  /// int intVal1 = Random::uniform(0, 10);
  /// int intVal2 = Random::uniform<int>(0, 10);
  ///
  /// // Generate a random double in [0.0, 10.0)
  /// double dblVal1 = Random::uniform(0.0, 10.0);
  /// double dblVal2 = Random::uniform<double>(0, 10);
  ///
  /// // Generate a random vector in [lb, ub)
  /// Vector3d lb = Vector3d::Constant(1);
  /// Vector3d ub = Vector3d::Constant(4);
  /// Vector3d vecVal1 = Random::uniform(lb, ub);
  /// Vector3d vecVal2 = Random::uniform<Vector3d>(lb, ub);
  ///
  /// // Generate a random matrix in [lb, ub)
  /// Matrix4f lb = Matrix4f::Constant(1);
  /// Matrix4f ub = Matrix4f::Constant(4);
  /// Matrix4f vecVal1 = Random::uniform(lb, ub);
  /// Matrix4f vecVal2 = Random::uniform<Matrix4f>(lb, ub);
  /// \endcode
  ///
  /// Note that the end of the range is closed for integer types (i.e.,
  /// [int_min, int_max]), but open for floating-point types (i.e., [float_min,
  /// float_max)).
  ///
  /// \tparam S The type of random value.
  /// \param[in] min Lower bound of the distribution.
  /// \param[in] max Upper bound of the distribution.
  ///
  /// \sa normal()
  template <typename S>
  static S uniform(S min, S max);

  /// Returns a random vector or matrix from an uniform distribution.
  ///
  /// This is a helper function for the case that the each of lower and upper
  /// bound has an uniform element value in it. For example, the lower bound is
  /// [1, 1, 1] or [-2, -2].
  ///
  /// This variant is meant to be used for fixed-size vector or matrix types.
  /// For dynamic-size types, please use other variants that takes the size of
  /// vector or matrix.
  ///
  /// Example:
  /// \code
  /// // Generate random vectors
  /// VectorXi vecXi = Random::uniform<VectorXi>(0, 10);
  /// VectorXd vecXd = Random::uniform<VectorXd>(0.0, 10.0);
  /// \endcode
  ///
  /// \tparam FixedSizeT The type of fixed-size vector or fixed-size matrix.
  /// \param[in] min The constant value of the lower bound.
  /// \param[in] max The constant value of the upper bound.
  ///
  /// \sa uniform()
  template <typename FixedSizeT>
  static FixedSizeT uniform(
      typename FixedSizeT::Scalar min, typename FixedSizeT::Scalar max);

  /// Returns a random vector from an uniform distribution.
  ///
  /// This variant is meant to be used for dynamic-size vector.
  ///
  /// Example:
  /// \code
  /// // Generate random matrices
  /// MatrixXi matXi = Random::uniform<MatrixXi>(0, 10);
  /// MatrixXd matXd = Random::uniform<MatrixXd>(0.0, 10.0);
  /// \endcode
  ///
  /// \tparam DynamicSizeVectorT The type of dynamic-size vector.
  /// \param[in] size The size of the vectors.
  /// \param[in] min The constant value of the lower bound vector.
  /// \param[in] max The constant value of the upper bound vector.
  template <typename DynamicSizeVectorT>
  static DynamicSizeVectorT uniform(
      int size,
      typename DynamicSizeVectorT::Scalar min,
      typename DynamicSizeVectorT::Scalar max);

  /// Returns a random matrix from an uniform distribution.
  ///
  /// This variant is meant to be used for dynamic-size matrix.
  ///
  /// \tparam DynamicSizeMatrixT The type of dynamic-size matrix.
  /// \param[in] rows The row size of the matrices.
  /// \param[in] cols The col size of the matrices.
  /// \param[in] min The constant value of the lower bound matrix.
  /// \param[in] max The constant value of the upper bound matrix.
  ///
  /// \sa uniform()
  template <typename DynamicSizeMatrixT>
  static DynamicSizeMatrixT uniform(
      int rows,
      int cols,
      typename DynamicSizeMatrixT::Scalar min,
      typename DynamicSizeMatrixT::Scalar max);

  /// Returns a random number from a normal distribution.
  ///
  /// This template function can generate different scalar types of random
  /// numbers as:
  /// - Floating-point number: \c float, \c double, \c long double
  /// - Integer number: [\c unsigned] \c short, [\c unsigned] \c int,
  ///   [\c unsigned] \c long, [\c unsigned] \c long \c long
  ///
  /// Example:
  /// \code
  /// // Generate a random int
  /// int intVal = Random::normal(0, 10);
  ///
  /// // Generate a random double
  /// double dblVal = Random::normal(0.0, 10.0);
  /// \endcode
  ///
  /// \param[in] mean Mean of the normal distribution.
  /// \param[in] sigma Standard deviation of the distribution.
  ///
  /// \sa uniform()
  template <typename S>
  static S normal(S mean, S sigma);

private:
  /// \return A mutable reference to the seed.
  static unsigned int& getSeedMutable();
};

} // namespace math
} // namespace dart

#include <dart/math/detail/Random-impl.hpp>

#endif // DART_MATH_RANDOM_HPP_
