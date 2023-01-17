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

#pragma once

#include <dart/math/Fwd.hpp>

#include <random>

namespace dart::math {

/// The Random class is a consolidated random number generator that provides a
/// unified and convenient interface for generating various types and
/// distributions of random numbers. It utilizes the <random> library of the C++
/// standard library and offers APIs for generating random numbers in the form
/// of scalars, vectors, and matrices.
///
/// The following scalar types are supported:
/// * Integers
/// * Floating-point numbers
///
/// The supported distributions include:
/// * Uniform
/// * Normal (or Gaussian)
///
/// The default engine used in this class is the high-quality std::mt19937,
/// which is a 32-bit random number generator based on the Mersenne Twister
/// algorithm. However, this can be replaced with any preferred engine by
/// specifying the template argument. For more information on available engines
/// and their specific characteristics, see
/// https://en.cppreference.com/w/cpp/numeric/random.
///
/// @tparam Generator_ The random number generator to use. The default is
/// std::mt19937.
template <typename Generator_ = std::mt19937>
class Random final
{
public:
  /// The type of the random number generator.
  using GeneratorType = Generator_;

  /// Returns a reference to the singleton instance of the random generator.
  ///
  /// @note This function is thread-safe.
  /// @return The singleton instance of the random generator.
  [[nodiscard]] static Random& GetInstance();

  /// Constructs a random number generator with the given seed.
  ///
  /// @param seed The seed of the random number generator.
  explicit Random(uint32_t seed = std::random_device{}());

  /// Returns the seed of the random number generator.
  [[nodiscard]] uint32_t getSeed() const;

  /// Sets the seed of the random number generator.
  /// @param seed The seed of the random number generator.
  void setSeed(uint32_t seed);

  /// Returns a random number from an uniform distribution.
  ///
  /// This template function can generate different types of random numbers,
  /// including:
  /// - Scalars: floating-point numbers (e.g. float, double, long double) and
  /// integers (e.g. short, int, long, long long)
  /// - Vectors and Matrices: both fixed-size (e.g. Vector3i, Matrix4d) and
  /// dynamic-size (e.g. VectorXi, MatrixXd)
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate a random int in the range [0, 10] (inclusive)
  /// int intVal1 = rng.uniform(0, 10);
  /// int intVal2 = rng.uniform<int>(0, 10);
  ///
  /// // Generate a random double in the range [0.0, 10.0) (exclusive)
  /// double dblVal1 = rng.uniform(0.0, 10.0);
  /// double dblVal2 = rng.uniform<double>(0, 10);
  ///
  /// // Generate a random Vector3d in the range [lb, ub) (exclusive)
  /// Vector3d lb = Vector3d::Constant(1);
  /// Vector3d ub = Vector3d::Constant(4);
  /// Vector3d vecVal1 = rng.uniform(lb, ub);
  /// Vector3d vecVal2 = rng.uniform<Vector3d>(lb, ub);
  ///
  /// // Generate a random Matrix4f in the range [lb, ub) (exclusive)
  /// Matrix4f lb = Matrix4f::Constant(1);
  /// Matrix4f ub = Matrix4f::Constant(4);
  /// Matrix4f vecVal1 = rng.uniform(lb, ub);
  /// Matrix4f vecVal2 = rng.uniform<Matrix4f>(lb, ub);
  /// @endcode
  ///
  /// Note that the upper bound of the range is closed for integer types, but
  /// open for floating-point types.
  ///
  /// @tparam T The type of random value.
  /// @param[in] min Lower bound of the distribution.
  /// @param[in] max Upper bound of the distribution.
  ///
  /// @sa normal()
  template <typename T>
  [[nodiscard]] T uniform(const T& min, const T& max);

  /// Returns a random fixed-size vector or matrix from an uniform distribution.
  ///
  /// This function is a helper for generating a fixed-size vector or matrix,
  /// where each element has the same uniform distribution.
  ///
  /// For example, if the lower bound is [1, 1, 1], each element of the vector
  /// would be randomly generated in the range [1, 1, 1] and if the upper bound
  /// is [-2, -2], each element of the vector would be randomly generated in the
  /// range [-2, -2].
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate random fixed-size vectors
  /// VectorXi vecXi = rng.uniform<VectorXi>(0, 10);
  /// VectorXd vecXd = rng.uniform<VectorXd>(0.0, 10.0);
  /// @endcode
  ///
  /// @tparam FixedSizeT The type of fixed-size vector or fixed-size matrix.
  /// @param[in] min The constant value of the lower bound for each element.
  /// @param[in] max The constant value of the upper bound for each element.
  ///
  /// @sa uniform()
  template <typename FixedSizeT>
  [[nodiscard]] FixedSizeT uniform(
      typename FixedSizeT::Scalar min, typename FixedSizeT::Scalar max);

  /// Returns a random dynamic-size vector from an uniform distribution.
  ///
  /// This function is used to generate a dynamic-size vector, where each
  /// element has the same uniform distribution.
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate random dynamic-size vectors
  /// VectorXd vecXd = rng.uniform<VectorXd>(10, 0.0, 10.0);
  /// VectorXi vecXi = rng.uniform<VectorXi>(5, 0, 10);
  /// @endcode
  ///
  /// @tparam DynamicSizeVectorT The type of dynamic-size vector.
  /// @param[in] size The size of the vector.
  /// @param[in] min The constant value of the lower bound for each element.
  /// @param[in] max The constant value of the upper bound for each element.
  template <typename DynamicSizeVectorT>
  [[nodiscard]] DynamicSizeVectorT uniform(
      int size,
      typename DynamicSizeVectorT::Scalar min,
      typename DynamicSizeVectorT::Scalar max);

  /// Returns a random dynamic-size matrix from an uniform distribution.
  ///
  /// This function is used to generate a dynamic-size matrix, where each
  /// element has the same uniform distribution.
  ///
  /// @tparam DynamicSizeMatrixT The type of dynamic-size matrix.
  /// @param[in] rows The number of rows in the matrix.
  /// @param[in] cols The number of columns in the matrix.
  /// @param[in] min The constant value of the lower bound for each element.
  /// @param[in] max The constant value of the upper bound for each element.
  ///
  /// @sa uniform()
  template <typename DynamicSizeMatrixT>
  [[nodiscard]] DynamicSizeMatrixT uniform(
      int rows,
      int cols,
      typename DynamicSizeMatrixT::Scalar min,
      typename DynamicSizeMatrixT::Scalar max);

  /// Returns a random number from a normal (or Gaussian) distribution.
  ///
  /// This template function can generate different scalar types of random
  /// numbers, including:
  /// - Floating-point numbers: float, double, long double
  /// - Integer numbers: [unsigned] short, [unsigned] int, [unsigned] long,
  /// [unsigned] long long
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate a random int
  /// int intVal = rng.normal(0, 10);
  ///
  /// // Generate a random double
  /// double dblVal = rng.normal(0.0, 10.0);
  /// @endcode
  ///
  /// @param[in] mean Mean value of the normal distribution.
  /// @param[in] sigma Standard deviation of the distribution.
  ///
  /// @sa uniform()
  template <typename T>
  [[nodiscard]] T normal(const T& mean, const T& sigma);

  /// Returns a random fixed-size vector or matrix from a normal distribution.
  ///
  /// This function is a helper for generating a fixed-size vector or matrix,
  /// where each element has the same normal distribution.
  ///
  /// For example, if the mean is [1, 1, 1], each element of the vector would be
  /// randomly generated from a normal distribution with mean [1, 1, 1] and if
  /// the standard deviation is [-2, -2], each element of the vector would be
  /// randomly generated from a normal distribution with standard deviation
  /// [-2, -2].
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate random fixed-size vectors
  /// VectorXi vecXi = rng.normal<VectorXi>(0, 10);
  /// VectorXd vecXd = rng.normal<VectorXd>(0.0, 10.0);
  /// @endcode
  ///
  /// @tparam FixedSizeT The type of fixed-size vector or fixed-size matrix.
  /// @param[in] mean The constant value of the mean for each element.
  /// @param[in] sigma The constant value of the standard deviation for each
  /// element.
  ///
  /// @sa normal()
  template <typename FixedSizeT>
  [[nodiscard]] FixedSizeT normal(
      typename FixedSizeT::Scalar mean, typename FixedSizeT::Scalar sigma);

  /// Returns a random dynamic-size vector from a normal distribution.
  ///
  /// This function is used to generate a dynamic-size vector, where each
  /// element has the same normal distribution.
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate random dynamic-size vectors
  /// VectorXd vecXd = rng.normal<VectorXd>(10, 0.0, 10.0);
  /// VectorXi vecXi = rng.normal<VectorXi>(5, 0, 10);
  /// @endcode
  ///
  /// @tparam DynamicSizeVectorT The type of dynamic-size vector.
  /// @param[in] size The size of the vector.
  /// @param[in] mean The constant value of the mean for each element.
  /// @param[in] sigma The constant value of the standard deviation for each
  /// element.
  template <typename DynamicSizeVectorT>
  [[nodiscard]] DynamicSizeVectorT normal(
      int size,
      typename DynamicSizeVectorT::Scalar mean,
      typename DynamicSizeVectorT::Scalar sigma);

  /// Returns a random dynamic-size matrix from a normal distribution.
  ///
  /// This function is used to generate a dynamic-size matrix, where each
  /// element has the same normal distribution.
  ///
  /// Example usage:
  /// @code
  /// auto rng = Random();
  ///
  /// // Generate random dynamic-size matrices
  /// MatrixXd matXd = rng.normal<MatrixXd>(10, 5, 0.0, 10.0);
  /// MatrixXi matXi = rng.normal<MatrixXi>(5, 10, 0, 10);
  /// @endcode
  ///
  /// @tparam DynamicSizeMatrixT The type of dynamic-size matrix.
  /// @param[in] rows The number of rows in the matrix.
  /// @param[in] cols The number of columns in the matrix.
  /// @param[in] mean The constant value of the mean for each element.
  /// @param[in] sigma The constant value of the standard deviation for each
  /// element.
  template <typename DynamicSizeMatrixT>
  [[nodiscard]] DynamicSizeMatrixT normal(
      int rows,
      int cols,
      typename DynamicSizeMatrixT::Scalar mean,
      typename DynamicSizeMatrixT::Scalar sigma);

private:
  /// The seed of the random number generator.
  uint32_t mSeed;

  /// The random number generator.
  GeneratorType mGenerator;
};

/// @brief Returns a random number from a uniform distribution between a given
/// range.
/// @tparam T The type of random number.
/// @param[in] min Lower bound of the distribution.
/// @param[in] max Upper bound of the distribution.
/// @return A random number from the uniform distribution.
template <typename T>
[[nodiscard]] T Uniform(const T& min, const T& max);

/// @brief Returns a random fixed-size vector or matrix from an uniform
/// distribution.
/// @tparam FixedSizeT The type of fixed-size vector or fixed-size matrix.
/// @param[in] min The constant value of the lower bound for each element.
/// @param[in] max The constant value of the upper bound for each element.
/// @return A random fixed-size vector or matrix from the uniform distribution.
template <typename FixedSizeT>
[[nodiscard]] FixedSizeT Uniform(
    typename FixedSizeT::Scalar min, typename FixedSizeT::Scalar max);

/// Returns a random dynamic-size vector from an uniform distribution.
/// @tparam DynamicSizeVectorT The type of dynamic-size vector.
/// @param[in] size The size of the vector.
/// @param[in] min The constant value of the lower bound for each element.
/// @param[in] max The constant value of the upper bound for each element.
/// @return A random dynamic-size vector from an uniform distribution.
template <typename DynamicSizeVectorT>
[[nodiscard]] DynamicSizeVectorT Uniform(
    int size,
    typename DynamicSizeVectorT::Scalar min,
    typename DynamicSizeVectorT::Scalar max);

/// Returns a random dynamic-size matrix from an uniform distribution.
/// @tparam DynamicSizeMatrixT The type of dynamic-size matrix.
/// @param[in] rows The number of rows in the matrix.
/// @param[in] cols The number of columns in the matrix.
/// @param[in] min The constant value of the lower bound for each element.
/// @param[in] max The constant value of the upper bound for each element.
/// @return A random dynamic-size matrix from an uniform distribution.
template <typename DynamicSizeMatrixT>
[[nodiscard]] DynamicSizeMatrixT Uniform(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar min,
    typename DynamicSizeMatrixT::Scalar max);

/// Returns a random number from a normal (or Gaussian) distribution.
/// @tparam S The type of random number.
/// @param[in] mean Mean value of the normal distribution.
/// @param[in] sigma Standard deviation of the distribution.
/// @return A random number from the normal distribution.
template <typename T>
[[nodiscard]] T Normal(const T& mean, const T& sigma);

/// Returns a random fixed-size vector or matrix from a normal distribution.
/// @tparam FixedSizeT The type of fixed-size vector or fixed-size matrix.
/// @param[in] mean The constant value of the mean for each element.
/// @param[in] sigma The constant value of the standard deviation for each
/// element.
/// @return A random fixed-size vector or matrix from the normal distribution.
template <typename FixedSizeT>
[[nodiscard]] FixedSizeT Normal(
    typename FixedSizeT::Scalar mean, typename FixedSizeT::Scalar sigma);

/// Returns a random dynamic-size vector from a normal distribution.
/// @tparam DynamicSizeVectorT The type of dynamic-size vector.
/// @param[in] size The size of the vector.
/// @param[in] mean The constant value of the mean for each element.
/// @param[in] sigma The constant value of the standard deviation for each
/// element.
/// @return A random dynamic-size vector from a normal distribution.
template <typename DynamicSizeVectorT>
[[nodiscard]] DynamicSizeVectorT Normal(
    int size,
    typename DynamicSizeVectorT::Scalar mean,
    typename DynamicSizeVectorT::Scalar sigma);

/// Returns a random dynamic-size matrix from a normal distribution.
/// @tparam DynamicSizeMatrixT The type of dynamic-size matrix.
/// @param[in] rows The number of rows in the matrix.
/// @param[in] cols The number of columns in the matrix.
/// @param[in] mean The constant value of the mean for each element.
/// @param[in] sigma The constant value of the standard deviation for each
/// element.
/// @return A random dynamic-size matrix from a normal distribution.
template <typename DynamicSizeMatrixT>
[[nodiscard]] DynamicSizeMatrixT Normal(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar mean,
    typename DynamicSizeMatrixT::Scalar sigma);

} // namespace dart::math

#include <dart/math/detail/Random-impl.hpp>
