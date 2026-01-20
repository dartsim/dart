/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include <dart/math/random.hpp>

#include <concepts>

namespace dart {
namespace math {

namespace detail {

//==============================================================================
// C++20 Concepts for type constraints

/// Check whether T can be used for std::uniform_int_distribution<T>
/// Reference:
/// https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
template <typename T>
concept UniformIntCompatible
    = std::same_as<std::remove_cv_t<T>, short>
      || std::same_as<std::remove_cv_t<T>, int>
      || std::same_as<std::remove_cv_t<T>, long>
      || std::same_as<std::remove_cv_t<T>, long long>
      || std::same_as<std::remove_cv_t<T>, unsigned short>
      || std::same_as<std::remove_cv_t<T>, unsigned int>
      || std::same_as<std::remove_cv_t<T>, unsigned long>
      || std::same_as<std::remove_cv_t<T>, unsigned long long>;

/// Check whether T is derived from Eigen::MatrixBase
template <typename T>
concept EigenMatrix = std::is_base_of_v<
    Eigen::MatrixBase<std::remove_cvref_t<T>>,
    std::remove_cvref_t<T>>;

//==============================================================================
// Kept for backward compatibility if needed elsewhere
template <template <typename...> class C, typename... Ts>
std::true_type is_base_of_template_impl(const C<Ts...>*);

template <template <typename...> class C>
std::false_type is_base_of_template_impl(...);

template <template <typename...> class C, typename T>
using is_base_of_template
    = decltype(is_base_of_template_impl<C>(std::declval<T*>()));

template <typename T>
using is_base_of_matrix = is_base_of_template<Eigen::MatrixBase, T>;

//==============================================================================
template <typename S>
struct UniformScalarImpl;

//==============================================================================
// Floating-point case
template <typename S>
  requires std::floating_point<S>
struct UniformScalarImpl<S>
{
  static S run(S min, S max)
  {
    // Distribution objects are lightweight so we simply construct a new
    // distribution for each random number generation.
    Random::UniformRealDist<S> d(min, max);
    return d(Random::getGenerator());
  }
};

//==============================================================================
// Integer case
template <typename S>
  requires UniformIntCompatible<S>
struct UniformScalarImpl<S>
{
  static S run(S min, S max)
  {
    // Distribution objects are lightweight so we simply construct a new
    // distribution for each random number generation.
    Random::UniformIntDist<S> d(min, max);
    return d(Random::getGenerator());
  }
};

//==============================================================================
template <typename Derived>
struct UniformMatrixImpl
{
  // Define nothing
};

//==============================================================================
// Dynamic matrix case
template <typename Derived>
  requires(
      EigenMatrix<Derived> && (Derived::IsVectorAtCompileTime == 0)
      && (Derived::SizeAtCompileTime == Eigen::Dynamic))
struct UniformMatrixImpl<Derived>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
    const auto minEval = min.eval();
    const auto maxEval = max.eval();
    typename Derived::PlainObject result(minEval.rows(), minEval.cols());
    for (Eigen::Index i = 0; i < minEval.rows(); ++i) {
      for (Eigen::Index j = 0; j < minEval.cols(); ++j) {
        result(i, j) = Random::uniform<typename Derived::Scalar>(
            minEval(i, j), maxEval(i, j));
      }
    }
    return result;
  }
};

//==============================================================================
// Dynamic vector case
template <typename Derived>
  requires(
      EigenMatrix<Derived> && (Derived::IsVectorAtCompileTime != 0)
      && (Derived::SizeAtCompileTime == Eigen::Dynamic))
struct UniformMatrixImpl<Derived>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
    const auto minEval = min.eval();
    const auto maxEval = max.eval();
    typename Derived::PlainObject result(minEval.size());
    for (Eigen::Index i = 0; i < minEval.size(); ++i) {
      result[i]
          = Random::uniform<typename Derived::Scalar>(minEval[i], maxEval[i]);
    }
    return result;
  }
};

//==============================================================================
// Fixed matrix case
template <typename Derived>
  requires(
      EigenMatrix<Derived> && (Derived::IsVectorAtCompileTime == 0)
      && (Derived::SizeAtCompileTime != Eigen::Dynamic))
struct UniformMatrixImpl<Derived>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
    const auto minEval = min.eval();
    const auto maxEval = max.eval();
    typename Derived::PlainObject result;
    for (Eigen::Index i = 0; i < minEval.rows(); ++i) {
      for (Eigen::Index j = 0; j < minEval.cols(); ++j) {
        result(i, j) = Random::uniform<typename Derived::Scalar>(
            minEval(i, j), maxEval(i, j));
      }
    }
    return result;
  }
};

//==============================================================================
// Fixed vector case
template <typename Derived>
  requires(
      EigenMatrix<Derived> && (Derived::IsVectorAtCompileTime != 0)
      && (Derived::SizeAtCompileTime != Eigen::Dynamic))
struct UniformMatrixImpl<Derived>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
    const auto minEval = min.eval();
    const auto maxEval = max.eval();
    typename Derived::PlainObject result;
    for (Eigen::Index i = 0; i < minEval.size(); ++i) {
      result[i]
          = Random::uniform<typename Derived::Scalar>(minEval[i], maxEval[i]);
    }
    return result;
  }
};

//==============================================================================
template <typename T>
struct UniformImpl;

//==============================================================================
template <typename T>
  requires std::is_arithmetic_v<T>
struct UniformImpl<T>
{
  static T run(T min, T max)
  {
    return UniformScalarImpl<T>::run(min, max);
  }
};

//==============================================================================
template <typename T>
  requires EigenMatrix<T>
struct UniformImpl<T>
{
  static T run(const Eigen::MatrixBase<T>& min, const Eigen::MatrixBase<T>& max)
  {
    return UniformMatrixImpl<T>::run(min, max);
  }
};

//==============================================================================
template <typename S>
struct NormalScalarImpl;

//==============================================================================
// Floating-point case
template <typename S>
  requires std::floating_point<S>
struct NormalScalarImpl<S>
{
  static S run(S mean, S sigma)
  {
    Random::NormalRealDist<S> d(mean, sigma);
    return d(Random::getGenerator());
  }
};

//==============================================================================
// Integer case - rounds normal distribution to nearest integer
template <typename S>
  requires UniformIntCompatible<S>
struct NormalScalarImpl<S>
{
  static S run(S mean, S sigma)
  {
    using DefaultFloatType = float;
    const DefaultFloatType realNormal = Random::normal(
        static_cast<DefaultFloatType>(mean),
        static_cast<DefaultFloatType>(sigma));
    return static_cast<S>(std::round(realNormal));
  }
};

//==============================================================================
template <typename T>
struct NormalImpl;

//==============================================================================
template <typename T>
  requires std::is_arithmetic_v<T>
struct NormalImpl<T>
{
  static T run(T min, T max)
  {
    return NormalScalarImpl<T>::run(min, max);
  }
};

} // namespace detail

//==============================================================================
template <typename S>
S Random::uniform(S min, S max)
{
  return detail::UniformImpl<S>::run(min, max);
}

//==============================================================================
template <typename FixedSizeT>
FixedSizeT Random::uniform(
    typename FixedSizeT::Scalar min, typename FixedSizeT::Scalar max)
{
  return uniform<FixedSizeT>(
      FixedSizeT::Constant(min), FixedSizeT::Constant(max));
}

//==============================================================================
template <typename DynamicSizeVectorT>
DynamicSizeVectorT Random::uniform(
    int size,
    typename DynamicSizeVectorT::Scalar min,
    typename DynamicSizeVectorT::Scalar max)
{
  return uniform<DynamicSizeVectorT>(
      DynamicSizeVectorT::Constant(size, min),
      DynamicSizeVectorT::Constant(size, max));
}

//==============================================================================
template <typename DynamicSizeMatrixT>
DynamicSizeMatrixT Random::uniform(
    int rows,
    int cols,
    typename DynamicSizeMatrixT::Scalar min,
    typename DynamicSizeMatrixT::Scalar max)
{
  return uniform<DynamicSizeMatrixT>(
      DynamicSizeMatrixT::Constant(rows, cols, min),
      DynamicSizeMatrixT::Constant(rows, cols, max));
}

//==============================================================================
template <typename S>
S Random::normal(S min, S max)
{
  return detail::NormalImpl<S>::run(min, max);
}

} // namespace math
} // namespace dart
