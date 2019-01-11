/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include "dart/math/Random.hpp"

//==============================================================================
// This workaround is necessary for old Eigen (< 3.3). See the details here:
// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1286
#if !EIGEN_VERSION_AT_LEAST(3, 3, 0)

namespace dart {
namespace math {
namespace detail {

template <typename Derived>
struct UniformScalarFromMatrixFunctor
{
  using S = typename Derived::Scalar;

  UniformScalarFromMatrixFunctor(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
    : mMin(min), mMax(max)
  {
    // Do nothing
  }

  S operator()(int i, int j) const
  {
    return Random::uniform<S>(mMin(i, j), mMax(i, j));
  }

  const Eigen::MatrixBase<Derived>& mMin;
  const Eigen::MatrixBase<Derived>& mMax;
};

template <typename Derived>
struct UniformScalarFromVectorFunctor
{
  using S = typename Derived::Scalar;

  UniformScalarFromVectorFunctor(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
    : mMin(min), mMax(max)
  {
    // Do nothing
  }

  S operator()(int i) const
  {
    return Random::uniform<S>(mMin[i], mMax[i]);
  }

  const Eigen::MatrixBase<Derived>& mMin;
  const Eigen::MatrixBase<Derived>& mMax;
};

} // namespace detail
} // namespace math
} // namespace dart

namespace Eigen {
namespace internal {

template <typename T>
struct functor_has_linear_access<
    dart::math::detail::UniformScalarFromMatrixFunctor<T>>
{
  enum
  {
    ret = false
  };
};

template <typename T>
struct functor_has_linear_access<
    dart::math::detail::UniformScalarFromVectorFunctor<T>>
{
  enum
  {
    ret = true
  };
};

} // namespace internal
} // namespace Eigen

#endif // !EIGEN_VERSION_AT_LEAST(3,3,0)

namespace dart {
namespace math {

namespace {

//==============================================================================
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
/// Check whether \c T can be used for std::uniform_int_distribution<T>
/// Reference:
/// https://en.cppreference.com/w/cpp/numeric/random/uniform_int_distribution
template <typename T, typename Enable = void>
struct is_compatible_to_uniform_int_distribution : std::false_type
{
  // Define nothing
};

// clang-format off

template <typename T>
struct is_compatible_to_uniform_int_distribution<
    T, typename std::enable_if<
        std::is_same<typename std::remove_cv<T>::type, short>::value
        || std::is_same<typename std::remove_cv<T>::type, int>::value
        || std::is_same<typename std::remove_cv<T>::type, long>::value
        || std::is_same<typename std::remove_cv<T>::type, long long>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned short>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned int>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned long>::value
        || std::is_same<typename std::remove_cv<T>::type, unsigned long long>::value
        >::type
    > : std::true_type
{
  // Define nothing
};

// clang-format on

//==============================================================================
template <typename S, typename Enable = void>
struct UniformScalarImpl
{
  // Define nothing
};

//==============================================================================
// Floating-point case
template <typename S>
struct UniformScalarImpl<
    S,
    typename std::enable_if<std::is_floating_point<S>::value>::type>
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
// Floating-point case
template <typename S>
struct UniformScalarImpl<
    S,
    typename std::enable_if<
        is_compatible_to_uniform_int_distribution<S>::value>::type>
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
template <typename Derived, typename Enable = void>
struct UniformMatrixImpl
{
  // Define nothing
};

//==============================================================================
// Dynamic matrix case
template <typename Derived>
struct UniformMatrixImpl<
    Derived,
    typename std::enable_if<
        !Derived::IsVectorAtCompileTime
        && Derived::SizeAtCompileTime == Eigen::Dynamic>::type>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
    const auto uniformFunc = [&](int i, int j) {
      return Random::uniform<typename Derived::Scalar>(min(i, j), max(i, j));
    };
    return Derived::PlainObject::NullaryExpr(
        min.rows(), min.cols(), uniformFunc);
#else
    return Derived::PlainObject::NullaryExpr(
        min.rows(),
        min.cols(),
        detail::UniformScalarFromMatrixFunctor<Derived>(min, max));
#endif
  }
};

//==============================================================================
// Dynamic vector case
template <typename Derived>
struct UniformMatrixImpl<
    Derived,
    typename std::enable_if<
        Derived::IsVectorAtCompileTime
        && Derived::SizeAtCompileTime == Eigen::Dynamic>::type>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
    const auto uniformFunc = [&](int i) {
      return Random::uniform<typename Derived::Scalar>(min[i], max[i]);
    };
    return Derived::PlainObject::NullaryExpr(min.size(), uniformFunc);
#else
    return Derived::PlainObject::NullaryExpr(
        min.size(), detail::UniformScalarFromVectorFunctor<Derived>(min, max));
#endif
  }
};

//==============================================================================
// Fixed matrix case
template <typename Derived>
struct UniformMatrixImpl<
    Derived,
    typename std::enable_if<
        !Derived::IsVectorAtCompileTime
        && Derived::SizeAtCompileTime != Eigen::Dynamic>::type>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
    const auto uniformFunc = [&](int i, int j) {
      return Random::uniform<typename Derived::Scalar>(min(i, j), max(i, j));
    };
    return Derived::PlainObject::NullaryExpr(uniformFunc);
#else
    return Derived::PlainObject::NullaryExpr(
        detail::UniformScalarFromMatrixFunctor<Derived>(min, max));
#endif
  }
};

//==============================================================================
// Fixed vector case
template <typename Derived>
struct UniformMatrixImpl<
    Derived,
    typename std::enable_if<
        Derived::IsVectorAtCompileTime
        && Derived::SizeAtCompileTime != Eigen::Dynamic>::type>
{
  static typename Derived::PlainObject run(
      const Eigen::MatrixBase<Derived>& min,
      const Eigen::MatrixBase<Derived>& max)
  {
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
    const auto uniformFunc = [&](int i) {
      return Random::uniform<typename Derived::Scalar>(min[i], max[i]);
    };
    return Derived::PlainObject::NullaryExpr(uniformFunc);
#else
    return Derived::PlainObject::NullaryExpr(
        detail::UniformScalarFromVectorFunctor<Derived>(min, max));
#endif
  }
};

//==============================================================================
template <typename T, typename Enable = void>
struct UniformImpl
{
  // Define nothing
};

//==============================================================================
template <typename T>
struct UniformImpl<
    T,
    typename std::enable_if<std::is_arithmetic<T>::value>::type>
{
  static T run(T min, T max)
  {
    return UniformScalarImpl<T>::run(min, max);
  }
};

//==============================================================================
template <typename T>
struct UniformImpl<
    T,
    typename std::enable_if<is_base_of_matrix<T>::value>::type>
{
  static T run(const Eigen::MatrixBase<T>& min, const Eigen::MatrixBase<T>& max)
  {
    return UniformMatrixImpl<T>::run(min, max);
  }
};

//==============================================================================
template <typename S, typename Enable = void>
struct NormalScalarImpl
{
  // Define nothing
};

//==============================================================================
// Floating-point case
template <typename S>
struct NormalScalarImpl<
    S,
    typename std::enable_if<std::is_floating_point<S>::value>::type>
{
  static S run(S mean, S sigma)
  {
    Random::NormalRealDist<S> d(mean, sigma);
    return d(Random::getGenerator());
  }
};

//==============================================================================
// Floating-point case
template <typename S>
struct NormalScalarImpl<
    S,
    typename std::enable_if<
        is_compatible_to_uniform_int_distribution<S>::value>::type>
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
template <typename T, typename Enable = void>
struct NormalImpl
{
  // Define nothing
};

//==============================================================================
template <typename T>
struct NormalImpl<
    T,
    typename std::enable_if<std::is_arithmetic<T>::value>::type>
{
  static T run(T min, T max)
  {
    return NormalScalarImpl<T>::run(min, max);
  }
};

} // namespace

//==============================================================================
template <typename S>
S Random::uniform(S min, S max)
{
  return UniformImpl<S>::run(min, max);
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
  return NormalImpl<S>::run(min, max);
}

} // namespace math
} // namespace dart
