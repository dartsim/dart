/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#define DART_LIE_GROUP_BASE_TYPES                                              \
  /** The Eigen options. */                                                    \
  static constexpr int Options = ::Eigen::internal::traits<Derived>::Options;  \
                                                                               \
  /** The dimension of the Euclidean space that the group is embedded. */      \
  static constexpr int SpaceDim                                                \
      = ::Eigen::internal::traits<Derived>::SpaceDim;                          \
                                                                               \
  /** The dimension (or degrees of freedom) of the Lie group. */               \
  static constexpr int GroupDim                                                \
      = ::Eigen::internal::traits<Derived>::GroupDim;                          \
                                                                               \
  /** The dimension of the matrix representation of the Lie group. */          \
  static constexpr int MatrixDim                                               \
      = ::Eigen::internal::traits<Derived>::MatrixDim;                         \
                                                                               \
  /** The dimension of internal parameters to represent the Lie group element. \
   */                                                                          \
  static constexpr int DataDim = ::Eigen::internal::traits<Derived>::DataDim;  \
                                                                               \
  /** The scalar type of the Lie group. */                                     \
  using Scalar = typename ::Eigen::internal::traits<Derived>::Scalar;          \
                                                                               \
  /** The matrix type of the Lie group. */                                     \
  using LieGroupMatrix = ::Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;        \
                                                                               \
  /** Lie group type of Derived */                                             \
  using LieGroup = typename ::Eigen::internal::traits<Derived>::LieGroup;      \
                                                                               \
  /** Lie group data type of Derived */                                        \
  using LieGroupData =                                                         \
      typename ::Eigen::internal::traits<Derived>::LieGroupData;               \
                                                                               \
  /** Lie algebra type of Derived */                                           \
  using LieAlgebra = typename ::Eigen::internal::traits<Derived>::LieAlgebra;  \
                                                                               \
  /** Lie algebra data type of Derived */                                      \
  using LieAlgebraData =                                                       \
      typename ::Eigen::internal::traits<Derived>::LieAlgebraData;             \
                                                                               \
  /** Tangent type of Derived */                                               \
  using Tangent = typename ::Eigen::internal::traits<Derived>::Tangent;        \
                                                                               \
  /** Tangent data type of Derived */                                          \
  using TangentData =                                                          \
      typename ::Eigen::internal::traits<Derived>::TangentData;                \
                                                                               \
  /** Cotangent type of Derived */                                             \
  using Cotangent = typename ::Eigen::internal::traits<Derived>::Cotangent;    \
                                                                               \
  /** Cotangent data type of Derived */                                        \
  using CotangentData =                                                        \
      typename ::Eigen::internal::traits<Derived>::CotangentData;              \
                                                                               \
  using Jacobian = typename ::Eigen::internal::traits<Derived>::Jacobian;      \
                                                                               \
  void _ANONYMOUS_FUNCTION_1()

#define DART_LIE_GROUP_BASE_ASSIGN_OPERATORS(X)                                \
  Derived& operator=(const X& other)                                           \
  {                                                                            \
    derived().data() = other.data();                                           \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  Derived& operator=(X&& other)                                                \
  {                                                                            \
    derived().data() = std::move(other.data());                                \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename OtherDerived>                                             \
  Derived& operator=(const X<OtherDerived>& other)                             \
  {                                                                            \
    derived().data() = other.data();                                           \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename OtherDerived>                                             \
  Derived& operator=(X<OtherDerived>&& other)                                  \
  {                                                                            \
    derived().data() = std::move(other.data());                                \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename EigenDerived>                                             \
  Derived& operator=(const ::Eigen::MatrixBase<EigenDerived>& other)           \
  {                                                                            \
    derived().data() = other;                                                  \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename EigenDerived>                                             \
  Derived& operator=(::Eigen::MatrixBase<EigenDerived>&& other)                \
  {                                                                            \
    derived().data() = std::move(other);                                       \
    return derived();                                                          \
  }

#define DART_LIE_GROUP_BASE_DERIVED                                            \
  /** Returns as the derived class */                                          \
  [[nodiscard]] const Derived& derived() const noexcept                        \
  {                                                                            \
    return *static_cast<const Derived*>(this);                                 \
  }                                                                            \
                                                                               \
  /** Returns as the mutable derived class */                                  \
  [[nodiscard]] Derived& derived() noexcept                                    \
  {                                                                            \
    return *static_cast<Derived*>(this);                                       \
  }

#define DART_LIE_GROUP_BASE_DATA(X)                                            \
  /** Returns the data of the drived class */                                  \
  [[nodiscard]] const X& coeffs() const                                        \
  {                                                                            \
    return derived().coeffs();                                                 \
  }                                                                            \
                                                                               \
  /** Returns the mutable data of the drived class */                          \
  [[nodiscard]] X& coeffs()                                                    \
  {                                                                            \
    return derived().coeffs();                                                 \
  }                                                                            \
                                                                               \
  /** Returns the pointer to the data of the drived class */                   \
  [[nodiscard]] const Scalar* data() const                                     \
  {                                                                            \
    return coeffs().data();                                                    \
  }                                                                            \
                                                                               \
  /** Returns the pointer to the mutable data of the drived class */           \
  [[nodiscard]] Scalar* data()                                                 \
  {                                                                            \
    return coeffs().data();                                                    \
  }

#define DART_LIE_GROUP_TRAITS_TYPES(X)                                         \
  using Scalar = Scalar_;                                                      \
                                                                               \
  using LieGroup = dart::math::X<Scalar, Options>;                             \
  using LieGroupData = Eigen::Matrix<Scalar, DataDim, 1>;                      \
                                                                               \
  using LieAlgebra = dart::math::X##Algebra<Scalar, Options>;                  \
  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;          \
                                                                               \
  using Tangent = dart::math::X##Tangent<Scalar, Options>;                     \
  using TangentData = Eigen::Matrix<Scalar, SpaceDim, 1>;                      \
                                                                               \
  using Cotangent = dart::math::X##Cotangent<Scalar, Options>;                 \
  using CotangentData = Eigen::Matrix<Scalar, 1, SpaceDim>;                    \
                                                                               \
  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

#define DART_LIE_GROUP_TRAITS_TYPES_FOR_R(X)                                   \
  using Scalar = Scalar_;                                                      \
                                                                               \
  using LieGroup = dart::math::X<Scalar, GroupDim, Options>;                   \
  using LieGroupData = Eigen::Matrix<Scalar, DataDim, 1>;                      \
                                                                               \
  using LieAlgebra = dart::math::X##Algebra<Scalar, GroupDim, Options>;        \
  using LieAlgebraData = Eigen::Matrix<Scalar, MatrixDim, MatrixDim>;          \
                                                                               \
  using Tangent = dart::math::X##Tangent<Scalar, GroupDim, Options>;           \
  using TangentData = Eigen::Matrix<Scalar, SpaceDim, 1>;                      \
                                                                               \
  using Cotangent = dart::math::X##Cotangent<Scalar, GroupDim, Options>;       \
  using CotangentData = Eigen::Matrix<Scalar, SpaceDim, 1>;                    \
                                                                               \
  using Jacobian = Eigen::Matrix<Scalar, GroupDim, GroupDim, Options>;

#define DART_LIE_GROUP_USE_BASE_TYPES                                          \
  using Base::Options;                                                         \
  using Base::SpaceDim;                                                        \
  using Base::GroupDim;                                                        \
  using Base::MatrixDim;                                                       \
  using Base::DataDim;                                                         \
  using Scalar = typename Base::Scalar;                                        \
  using LieGroup = typename Base::LieGroup;                                    \
  using LieGroupData = typename Base::LieGroupData;                            \
  using LieAlgebra = typename Base::LieAlgebra;                                \
  using LieAlgebraData = typename Base::LieAlgebraData;                        \
  using Tangent = typename Base::Tangent;                                      \
  using TangentData = typename Base::TangentData;                              \
  using Cotangent = typename Base::Cotangent;                                  \
  using CotangentData = typename Base::CotangentData;                          \
  using Jacobian = typename Base::Jacobian;

#define DART_LIE_GROUP_USE_BASE_GROUP_CONST_OPERATIONS                         \
  /** ... */                                                                   \
  using Base::inverse;

#define DART_LIE_GROUP_USE_BASE_GROUP_OPERATIONS                               \
  DART_LIE_GROUP_USE_BASE_GROUP_CONST_OPERATIONS                               \
  using Base::inverse_in_place;                                                \
  void _ANONYMOUS_FUNCTION_9()

#define DART_LIE_GROUP_USE_BASE_OPERATORS                                      \
  using Base::operator=;                                                       \
  using Base::operator*;

#define DART_LIE_GROUP_CONSTRUCTORS(X)                                         \
  /** Copy constructor*/                                                       \
  X(const X& other) : m_data(other.coeffs())                                   \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /** Move constructor */                                                      \
  X(X&& other) : m_data(std::move(other.coeffs()))                             \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /** Copy constructor from X##Base */                                         \
  template <typename OtherDerived>                                             \
  X(const X##Base<OtherDerived>& other) : m_data(other.coeffs())               \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /** Move constructor from X##Base */                                         \
  template <typename OtherDerived>                                             \
  X(X##Base<OtherDerived>&& other) : m_data(std::move(other.coeffs()))         \
  {                                                                            \
    /* Do nothing */                                                           \
  }                                                                            \
                                                                               \
  /** Constructs from MatrixBase */                                            \
  template <typename MatrixDerived>                                            \
  X(const Eigen::MatrixBase<MatrixDerived>& vector)                            \
    : m_data(std::move(vector))                                                \
  {                                                                            \
    detail::NormalizationOperator<typename X::Base>::run(*this);               \
  }                                                                            \
                                                                               \
  /** Constructs from MatrixBase */                                            \
  template <typename MatrixDerived>                                            \
  X(Eigen::MatrixBase<MatrixDerived>&& vector) : m_data(std::move(vector))     \
  {                                                                            \
    detail::NormalizationOperator<typename X::Base>::run(*this);               \
  }

#define DART_LIE_GROUP_ASSIGN_OPERATORS(X)                                     \
  X& operator=(const X& other)                                                 \
  {                                                                            \
    coeffs() = other.coeffs();                                                 \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  X& operator=(X&& other)                                                      \
  {                                                                            \
    coeffs() = std::move(other.coeffs());                                      \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename OtherDerived_>                                            \
  X& operator=(const X##Base<OtherDerived_>& other)                            \
  {                                                                            \
    coeffs() = other.coeffs();                                                 \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename OtherDerived_>                                            \
  X& operator=(X##Base<OtherDerived_>&& other)                                 \
  {                                                                            \
    coeffs() = std::move(other.coeffs());                                      \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename OtherDerived_>                                            \
  X& operator=(const LieGroupBase<OtherDerived_>& other)                       \
  {                                                                            \
    coeffs() = other.coeffs();                                                 \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename OtherDerived_>                                            \
  X& operator=(LieGroupBase<OtherDerived_>&& other)                            \
  {                                                                            \
    coeffs() = std::move(other.coeffs());                                      \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename MatrixDerived_>                                           \
  X& operator=(const Eigen::MatrixBase<MatrixDerived_>& other)                 \
  {                                                                            \
    coeffs() = other.coeffs();                                                 \
    detail::NormalizationOperator<typename X::Base>::run(*this);               \
    return derived();                                                          \
  }                                                                            \
                                                                               \
  template <typename MatrixDerived_>                                           \
  X& operator=(Eigen::MatrixBase<MatrixDerived_>&& other)                      \
  {                                                                            \
    coeffs() = std::move(other.coeffs());                                      \
    detail::NormalizationOperator<typename X::Base>::run(*this);               \
    return derived();                                                          \
  }

#define DART_LIE_GROUP_MAP_ASSIGN_OPERATORS(X)                                 \
  Map(const Map& other) : Base(), m_data(other.m_data) {}                      \
  Map(Map&& other) : Base(), m_data(std::move(other.m_data)) {}                \
  Map& operator=(const Map& other)                                             \
  {                                                                            \
    coeffs() = other.coeffs();                                                 \
    return *this;                                                              \
  }                                                                            \
  Map& operator=(Map&& other)                                                  \
  {                                                                            \
    coeffs() = std::move(other.coeffs());                                      \
    return *this;                                                              \
  }                                                                            \
  template <typename OtherDerived>                                             \
  Map& operator=(const ::dart::math::X##Base<OtherDerived>& other)             \
  {                                                                            \
    coeffs() = other.coeffs();                                                 \
    return *this;                                                              \
  }                                                                            \
  template <typename OtherDerived>                                             \
  Map& operator=(::dart::math::X##Base<OtherDerived>&& other)                  \
  {                                                                            \
    coeffs() = std::move(other.coeffs());                                      \
    return *this;                                                              \
  }
