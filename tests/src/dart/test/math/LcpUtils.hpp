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

#include <dart/math/lcp/Lcp.hpp>

namespace dart::test {

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem1();

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem2();

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem4();

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem6();

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem12();

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem();

template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateValidLcpProblem();

} // namespace dart::test

//==============================================================================
// Implementation
//==============================================================================

#include <dart/math/Random.hpp>

namespace dart::test {

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem1()
{
  math::MatrixX<S> A;
  A.resize(1, 1);
  A << 1;

  math::VectorX<S> b;
  b.resize(1);
  b << -1.5;

  return {A, b};
}

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem2()
{
  math::MatrixX<S> A;
  math::VectorX<S> b;
  A.resize(2, 2);
  b.resize(2);

  A << 3.12, 0.1877, 0.1877, 3.254;
  b << -0.00662, -0.006711;

  return {A, b};
}

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem4()
{
  math::MatrixX<S> A;
  math::VectorX<S> b;
  A.resize(4, 4);
  b.resize(4);

  A << 3.999, 0.9985, 1.001, -2, 0.9985, 3.998, -2, 0.9995, 1.001, -2, 4.002,
      1.001, -2, 0.9995, 1.001, 4.001;
  b << -0.01008, -0.009494, -0.07234, -0.07177;

  return {A, b};
}

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem6()
{
  math::MatrixX<S> A;
  math::VectorX<S> b;
  A.resize(6, 6);
  b.resize(6);

  A << 3.1360, -2.0370, 0.9723, 0.1096, -2.0370, 0.9723, -2.0370, 3.7820,
      0.8302, -0.0257, 2.4730, 0.0105, 0.9723, 0.8302, 5.1250, -2.2390, -1.9120,
      3.4080, 0.1096, -0.0257, -2.2390, 3.1010, -0.0257, -2.2390, -2.0370,
      2.4730, -1.9120, -0.0257, 5.4870, -0.0242, 0.9723, 0.0105, 3.4080,
      -2.2390, -0.0242, 3.3860;
  b << 0.1649, -0.0025, -0.0904, -0.0093, -0.0000, -0.0889;

  return {A, b};
}

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem12()
{
  math::MatrixX<S> A;
  math::VectorX<S> b;
  A.resize(12, 12);
  b.resize(12);

  A << 4.03, -1.014, -1.898, 1.03, -1.014, -1.898, 1, -1.014, -1.898, -2,
      -1.014, -1.898, -1.014, 4.885, -1.259, 1.888, 3.81, 2.345, -1.879, 1.281,
      -2.334, 1.022, 0.206, 1.27, -1.898, -1.259, 3.2, -1.032, -0.6849, 1.275,
      1.003, 0.6657, 3.774, 1.869, 1.24, 1.85, 1.03, 1.888, -1.032, 4.03, 1.888,
      -1.032, -2, 1.888, -1.032, 1, 1.888, -1.032, -1.014, 3.81, -0.6849, 1.888,
      3.225, 1.275, -1.879, 1.85, -1.27, 1.022, 1.265, 0.6907, -1.898, 2.345,
      1.275, -1.032, 1.275, 4.86, 1.003, -1.24, 0.2059, 1.869, -2.309, 3.791, 1,
      -1.879, 1.003, -2, -1.879, 1.003, 3.97, -1.879, 1.003, 0.9703, -1.879,
      1.003, -1.014, 1.281, 0.6657, 1.888, 1.85, -1.24, -1.879, 3.187, 1.234,
      1.022, 3.755, -0.6714, -1.898, -2.334, 3.774, -1.032, -1.27, 0.2059,
      1.003, 1.234, 4.839, 1.869, 2.299, 1.27, -2, 1.022, 1.869, 1, 1.022,
      1.869, 0.9703, 1.022, 1.869, 3.97, 1.022, 1.869, -1.014, 0.206, 1.24,
      1.888, 1.265, -2.309, -1.879, 3.755, 2.299, 1.022, 4.814, -1.25, -1.898,
      1.27, 1.85, -1.032, 0.6907, 3.791, 1.003, -0.6714, 1.27, 1.869, -1.25,
      3.212;
  b << -0.00981, -1.458e-10, 5.357e-10, -0.0098, -1.44e-10, 5.298e-10,
      -0.009807, -1.399e-10, 5.375e-10, -0.009807, -1.381e-10, 5.316e-10;

  return {A, b};
}

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateLcpProblem(std::size_t n)
{
  math::MatrixX<S> A;
  math::VectorX<S> b;
  A = math::MatrixX<S>::Random(n, n);
  A = A * A.transpose();
  b = math::VectorX<S>::Random(n);

  return {A, b};
}

//==============================================================================
template <typename S>
std::pair<math::MatrixX<S>, math::VectorX<S>> generateValidLcpProblem(
    std::size_t n)
{
  math::MatrixX<S> A;
  math::VectorX<S> b;
  A = math::MatrixX<S>::Random(n, n);
  A = A * A.transpose();

  const math::VectorX<int> vec = math::Uniform<math::VectorXi>(n, 0, 1);

  math::VectorX<S> x = math::Uniform<math::VectorX<S>>(n, 0, 1);
  math::VectorX<S> y = math::Uniform<math::VectorX<S>>(n, 0, 1);

  for (auto i = 0u; i < n; ++i) {
    if (vec[i] == 0) {
      x[i] = S(0);
    } else {
      y[i] = S(0);
    }
  }

  b = y - A * x;

  return {A, b};
}

} // namespace dart::test
