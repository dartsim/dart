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

#include <dart/math/math.hpp>

#include <dart/common/common.hpp>

#include <fmt/format.h>

namespace dart::python {

using namespace math;

template <typename Derived>
std::string toString(const Eigen::MatrixBase<Derived>& mat)
{
  std::string out;

  // Compute the maximum width of the elements
  int max_width = 0;
  const int precision = 8;
  for (auto i : common::Range(mat.rows())) {
    for (auto j : common::Range(mat.cols())) {
      const auto val = mat(i, j);
      max_width = std::max<int>(
          max_width, fmt::format("{:.{}}", val, precision).size());
    }
  }

  // Format each element with the maximum width
  if (mat.rows() == 1) {
    out += "[";
    for (auto j : common::Range(mat.cols())) {
      const auto val = mat(0, j);
      out += fmt::format(
          "{:>{}} ", fmt::format("{:.{}}", val, precision), max_width);
      if (j != mat.cols() - 1) {
        out += " ";
      }
    }
    out += "]\n";
  } else {
    for (auto i : common::Range(mat.rows())) {
      if (i == 0) {
        out += "[[";
      } else {
        out += " [";
      }
      for (auto j : common::Range(mat.cols())) {
        const auto val = mat(i, j);
        out += fmt::format(
            "{:>{}} ", fmt::format("{:.{}}", val, precision), max_width);
        if (j != mat.cols() - 1) {
          out += " ";
        }
      }
      if (i != mat.rows() - 1) {
        out += "]\n";
      } else {
        out += "]]\n";
      }
    }
  }

  return out;
}

template <typename Derived>
std::string toRepr(
    const std::string& name, const Eigen::MatrixBase<Derived>& mat)
{
  std::string out;

  // Compute the maximum width of the elements
  int max_width = 0;
  const int precision = 8;
  for (auto i : common::Range(mat.rows())) {
    for (auto j : common::Range(mat.cols())) {
      const auto val = mat(i, j);
      max_width = std::max<int>(
          max_width, fmt::format("{:.{}}", val, precision).size());
    }
  }

  // Format each element with the maximum width
  if (mat.rows() == 1) {
    out += fmt::format("{}([", name);
    for (auto j : common::Range(mat.cols())) {
      const auto val = mat(0, j);
      out += fmt::format(
          "{:>{}} ", fmt::format("{:.{}}", val, precision), max_width);
      if (j != mat.cols() - 1) {
        out += " ";
      }
    }
    out += "])\n";
  } else {
    for (auto i : common::Range(mat.rows())) {
      if (i == 0) {
        out += fmt::format("{}([", name);
      } else {
        out += fmt::format("{: <{}}[", "", name.size() + 1u);
      }
      for (auto j : common::Range(mat.cols())) {
        const auto val = mat(i, j);
        out += fmt::format(
            "{:>{}} ", fmt::format("{:.{}}", val, precision), max_width);
        if (j != mat.cols() - 1) {
          out += " ";
        }
      }
      if (i != mat.rows() - 1) {
        out += "]\n";
      } else {
        out += "]]\n";
      }
    }
  }

  return out;
}

} // namespace dart::python
