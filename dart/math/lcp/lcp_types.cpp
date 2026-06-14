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

#include "dart/math/lcp/lcp_types.hpp"

namespace dart {
namespace math {

//==============================================================================
std::string toString(LcpSolverStatus status)
{
  switch (status) {
    using enum LcpSolverStatus;
    case Success:
      return "Success";
    case Failed:
      return "Failed";
    case MaxIterations:
      return "MaxIterations";
    case NumericalError:
      return "NumericalError";
    case InvalidProblem:
      return "InvalidProblem";
    case Degenerate:
      return "Degenerate";
    case NotSolved:
      return "NotSolved";
    default:
      return "Unknown";
  }
}

//==============================================================================
std::string toString(LcpProblemType type)
{
  switch (type) {
    using enum LcpProblemType;
    case Invalid:
      return "Invalid";
    case Standard:
      return "Standard";
    case Boxed:
      return "Boxed";
    case FrictionIndex:
      return "FrictionIndex";
    default:
      return "Unknown";
  }
}

} // namespace math
} // namespace dart
