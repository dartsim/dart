/*
 * Copyright (c) 2011-2022, The DART development contributors:
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

#include "dart/math/lie_group/function.hpp"

namespace dart::math {

//==============================================================================
template <typename DerivedA, typename DerivedB>
SE3Tangent<typename DerivedB::Scalar> Ad(
    const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V)
{
  using Scalar = typename DerivedB::Scalar;
  return SE3<Scalar>::Ad(T, V);
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
SE3Tangent<typename DerivedB::Scalar> Ad_R(
    const SE3Base<DerivedA>& T, const SE3TangentBase<DerivedB>& V)
{
  using Scalar = typename DerivedB::Scalar;
  return SE3<Scalar>::Ad_R(T, V);
}

//==============================================================================
template <typename DerivedA, typename DerivedB>
DerivedA ad(
    const SE3TangentBase<DerivedA>& s1, const SE3TangentBase<DerivedB>& s2)
{
  return s1.ad(s2);
}

} // namespace dart::math
