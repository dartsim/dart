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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/experimental/detail/deformable_contact/primitive_distance.hpp>
#include <dart/simulation/experimental/detail/newton_barrier/tangent_stencil.hpp>

namespace dart::simulation::experimental::detail::deformable_contact {

using newton_barrier::EdgeEdgeTangentStencil;
using newton_barrier::edgeEdgeTangentStencil;
using newton_barrier::liftTangentialDisplacement;
using newton_barrier::Matrix2x12d;
using newton_barrier::Matrix2x6d;
using newton_barrier::Matrix2x9d;
using newton_barrier::Matrix3x2d;
using newton_barrier::PointEdgeTangentStencil;
using newton_barrier::pointEdgeTangentStencil;
using newton_barrier::PointPointTangentStencil;
using newton_barrier::pointPointTangentStencil;
using newton_barrier::PointTriangleTangentStencil;
using newton_barrier::pointTriangleTangentStencil;
using newton_barrier::projectTangentialDisplacement;

namespace detail {

using newton_barrier::detail::basisFromFirstTangentAndSecondHint;
using newton_barrier::detail::edgeEdgeCoordinates;
using newton_barrier::detail::fallbackBasisFromNormal;
using newton_barrier::detail::isFinite;
using newton_barrier::detail::kTangentBasisEpsilon;
using newton_barrier::detail::normalized;
using newton_barrier::detail::pointEdgeCoordinate;
using newton_barrier::detail::pointTriangleCoordinates;
using newton_barrier::detail::projectionFromCoefficients;

} // namespace detail
} // namespace dart::simulation::experimental::detail::deformable_contact
