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

#include <dart/simulation/detail/deformable_contact/primitive_distance.hpp>
#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>

namespace dart::simulation::detail::deformable_contact {

using newton_barrier::BarrierScalarDerivatives;
using newton_barrier::c2ClampedLogBarrier;
using newton_barrier::edgeEdgeBarrier;
using newton_barrier::mollifiedEdgeEdgeBarrier;
using newton_barrier::pointEdgeBarrier;
using newton_barrier::pointPointBarrier;
using newton_barrier::pointTriangleBarrier;
using newton_barrier::PrimitiveBarrierResult;

namespace detail {

using newton_barrier::detail::kBarrierDistanceFloorScale;
using newton_barrier::detail::makeInactivePrimitiveBarrier;
using newton_barrier::detail::nonnegativeFiniteStiffness;
using newton_barrier::detail::safeSquaredBarrierDistance;

} // namespace detail
} // namespace dart::simulation::detail::deformable_contact
