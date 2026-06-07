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

#include <dart/simulation/detail/newton_barrier/primitive_distance.hpp>

namespace dart::simulation::detail::deformable_contact {

using newton_barrier::edgeEdgeCrossSquaredNorm;
using newton_barrier::edgeEdgeCrossSquaredNormGradient;
using newton_barrier::edgeEdgeCrossSquaredNormHessian;
using newton_barrier::EdgeEdgeDistance;
using newton_barrier::EdgeEdgeFeature;
using newton_barrier::edgeEdgeMollifier;
using newton_barrier::edgeEdgeMollifierGradient;
using newton_barrier::edgeEdgeMollifierHessian;
using newton_barrier::edgeEdgeMollifierThreshold;
using newton_barrier::edgeEdgeSquaredDistance;
using newton_barrier::edgeEdgeSquaredDistanceGradient;
using newton_barrier::edgeEdgeSquaredDistanceHessian;
using newton_barrier::Matrix12d;
using newton_barrier::Matrix6d;
using newton_barrier::Matrix9d;
using newton_barrier::PointEdgeDistance;
using newton_barrier::PointEdgeFeature;
using newton_barrier::pointEdgeSquaredDistance;
using newton_barrier::pointEdgeSquaredDistanceGradient;
using newton_barrier::pointEdgeSquaredDistanceHessian;
using newton_barrier::pointPointSquaredDistance;
using newton_barrier::pointPointSquaredDistanceGradient;
using newton_barrier::pointPointSquaredDistanceHessian;
using newton_barrier::PointTriangleDistance;
using newton_barrier::PointTriangleFeature;
using newton_barrier::pointTriangleSquaredDistance;
using newton_barrier::pointTriangleSquaredDistanceGradient;
using newton_barrier::pointTriangleSquaredDistanceHessian;
using newton_barrier::Vector12d;
using newton_barrier::Vector6d;
using newton_barrier::Vector9d;

namespace detail {

using newton_barrier::detail::crossSquaredNormGradientFromLinearVectors;
using newton_barrier::detail::crossSquaredNormHessianFromEdges;
using newton_barrier::detail::crossSquaredNormHessianFromLinearVectors;
using newton_barrier::detail::isDegenerateSegment;
using newton_barrier::detail::isDegenerateTriangle;
using newton_barrier::detail::isParallelEdges;
using newton_barrier::detail::kMollifierThresholdScale;
using newton_barrier::detail::kRelativeEpsilon;
using newton_barrier::detail::pointPointHessianMapped;
using newton_barrier::detail::ratioHessian;
using newton_barrier::detail::scatterPointPointHessian;
using newton_barrier::detail::scatterThreeBlockHessian;
using newton_barrier::detail::skew;
using newton_barrier::detail::squaredNormGradientFromLinearVector;
using newton_barrier::detail::squaredNormHessianFromLinearVector;
using newton_barrier::detail::squaredRatioHessian;
using newton_barrier::detail::tripleProductGradientFromLinearVectors;
using newton_barrier::detail::tripleProductHessianFromLinearVectors;

} // namespace detail
} // namespace dart::simulation::detail::deformable_contact
