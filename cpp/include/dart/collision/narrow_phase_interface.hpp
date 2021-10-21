/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/collision/collision_option.hpp"
#include "dart/collision/export.hpp"
#include "dart/collision/object.hpp"

namespace dart::collision {

template <typename Scalar>
bool collide(
    ObjectPtr<Scalar> object1,
    ObjectPtr<Scalar> object2,
    const CollisionOption<Scalar>& option = {},
    CollisionResult<Scalar>* result = nullptr);

#if DART_BUILD_TEMPLATE_CODE_FOR_DOUBLE
extern template DART_COLLISION_API bool collide(
    ObjectPtr<double> object1,
    ObjectPtr<double> object2,
    const CollisionOption<double>& option,
    CollisionResult<double>* result);
#endif

#if DART_BUILD_TEMPLATE_CODE_FOR_FLOAT
extern template DART_COLLISION_API bool collide(
    ObjectPtr<float> object1,
    ObjectPtr<float> object2,
    const CollisionOption<float>& option,
    CollisionResult<float>* result);
#endif

} // namespace dart::collision

#include "dart/collision/detail/narrow_phase_interface_impl.hpp"
