/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COMMON_REQUIRESASPECT_HPP_
#define DART_COMMON_REQUIRESASPECT_HPP_

#include "dart/common/SpecializedForAspect.hpp"

namespace dart {
namespace common {

//==============================================================================
/// RequiresAspect allows classes that inherit Composite to know which Aspects
/// are required for their operation. This guarantees that there is no way for
/// a required Aspect do not get unexpectedly removed from their composite.
///
/// Required Aspects are also automatically specialized for.
template <class... OtherRequiredAspects>
class RequiresAspect { };

//==============================================================================
template <class ReqAspect>
class RequiresAspect<ReqAspect> : public virtual SpecializedForAspect<ReqAspect>
{
public:

  /// Default constructor. This is where the base Composite is informed that
  /// the Aspect type is required.
  RequiresAspect();

};

//==============================================================================
template <class ReqAspect1, class... OtherReqAspects>
class RequiresAspect<ReqAspect1, OtherReqAspects...> :
    public CompositeJoiner< Virtual< RequiresAspect<ReqAspect1> >,
                               Virtual< RequiresAspect<OtherReqAspects...> > > { };

} // namespace common
} // namespace dart

#include "dart/common/detail/RequiresAspect.hpp"

#endif // DART_COMMON_REQUIRESASPECT_HPP_
