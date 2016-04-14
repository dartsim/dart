/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COMMON_DETAIL_ASPECT_H_
#define DART_COMMON_DETAIL_ASPECT_H_

#include <cassert>

#include "dart/common/Console.h"
#include "dart/common/Aspect.h"

namespace dart {
namespace common {

//==============================================================================
template <class CompositeType>
CompositeTrackingAspect<CompositeType>::CompositeTrackingAspect(Composite* comp)
  : Aspect(comp),
    mComposite(nullptr) // This will be set later when the Composite calls setComposite
{
  // Do nothing
}

//==============================================================================
template <class CompositeType>
CompositeType* CompositeTrackingAspect<CompositeType>::getComposite()
{
  return mComposite;
}

//==============================================================================
template <class CompositeType>
const CompositeType* CompositeTrackingAspect<CompositeType>::getComposite() const
{
  return mComposite;
}

//==============================================================================
template <class CompositeType>
void CompositeTrackingAspect<CompositeType>::setComposite(Composite* newComposite)
{
  mComposite = dynamic_cast<CompositeType*>(newComposite);
  if(nullptr == mComposite)
  {
    dterr << "[" << typeid(*this).name() << "::setComposite] Attempting to use a "
          << "[" << typeid(*newComposite).name() << "] type composite, but this "
          << "Aspect is only designed to be attached to a ["
          << typeid(CompositeType).name() << "] type composite. This may cause "
          << "undefined behavior!\n";
    assert(false);
  }
}

//==============================================================================
template <class CompositeType>
void CompositeTrackingAspect<CompositeType>::loseComposite(Composite* /*oldComposite*/)
{
  mComposite = nullptr;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ASPECT_H_
