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

#ifndef DART_COMMON_DETAIL_ADDON_HPP_
#define DART_COMMON_DETAIL_ADDON_HPP_

#include <cassert>

#include "dart/common/Console.hpp"
#include "dart/common/Addon.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class ManagerType>
ManagerTrackingAddon<ManagerType>::ManagerTrackingAddon(
    AddonManager* mgr)
  : Addon(mgr),
    mManager(nullptr) // This will be set later when the Manager calls setManager
{
  // Do nothing
}

//==============================================================================
template <class ManagerType>
ManagerType* ManagerTrackingAddon<ManagerType>::getManager()
{
  return mManager;
}

//==============================================================================
template <class ManagerType>
const ManagerType* ManagerTrackingAddon<ManagerType>::getManager() const
{
  return mManager;
}

//==============================================================================
template <class ManagerType>
void ManagerTrackingAddon<ManagerType>::setManager(
    AddonManager* newManager, bool)
{
  mManager = dynamic_cast<ManagerType*>(newManager);
  if(nullptr == mManager)
  {
    dterr << "[" << typeid(*this).name() << "::setManager] Attempting to use a "
          << "[" << typeid(newManager).name() << "] type manager, but this "
          << "Addon is only designed to be attached to a ["
          << typeid(ManagerType).name() << "] type manager. This may cause "
          << "undefined behavior!\n";
    assert(false);
  }
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ADDON_HPP_
