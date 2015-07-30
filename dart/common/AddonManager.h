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

#ifndef DART_COMMON_ADDONMANAGER_H_
#define DART_COMMON_ADDONMANAGER_H_

#include <map>
#include <typeinfo>
#include <typeindex>

#include "dart/common/Addon.h"

namespace dart {
namespace common {

class AddonManager
{
public:

  /// Virtual destructor
  virtual ~AddonManager() = default;

  /// Check if this AddonManager has a certain type of Addon
  template <class T>
  bool has() const;

  /// Get a certain type of Addon from this AddonManager
  template <class T>
  T* get();

  /// Get a certain type of Addon from this AddonManager
  template <class T>
  const T* get() const;

  /// Make a clone of the addon and place the clone into this AddonManager. If
  /// an Addon of the same type already exists in this AddonManager, the
  /// existing Addon will be destroyed.
  template <class T>
  void set(const std::unique_ptr<T>& addon);

  /// Use move semantics to place addon into this AddonManager. If an Addon of
  /// the same type already exists in this AddonManager, the existing Addon will
  /// be destroyed.
  template <class T>
  void set(std::unique_ptr<T>&& addon);

  /// Construct an Addon inside of this AddonManager
  template <class T, typename ...Args>
  T* construct(Args&&... args);

  /// Remove an Addon from this AddonManager.
  ///
  /// Note that this does not remove the entry from the AddonManager, but it
  /// sets the entry to a nullptr.
  template <class T>
  void erase();

protected:

  typedef std::map< std::type_index, std::unique_ptr<Addon> > AddonMap;

  /// A map that relates the type of Addon to its pointer
  AddonMap mAddonMap;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/AddonManager.h"

#endif // DART_COMMON_ADDONMANAGER_H_
