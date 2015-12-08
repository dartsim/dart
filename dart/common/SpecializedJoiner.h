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

#ifndef DART_COMMON_SPECIALIZEDJOINER_H_
#define DART_COMMON_SPECIALIZEDJOINER_H_

#include "dart/common/AddonManager.h"

namespace dart {
namespace common {

/// SpecializedJoiner allows classes that inherit from various
/// SpecializedManager types to be inherited by a single derived class. This
/// class solves the diamond-of-death problem for multiple SpecializedManager
/// inheritance.
template <class Base1, class Base2>
class SpecializedJoiner : public Base1, public Base2
{
public:

  // Documentation inherited
  template <class T>
  bool has() const;

  // Documentation inherited
  template <class T>
  T* get();

  // Documentation inherited
  template <class T>
  const T* get() const;

  // Documentation inherited
  template <class T>
  void set(const T* addon);

  // Documentation inherited
  template <class T>
  void set(std::unique_ptr<T>&& addon);

  // Documentation inherited
  template <class T, typename ...Args>
  T* create(Args&&... args);

  // Documentation inherited
  template <class T>
  void erase();

  // Documentation inherited
  template <class T>
  std::unique_ptr<T> release();

  // Documentation inherited
  template <class T>
  static constexpr bool isSpecializedFor();

};

/// This is the variadic version of the SpecializedJoiner class which allows
/// you to include arbitrarily many base classes in the joining.
template <class Base1, class... OtherBases>
class SpecializedJoiner<Base1, OtherBases...> :
    public SpecializedJoiner< Base1, SpecializedJoiner<OtherBases...> > { };

} // namespace common
} // namespace dart

#include "dart/common/detail/SpecializedJoiner.h"

#endif // DART_COMMON_SPECIALIZEDJOINER_H_

