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

#ifndef DART_COMMON_ADDONMANAGERJOINER_H_
#define DART_COMMON_ADDONMANAGERJOINER_H_

#include "dart/common/AddonManager.h"
#include "dart/common/Empty.h"

namespace dart {
namespace common {

/// Terminator for the variadic template
template <class... OtherBases>
class AddonManagerJoiner { };

/// Special case of only having 1 class: we do nothing but inherit it.
template <class Base1>
class AddonManagerJoiner<Base1> : public Base1 { };

/// AddonManagerJoiner allows classes that inherit from various
/// SpecializedAddonManager types to be inherited by a single derived class.
/// This class solves the diamond-of-death problem for multiple
/// SpecializedAddonManager inheritance.
template <class Base1, class Base2>
class AddonManagerJoiner<Base1, Base2> : public Base1, public Base2
{
public:

  /// Default constructor
  AddonManagerJoiner() = default;

  /// This constructor allows one argument to be passed to the Base1 constructor
  /// and arbitrarily many arguments to be passed to the Base2 constructor.
  //
  // TODO(MXG): When we migrate to using C++14, we can use std::tuple and
  // std::index_sequence (which is only available in C++14) to dispatch
  // arbitrarily many arguments to the constructors of each base class. Until
  // then, this is the best we can offer due to fundamental limitations of
  // variadic templates in C++11.
  template <typename Base1Arg, typename... Base2Args>
  AddonManagerJoiner(Base1Arg&& arg1, Base2Args&&... args2);

  /// This constructor passes one argument to the Base1 constructor and no
  /// arguments to the Base2 constructor.
  template <typename Base1Arg>
  AddonManagerJoiner(Base1Arg&& arg1, NoArg_t);

  /// This constructor passes no arguments to the Base1 constructor and
  /// arbitrarily many arguments to the Base2 constructor.
  template <typename... Base2Args>
  AddonManagerJoiner(NoArg_t, Base2Args&&... args2);

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

/// This is the variadic version of the AddonManagerJoiner class which allows
/// you to include arbitrarily many base classes in the joining.
template <class Base1, class Base2, class... OtherBases>
class AddonManagerJoiner<Base1, Base2, OtherBases...> :
    public AddonManagerJoiner< Base1, AddonManagerJoiner<Base2, OtherBases...> >
{
public:

  /// Default constructor
  AddonManagerJoiner() = default;

  /// This constructor allows one argument to be passed to each Base class's
  /// constructor (except the final Base class, which accepts arbitrarily many
  /// arguments). Pass in dart::common::NoArgs for classes whose constructors
  /// do not take any arguments.
  //
  // TODO(MXG): When we migrate to using C++14, we can use std::tuple and
  // std::index_sequence (which is only available in C++14) to dispatch
  // arbitrarily many arguments to the constructors of each base class. Until
  // then, this is the best we can offer due to fundamental limitations of
  // variadic templates in C++11.
  template <typename... Args>
  AddonManagerJoiner(Args&&... args);

};

} // namespace common
} // namespace dart

#include "dart/common/detail/AddonManagerJoiner.h"

#endif // DART_COMMON_ADDONMANAGERJOINER_H_

