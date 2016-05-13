/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_COMPOSITEJOINER_HPP_
#define DART_COMMON_COMPOSITEJOINER_HPP_

#include "dart/common/Composite.hpp"
#include "dart/common/Empty.hpp"

namespace dart {
namespace common {

/// Terminator for the variadic template
template <class... OtherBases>
class CompositeJoiner
{
public:
  virtual ~CompositeJoiner() = default;
};

/// Special case of only having 1 class: we do nothing but inherit it.
template <class Base1>
class CompositeJoiner<Base1> : public Base1
{
public:
  virtual ~CompositeJoiner() = default;
};

/// CompositeJoiner allows classes that inherit from various
/// SpecializedForAspect types to be inherited by a single derived class.
/// This class solves the diamond-of-death problem for multiple
/// SpecializedForAspect inheritance.
template <class Base1, class Base2>
class CompositeJoiner<Base1, Base2> : public Base1, public Base2
{
public:

  /// Default constructor
  CompositeJoiner() = default;

  virtual ~CompositeJoiner() = default;

  /// This constructor allows one argument to be passed to the Base1 constructor
  /// and arbitrarily many arguments to be passed to the Base2 constructor.
  //
  // TODO(MXG): When we migrate to using C++14, we can use std::tuple and
  // std::index_sequence (which is only available in C++14) to dispatch
  // arbitrarily many arguments to the constructors of each base class. Until
  // then, this is the best we can offer due to fundamental limitations of
  // variadic templates in C++11.
  template <typename Base1Arg, typename... Base2Args>
  CompositeJoiner(Base1Arg&& arg1, Base2Args&&... args2);

  /// This constructor passes one argument to the Base1 constructor and no
  /// arguments to the Base2 constructor.
  template <typename Base1Arg>
  CompositeJoiner(Base1Arg&& arg1, NoArgTag);

  /// This constructor passes no arguments to the Base1 constructor and
  /// arbitrarily many arguments to the Base2 constructor.
  template <typename... Base2Args>
  CompositeJoiner(NoArgTag, Base2Args&&... args2);

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
  void set(const T* aspect);

  // Documentation inherited
  template <class T>
  void set(std::unique_ptr<T>&& aspect);

  // Documentation inherited
  template <class T, typename ...Args>
  T* createAspect(Args&&... args);

  // Documentation inherited
  template <class T>
  void removeAspect();

  // Documentation inherited
  template <class T>
  std::unique_ptr<T> releaseAspect();

  // Documentation inherited
  template <class T>
  static constexpr bool isSpecializedFor();

};

/// This is the variadic version of the CompositeJoiner class which allows
/// you to include arbitrarily many base classes in the joining.
template <class Base1, class Base2, class... OtherBases>
class CompositeJoiner<Base1, Base2, OtherBases...> :
    public CompositeJoiner< Base1, CompositeJoiner<Base2, OtherBases...> >
{
public:

  /// Default constructor
  CompositeJoiner() = default;

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
  CompositeJoiner(Args&&... args);

  virtual ~CompositeJoiner() = default;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/CompositeJoiner.hpp"

#endif // DART_COMMON_COMPOSITEJOINER_HPP_

