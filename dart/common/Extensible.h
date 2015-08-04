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

#ifndef DART_COMMON_EXTENSIBLE_H_
#define DART_COMMON_EXTENSIBLE_H_

#include <memory>

namespace dart {
namespace common {

/// Extensible is a CRTP base class that provides an interface for easily
/// creating data structures that are meant to be extended. Ordinary copying
/// does not work with extended data structures, because information from the
/// derived classes will typically be lost during a copy. The Extensible class
/// provides an interface for creating a new copy of an extended data structure,
/// as well as copying information between two extended data structures of the
/// same type.
template <class T>
class Extensible
{
public:

  /// Default constructor
  Extensible() = default;

  /// Virtual destructor
  virtual ~Extensible() = default;

  /// Do not copy this class directly, use clone() or copy() instead
  Extensible(const Extensible& doNotCopy) = delete;

  /// Do not copy this class directly, use clone() or copy() instead
  Extensible& operator=(const Extensible& doNotCopy) = delete;

  /// Implement this function to allow your Extensible type to be copied safely.
  virtual std::unique_ptr<T> clone() const = 0;

  /// Copy the contents of anotherExtensible into this one
  virtual void copy(const T& anotherExtensible) = 0;
};

} // namespace common
} // namespace dart

#endif // DART_COMMON_EXTENSIBLE_H_
