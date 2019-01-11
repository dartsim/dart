/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_SINGLETON_HPP_
#define DART_COMMON_SINGLETON_HPP_

namespace dart {
namespace common {

/// Singleton template class
///
/// \note This singleton is not thread safe. For use of thread safe singleton,
/// use static initialization as:
///
/// // Singletone class Engine
/// class Engine : public Singleton<Engine> {};
///
/// // Call before main() and use theT only instead of calling getSingleton()
/// static T& theT = T::getSingleton();
template<typename T>
class Singleton
{
public:
  /// Returns reference of the singleton
  template <typename... Args>
  static T& getSingleton(Args... _args);

  /// Returns pointer of the singleton
  template <typename ... Args>
  static T* getSingletonPtr(Args... _args);

protected:
  /// Constructor
  Singleton() = default;

  /// Destructor
  virtual ~Singleton() = default;

private:
  /// Don't implement copy constructor
  Singleton(const T&) = delete;

  /// Don't assignment operator
  const T& operator=(const T&) = delete;

private:
  /// Singleton instance
  static T* mInstance;
};

} // namespace common
} // namespace dart

#include "dart/common/detail/Singleton-impl.hpp"

#endif // DART_COMMON_SINGLETON_HPP_
