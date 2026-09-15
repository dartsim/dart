/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_COMMON_VIRTUAL_HPP_
#define DART_COMMON_VIRTUAL_HPP_

#include <dart/common/ClassWithVirtualBase.hpp>

namespace dart {
namespace common {

/// This class is used to have CRTP functions inherit their template parameters
/// virtually instead of directly.
///
/// alignas(T): keep the non-virtual part as aligned as the virtual base so that
/// derived classes never place it under-aligned (see #3447 for Frame). The
/// alignof(void*) floor keeps instantiations whose T is aligned below the
/// vtable pointer well-formed; it must stay a single alignas specifier because
/// GCC does not combine two of them for the non-virtual part.
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN
template <class T>
class alignas(alignof(T) > alignof(void*) ? alignof(T) : alignof(void*)) Virtual
  : public virtual T
{
public:
  virtual ~Virtual() = default;
};
DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_END

} // namespace common
} // namespace dart

#endif // DART_COMMON_VIRTUAL_HPP_
