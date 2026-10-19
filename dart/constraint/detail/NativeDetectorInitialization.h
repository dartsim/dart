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
 *   * Neither the name of the Georgia Institute of Technology nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_CONSTRAINT_DETAIL_NATIVEDETECTORINITIALIZATION_H_
#define DART_CONSTRAINT_DETAIL_NATIVEDETECTORINITIALIZATION_H_

// Private to DART's sources; the .h extension keeps it out of installs.

namespace dart {
namespace collision {
class CollisionDetector;
} // namespace collision
namespace constraint {
class ConstraintSolver;
namespace detail {

// Preserve only a pristine native World certificate while its detector is
// installed. Ownership queries remain live until the complete setter returns.
class NativeDetectorInitialization
{
public:
  NativeDetectorInitialization(
      const ConstraintSolver* solver,
      const collision::CollisionDetector* incoming);
  ~NativeDetectorInitialization();

  NativeDetectorInitialization(const NativeDetectorInitialization&) = delete;
  NativeDetectorInitialization& operator=(const NativeDetectorInitialization&)
      = delete;

private:
  const ConstraintSolver* mSolver;
  int mUncaughtExceptions;
};

// Internal invariant observation: later membership mutations would otherwise
// mask an initialization scope incorrectly resurrecting a retired certificate.
bool hasNativeOwnershipCertificate(const ConstraintSolver* solver);

} // namespace detail
} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_DETAIL_NATIVEDETECTORINITIALIZATION_H_
