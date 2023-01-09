/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#pragma once

// Note: It is important to note that DART should not be installed with profile enabled.

// DART_ENABLE_PROFILE is defined in dart/CMakeLists.txt
#if DART_PROFILE_BACKEND_REMOTERY
  #define DART_ENABLE_PROFILE 1

  // If DART_ENABLE_PROFILE is defined, then we define macros for profiling
  #include <Remotery.h>
  #define DART_PROFILE_STARTUP(name)                                           \
    static Remotery* name = nullptr;                                           \
    rmt_CreateGlobalInstance(&name)
  #define DART_PROFILE_SHUTDOWN(name)                                          \
    if (name)                                                                  \
    rmt_DestroyGlobalInstance(name)
  #define DART_PROFILE_SCOPED(name) rmt_ScopedCPUSample(name, 0)
  #define DART_PROFILE_PUSH(name) rmt_BeginCPUSample(name, 0)
  #define DART_PROFILE_POP() rmt_EndCPUSample()

#else

  // If DART_ENABLE_PROFILE is not defined, then we define empty macros
  #define DART_PROFILE_STARTUP(name)
  #define DART_PROFILE_SHUTDOWN(name)
  #define DART_PROFILE_SCOPED(name)
  #define DART_PROFILE_PUSH(name)
  #define DART_PROFILE_POP()

#endif
