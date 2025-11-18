/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_OPTIMIZER_IPOPT_BACKWARDCOMPATIBILITY_HPP_
#define DART_OPTIMIZER_IPOPT_BACKWARDCOMPATIBILITY_HPP_

#ifndef __has_include
  #define __has_include(x) 0
#endif

#if __has_include(<IpoptConfig.h>)
  #include <IpoptConfig.h>
#elif __has_include(<coin/IpoptConfig.h>)
  #include <coin/IpoptConfig.h>
#else
  #error "IpoptConfig.h not found. Please install Ipopt headers."
#endif

#ifndef IPOPT_VERSION_PATCH
  #ifdef IPOPT_VERSION_RELEASE
    #define IPOPT_VERSION_PATCH IPOPT_VERSION_RELEASE
  #else
    #define IPOPT_VERSION_PATCH 0
  #endif
#endif

#if __has_include(<IpIpoptApplication.hpp>) && __has_include(<IpTNLP.hpp>)
  #define DART_IPOPT_USE_COIN_NAMESPACE 0
  #include <IpIpoptApplication.hpp>
  #include <IpTNLP.hpp>
#elif __has_include(<coin/IpIpoptApplication.hpp>) && __has_include(<coin/IpTNLP.hpp>)
  #define DART_IPOPT_USE_COIN_NAMESPACE 1
  #include <coin/IpIpoptApplication.hpp>
  #include <coin/IpTNLP.hpp>
#else
  #error "Failed to locate Ipopt headers. Ensure Ipopt is installed."
#endif

#define IPOPT_VERSION_GE(x, y, z)                                              \
  (IPOPT_VERSION_MAJOR > x                                                     \
   || (IPOPT_VERSION_MAJOR == x                                                \
       && (IPOPT_VERSION_MINOR > y                                             \
           || (IPOPT_VERSION_MINOR == y && IPOPT_VERSION_PATCH >= z))))

#endif // DART_OPTIMIZER_IPOPT_BACKWARDCOMPATIBILITY_HPP_
