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

#pragma once

//==============================================================================
// Common Preprocessor Macros
//==============================================================================

/// Concatenate two tokens
/// Implementation detail for DART_EXPERIMENTAL_CONCAT
#define DART_EXPERIMENTAL_CONCAT_IMPL(a, b) a##b

/// Concatenate two tokens with macro expansion
/// Example: DART_EXPERIMENTAL_CONCAT(foo, __LINE__) -> foo123
#define DART_EXPERIMENTAL_CONCAT(a, b) DART_EXPERIMENTAL_CONCAT_IMPL(a, b)

/// Generate a unique identifier with the given prefix
/// Uses __COUNTER__ to ensure uniqueness across translation units
/// Example: DART_EXPERIMENTAL_UNIQUE_NAME(my_var) -> my_var0, my_var1, my_var2,
/// ...
#define DART_EXPERIMENTAL_UNIQUE_NAME(prefix)                                  \
  DART_EXPERIMENTAL_CONCAT(prefix, __COUNTER__)
