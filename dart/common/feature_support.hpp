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

#ifndef DART_COMMON_FEATURE_SUPPORT_HPP_
#define DART_COMMON_FEATURE_SUPPORT_HPP_

// Centralized C++23 (and selected core-language) feature detection for DART.
//
// Each DART_HAS_* macro expands to 1 when the feature is available on the
// current toolchain, otherwise 0, so call sites can write:
//
//   #if DART_HAS_STD_FLAT_MAP
//     // std::flat_map fast path
//   #else
//     // std::map fallback
//   #endif
//
// Background, per-feature portability, and the staged adoption plan live in
// docs/design/cpp23_modernization.md. In short:
//
//   - "Adopt now" features (expected, optional monadic ops, to_underlying,
//     unreachable, deducing this, multidimensional subscript, ranges zip/to)
//     are guaranteed on DART's pinned compiler floor and may be used
//     unconditionally once the project compiles as C++23.
//   - "Guard" features (print, flat_map/flat_set, move_only_function, ranges
//     enumerate) may be absent on some supported standard libraries -- libc++,
//     DART's macOS floor, does not implement std::views::enumerate -- and MUST
//     be used behind the matching macro with a fallback.
//   - "Deferred" features (generator, ranges chunk/slide, mdspan) are missing
//     from libc++ and/or the GCC 15 libstdc++ floor; do not use them yet.

#include <version> // standard library feature-test macros (__cpp_lib_*)

// ===========================================================================
// Core-language features
// ===========================================================================

#if defined(__cpp_explicit_this_parameter)                                     \
    && __cpp_explicit_this_parameter >= 202110L
  #define DART_HAS_DEDUCING_THIS 1
#else
  #define DART_HAS_DEDUCING_THIS 0
#endif

#if defined(__cpp_multidimensional_subscript)                                  \
    && __cpp_multidimensional_subscript >= 202110L
  #define DART_HAS_MULTIDIM_SUBSCRIPT 1
#else
  #define DART_HAS_MULTIDIM_SUBSCRIPT 0
#endif

#if defined(__cpp_static_call_operator) && __cpp_static_call_operator >= 202207L
  #define DART_HAS_STATIC_CALL_OPERATOR 1
#else
  #define DART_HAS_STATIC_CALL_OPERATOR 0
#endif

#if defined(__cpp_if_consteval) && __cpp_if_consteval >= 202106L
  #define DART_HAS_IF_CONSTEVAL 1
#else
  #define DART_HAS_IF_CONSTEVAL 0
#endif

// ===========================================================================
// "Adopt now" library features (present on DART's pinned compiler floor)
// ===========================================================================

#if defined(__cpp_lib_expected) && __cpp_lib_expected >= 202202L
  #define DART_HAS_STD_EXPECTED 1
#else
  #define DART_HAS_STD_EXPECTED 0
#endif

// Monadic and_then/transform/or_else on std::optional (P0798).
#if defined(__cpp_lib_optional) && __cpp_lib_optional >= 202110L
  #define DART_HAS_STD_OPTIONAL_MONADIC 1
#else
  #define DART_HAS_STD_OPTIONAL_MONADIC 0
#endif

#if defined(__cpp_lib_to_underlying) && __cpp_lib_to_underlying >= 202102L
  #define DART_HAS_STD_TO_UNDERLYING 1
#else
  #define DART_HAS_STD_TO_UNDERLYING 0
#endif

#if defined(__cpp_lib_unreachable) && __cpp_lib_unreachable >= 202202L
  #define DART_HAS_STD_UNREACHABLE 1
#else
  #define DART_HAS_STD_UNREACHABLE 0
#endif

#if defined(__cpp_lib_ranges_zip) && __cpp_lib_ranges_zip >= 202110L
  #define DART_HAS_STD_RANGES_ZIP 1
#else
  #define DART_HAS_STD_RANGES_ZIP 0
#endif

#if defined(__cpp_lib_ranges_enumerate) && __cpp_lib_ranges_enumerate >= 202302L
  #define DART_HAS_STD_RANGES_ENUMERATE 1
#else
  #define DART_HAS_STD_RANGES_ENUMERATE 0
#endif

#if defined(__cpp_lib_ranges_to_container)                                     \
    && __cpp_lib_ranges_to_container >= 202202L
  #define DART_HAS_STD_RANGES_TO 1
#else
  #define DART_HAS_STD_RANGES_TO 0
#endif

// ===========================================================================
// "Guard" library features (may be absent; always provide a fallback)
// ===========================================================================

#if defined(__cpp_lib_print) && __cpp_lib_print >= 202207L
  #define DART_HAS_STD_PRINT 1
#else
  #define DART_HAS_STD_PRINT 0
#endif

#if defined(__cpp_lib_flat_map) && __cpp_lib_flat_map >= 202207L
  #define DART_HAS_STD_FLAT_MAP 1
#else
  #define DART_HAS_STD_FLAT_MAP 0
#endif

#if defined(__cpp_lib_flat_set) && __cpp_lib_flat_set >= 202207L
  #define DART_HAS_STD_FLAT_SET 1
#else
  #define DART_HAS_STD_FLAT_SET 0
#endif

#if defined(__cpp_lib_move_only_function)                                      \
    && __cpp_lib_move_only_function >= 202110L
  #define DART_HAS_STD_MOVE_ONLY_FUNCTION 1
#else
  #define DART_HAS_STD_MOVE_ONLY_FUNCTION 0
#endif

// ===========================================================================
// "Deferred" library features (typically absent on libc++ / GCC 15 libstdc++)
// ===========================================================================

#if defined(__cpp_lib_generator) && __cpp_lib_generator >= 202207L
  #define DART_HAS_STD_GENERATOR 1
#else
  #define DART_HAS_STD_GENERATOR 0
#endif

#if defined(__cpp_lib_ranges_chunk) && __cpp_lib_ranges_chunk >= 202202L
  #define DART_HAS_STD_RANGES_CHUNK 1
#else
  #define DART_HAS_STD_RANGES_CHUNK 0
#endif

#if defined(__cpp_lib_ranges_slide) && __cpp_lib_ranges_slide >= 202202L
  #define DART_HAS_STD_RANGES_SLIDE 1
#else
  #define DART_HAS_STD_RANGES_SLIDE 0
#endif

#if defined(__cpp_lib_mdspan) && __cpp_lib_mdspan >= 202207L
  #define DART_HAS_STD_MDSPAN 1
#else
  #define DART_HAS_STD_MDSPAN 0
#endif

#endif // DART_COMMON_FEATURE_SUPPORT_HPP_
