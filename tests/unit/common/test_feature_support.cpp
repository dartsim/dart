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

#include <dart/common/feature_support.hpp>
#include <dart/common/macros.hpp>

#include <gtest/gtest.h>

#include <cstdint>

// ===========================================================================
// Compile-time smoke test of the feature-probe header.
//
// Two layers:
//   1. Every DART_HAS_* probe must expand to exactly 0 or 1 on every toolchain.
//      A typo (missing #else, wrong macro name) would otherwise expand to an
//      empty token or non-binary value and silently misroute every #if guard.
//   2. Once DART compiles as C++23 (its pinned floor), the "adopt now" features
//      MUST resolve to 1. If a CI runner's standard library unexpectedly lacks
//      one (a misconfigured toolchain, a stale image), this fails the build
//      loudly instead of letting call sites degrade to a silent fallback.
// ===========================================================================

// Layer 1: binary-valued probes. (X + 1 is ill-formed unless X is an integer
// literal, and the comparison pins it to {0, 1}.)
static_assert(
    DART_HAS_DEDUCING_THIS == 0 || DART_HAS_DEDUCING_THIS == 1,
    "DART_HAS_DEDUCING_THIS must be 0 or 1");
static_assert(
    DART_HAS_MULTIDIM_SUBSCRIPT == 0 || DART_HAS_MULTIDIM_SUBSCRIPT == 1,
    "DART_HAS_MULTIDIM_SUBSCRIPT must be 0 or 1");
static_assert(
    DART_HAS_STATIC_CALL_OPERATOR == 0 || DART_HAS_STATIC_CALL_OPERATOR == 1,
    "DART_HAS_STATIC_CALL_OPERATOR must be 0 or 1");
static_assert(
    DART_HAS_IF_CONSTEVAL == 0 || DART_HAS_IF_CONSTEVAL == 1,
    "DART_HAS_IF_CONSTEVAL must be 0 or 1");
static_assert(
    DART_HAS_STD_EXPECTED == 0 || DART_HAS_STD_EXPECTED == 1,
    "DART_HAS_STD_EXPECTED must be 0 or 1");
static_assert(
    DART_HAS_STD_OPTIONAL_MONADIC == 0 || DART_HAS_STD_OPTIONAL_MONADIC == 1,
    "DART_HAS_STD_OPTIONAL_MONADIC must be 0 or 1");
static_assert(
    DART_HAS_STD_TO_UNDERLYING == 0 || DART_HAS_STD_TO_UNDERLYING == 1,
    "DART_HAS_STD_TO_UNDERLYING must be 0 or 1");
static_assert(
    DART_HAS_STD_UNREACHABLE == 0 || DART_HAS_STD_UNREACHABLE == 1,
    "DART_HAS_STD_UNREACHABLE must be 0 or 1");
static_assert(
    DART_HAS_STD_RANGES_ZIP == 0 || DART_HAS_STD_RANGES_ZIP == 1,
    "DART_HAS_STD_RANGES_ZIP must be 0 or 1");
static_assert(
    DART_HAS_STD_RANGES_ENUMERATE == 0 || DART_HAS_STD_RANGES_ENUMERATE == 1,
    "DART_HAS_STD_RANGES_ENUMERATE must be 0 or 1");
static_assert(
    DART_HAS_STD_RANGES_TO == 0 || DART_HAS_STD_RANGES_TO == 1,
    "DART_HAS_STD_RANGES_TO must be 0 or 1");
static_assert(
    DART_HAS_STD_PRINT == 0 || DART_HAS_STD_PRINT == 1,
    "DART_HAS_STD_PRINT must be 0 or 1");
static_assert(
    DART_HAS_STD_FLAT_MAP == 0 || DART_HAS_STD_FLAT_MAP == 1,
    "DART_HAS_STD_FLAT_MAP must be 0 or 1");
static_assert(
    DART_HAS_STD_FLAT_SET == 0 || DART_HAS_STD_FLAT_SET == 1,
    "DART_HAS_STD_FLAT_SET must be 0 or 1");
static_assert(
    DART_HAS_STD_MOVE_ONLY_FUNCTION == 0
        || DART_HAS_STD_MOVE_ONLY_FUNCTION == 1,
    "DART_HAS_STD_MOVE_ONLY_FUNCTION must be 0 or 1");
static_assert(
    DART_HAS_STD_GENERATOR == 0 || DART_HAS_STD_GENERATOR == 1,
    "DART_HAS_STD_GENERATOR must be 0 or 1");
static_assert(
    DART_HAS_STD_RANGES_CHUNK == 0 || DART_HAS_STD_RANGES_CHUNK == 1,
    "DART_HAS_STD_RANGES_CHUNK must be 0 or 1");
static_assert(
    DART_HAS_STD_RANGES_SLIDE == 0 || DART_HAS_STD_RANGES_SLIDE == 1,
    "DART_HAS_STD_RANGES_SLIDE must be 0 or 1");
static_assert(
    DART_HAS_STD_MDSPAN == 0 || DART_HAS_STD_MDSPAN == 1,
    "DART_HAS_STD_MDSPAN must be 0 or 1");

// Layer 2: the "adopt now" guarantee, active only once we build as C++23.
#if defined(__cplusplus) && __cplusplus >= 202302L
static_assert(
    DART_HAS_STD_EXPECTED,
    "std::expected must be available on DART's C++23 compiler floor");
static_assert(
    DART_HAS_STD_OPTIONAL_MONADIC,
    "std::optional monadic ops must be available on DART's C++23 floor");
static_assert(
    DART_HAS_STD_TO_UNDERLYING,
    "std::to_underlying must be available on DART's C++23 floor");
static_assert(
    DART_HAS_STD_UNREACHABLE,
    "std::unreachable must be available on DART's C++23 floor");
static_assert(
    DART_HAS_DEDUCING_THIS,
    "deducing-this must be available on DART's C++23 floor");
static_assert(
    DART_HAS_MULTIDIM_SUBSCRIPT,
    "multidimensional subscript must be available on DART's C++23 floor");
static_assert(
    DART_HAS_STD_RANGES_ZIP,
    "std::views::zip must be available on DART's C++23 floor");
// NOTE: std::views::enumerate is intentionally NOT required here. libc++ (the
// macOS floor) does not implement it at all -- only libstdc++ ships it -- so it
// is a "guard" feature, used behind DART_HAS_STD_RANGES_ENUMERATE with a
// fallback rather than unconditionally. See docs/design/cpp23_modernization.md.
#endif

namespace {

enum class TrafficLight : std::uint8_t
{
  Red,
  Yellow,
  Green,
};

// Exhaustive switch with no fall-through return: DART_UNREACHABLE() documents
// to the reader and the optimizer that every enumerator is handled.
int dwellSeconds(TrafficLight light)
{
  switch (light) {
    case TrafficLight::Red:
      return 30;
    case TrafficLight::Yellow:
      return 5;
    case TrafficLight::Green:
      return 25;
  }
  DART_UNREACHABLE();
}

} // namespace

// The probe header is exercised entirely at compile time; this runtime case
// just confirms DART_UNREACHABLE() is usable as the tail of an exhaustive
// switch and never taken for valid inputs.
TEST(FeatureSupport, UnreachableTailOfExhaustiveSwitch)
{
  EXPECT_EQ(dwellSeconds(TrafficLight::Red), 30);
  EXPECT_EQ(dwellSeconds(TrafficLight::Yellow), 5);
  EXPECT_EQ(dwellSeconds(TrafficLight::Green), 25);
}

TEST(FeatureSupport, ProbesCompileAndLink)
{
  // A trivial assertion so the translation unit registers at least one gtest
  // case; the substantive checks above are all static_asserts.
  SUCCEED();
}
