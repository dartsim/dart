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

#include <dart/constraint/detail/FrictionRows.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

using namespace dart::constraint::detail;

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();

struct Rows
{
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<int> findex;
};

FrictionRowClassification classify(const Rows& rows, bool box = true)
{
  FrictionRowClassification result;
  EXPECT_TRUE(classifyFrictionRows(
      static_cast<int>(rows.lo.size()),
      rows.lo.data(),
      rows.hi.data(),
      rows.findex.data(),
      result,
      box));
  std::vector<int> all = result.scalarRows;
  all.insert(all.end(), result.contactRows.begin(), result.contactRows.end());
  std::sort(all.begin(), all.end());
  EXPECT_EQ(all.size(), rows.lo.size());
  for (std::size_t i = 0; i < all.size(); ++i)
    EXPECT_EQ(all[i], static_cast<int>(i));
  return result;
}

} // namespace

TEST(FrictionRows, ClassificationTable)
{
  const Rows rows{
      {0, -.5, -.5, 0, -.7, -150, 0, -.7, -1e-12, 0, -infinity, -2, 0, 0, 0},
      {infinity,
       .5,
       .5,
       infinity,
       .7,
       150,
       infinity,
       .7,
       1e-12,
       infinity,
       infinity,
       2,
       infinity,
       0,
       0},
      {-1, 0, 0, 3, 3, 3, -1, 6, 6, -1, -1, -1, -1, -1, -1}};
  const auto result = classify(rows);
  ASSERT_EQ(result.contacts.size(), 3);
  EXPECT_EQ(result.numBoxContacts, 2);
  EXPECT_EQ(result.scalarRows, (std::vector<int>{9, 10, 11, 12, 13, 14}));
  EXPECT_EQ(result.contacts[0].normalRow, 0);
  EXPECT_EQ(result.contacts[0].tangentRows, (std::array<int, 2>{{1, 2}}));
  EXPECT_EQ(result.contacts[0].cone.law, FrictionConeLaw::Ellipse);
  EXPECT_EQ(result.contacts[1].cone.law, FrictionConeLaw::Box);
  EXPECT_EQ(result.contacts[1].cone.mu, Eigen::Vector2d(.7, 150));
  EXPECT_EQ(result.contacts[2].cone.mu, Eigen::Vector2d(.7, 0));
  for (const auto& contact : result.contacts)
    EXPECT_FALSE(contact.usePgs);
}

TEST(FrictionRows, AnisotropicSwitchAndOneDimensionalReduction)
{
  for (const double delta : {0., .5e-9, 2e-9}) {
    const Rows rows{
        {0, -1, -(1 + delta)}, {infinity, 1, 1 + delta}, {-1, 0, 0}};
    auto result = classify(rows);
    EXPECT_EQ(
        result.contacts[0].cone.law,
        delta > 1e-9 ? FrictionConeLaw::Box : FrictionConeLaw::Ellipse);
    result = classify(rows, false);
    EXPECT_EQ(result.contacts[0].cone.law, FrictionConeLaw::Ellipse);
  }
  for (const double minor : {0., 1e-12, 1e-7, 1e-6, 2e-6}) {
    const auto result
        = classify({{0, -minor, -1}, {infinity, minor, 1}, {-1, 0, 0}}, false);
    EXPECT_EQ(result.contacts[0].cone.mu[0], minor <= 1e-6 ? 0 : minor);
    EXPECT_EQ(result.contacts[0].cone.mu[1], 1);
  }
  for (const double mu : {0., 1e-12, .5}) {
    const auto result = classify({{0, -mu}, {infinity, mu}, {-1, 0}});
    EXPECT_FALSE(result.contacts[0].usePgs);
    EXPECT_EQ(result.contacts[0].cone.mu, Eigen::Vector2d(mu, 0));
  }
}

TEST(FrictionRows, MalformedCouplingIsRetainedForPgs)
{
  const std::vector<Rows> bank{
      {{0, -.5, -.5}, {3, .5, .5}, {-1, 0, 0}},        // finite normal cap
      {{0, -.4, -.5}, {infinity, .5, .5}, {-1, 0, 0}}, // asymmetry
      {{0, -.5, -.5, -.5}, {infinity, .5, .5, .5}, {-1, 0, 0, 0}},
      {{-1, -.5}, {1, .5}, {-1, 0}},                   // non-normal parent
      {{0, -.5, -.5}, {infinity, .5, .5}, {-1, 0, 1}}, // chain
      {{-.5, -.5}, {.5, .5}, {1, 0}},                  // cycle
      {{0, -infinity}, {infinity, infinity}, {-1, 0}}};
  for (std::size_t i = 0; i < bank.size(); ++i) {
    SCOPED_TRACE(i);
    const auto result = classify(bank[i]);
    ASSERT_EQ(result.contacts.size(), 1);
    EXPECT_TRUE(result.contacts[0].usePgs);
    EXPECT_EQ(result.contacts[0].cone.law, FrictionConeLaw::Box);
    EXPECT_EQ(result.numBoxContacts, 1);
    EXPECT_TRUE(result.scalarRows.empty());
    EXPECT_EQ(result.contacts[0].rowCount, bank[i].lo.size());
  }
}

TEST(FrictionRows, NonContiguousContactsAndReadOnlyInputs)
{
  const auto reversed
      = classify({{-.5, 0, -.5}, {.5, infinity, .5}, {1, -1, 1}}, false);
  ASSERT_EQ(reversed.contacts.size(), 1);
  EXPECT_FALSE(reversed.contacts[0].usePgs);
  EXPECT_EQ(reversed.contacts[0].normalRow, 1);
  EXPECT_EQ(reversed.contacts[0].tangentRows, (std::array<int, 2>{{0, 2}}));
  EXPECT_EQ(reversed.numBoxContacts, 0);
  const Rows rows{
      {0, 0, -infinity, -.5, -.7, -.5, -150},
      {infinity, infinity, infinity, .5, .7, .5, 150},
      {-1, -1, 2, 0, 1, 0, 1}};
  const auto original = rows;
  const auto result = classify(rows);
  ASSERT_EQ(result.contacts.size(), 2);
  EXPECT_EQ(result.contactRows, (std::vector<int>{0, 3, 5, 1, 4, 6}));
  EXPECT_EQ(result.scalarRows, (std::vector<int>{2}));
  EXPECT_EQ(rows.lo, original.lo);
  EXPECT_EQ(rows.hi, original.hi);
  EXPECT_EQ(rows.findex, original.findex);
  FrictionRowClassification scratch;
  for (int repeat = 0; repeat < 10; ++repeat) {
    ASSERT_TRUE(classifyFrictionRows(
        7, rows.lo.data(), rows.hi.data(), rows.findex.data(), scratch));
    EXPECT_EQ(scratch.contactRows, result.contactRows);
    EXPECT_EQ(scratch.scalarRows, result.scalarRows);
  }
}

TEST(FrictionRows, InvalidIndicesAreRejectedAndEmptyGroupIsValid)
{
  FrictionRowClassification result;
  EXPECT_TRUE(classifyFrictionRows(0, nullptr, nullptr, nullptr, result));
  EXPECT_FALSE(classifyFrictionRows(-1, nullptr, nullptr, nullptr, result));
  EXPECT_FALSE(classifyFrictionRows(1, nullptr, nullptr, nullptr, result));
  const double lo = 0, hi = infinity;
  for (const int parent : {-2, 1, 100})
    EXPECT_FALSE(classifyFrictionRows(1, &lo, &hi, &parent, result));
}
