/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <gtest/gtest.h>
#include "dart/utils/PackageResourceRetriever.hpp"
#include <test/TestHelpers.hpp>

using dart::common::Uri;
using dart::common::Resource;
using dart::common::ResourcePtr;
using dart::common::ResourceRetriever;
using dart::utils::PackageResourceRetriever;

TEST(PackageResourceRetriever, exists_UnableToResolve_ReturnsFalse)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_DelegateFails_ReturnsFalse)
{
  // GTest breaks the string concatenation.
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test");

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(1u, mockRetriever->mExists.size());
  EXPECT_EQ(expected, mockRetriever->mExists.front());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_UnsupportedUri_ReturnsFalse)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test");

  EXPECT_FALSE(retriever.exists(Uri::createFromString("foo://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_StripsTrailingSlash)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test/");

  EXPECT_TRUE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(1u, mockRetriever->mExists.size());
  EXPECT_EQ(expected, mockRetriever->mExists.front());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_FirstUriSucceeds)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_PATH"test1/foo";
#else
  const char* expected = "file://" DART_DATA_PATH"test1/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_PATH"test2");

  EXPECT_TRUE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(1u, mockRetriever->mExists.size());
  EXPECT_EQ(expected, mockRetriever->mExists.front());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_FallsBackOnSecondUri)
{
#ifdef _WIN32
  const char* expected1 = "file:///" DART_DATA_PATH"test1/foo";
  const char* expected2 = "file:///" DART_DATA_PATH"test2/foo";
#else
  const char* expected1 = "file://" DART_DATA_PATH"test1/foo";
  const char* expected2 = "file://" DART_DATA_PATH"test2/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_PATH"test2");

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(2u, mockRetriever->mExists.size());
  EXPECT_EQ(expected1, mockRetriever->mExists[0]);
  EXPECT_EQ(expected2, mockRetriever->mExists[1]);
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, retrieve_UnableToResolve_ReturnsNull)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, retrieve_DelegateFails_ReturnsNull)
{
  // GTest breaks the string concatenation.
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test");

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(1u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected, mockRetriever->mRetrieve.front());
}

TEST(PackageResourceRetriever, retrieve_UnsupportedUri_ReturnsNull)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test");

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("foo://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, retrieve_StripsTrailingSlash)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test/");

  EXPECT_TRUE(retriever.retrieve(Uri::createFromString("package://test/foo")) != nullptr);
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(1u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected, mockRetriever->mRetrieve.front());
}

TEST(PackageResourceRetriever, retrieve_FirstUriSucceeds)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_PATH"test1/foo";
#else
  const char* expected = "file://" DART_DATA_PATH"test1/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_PATH"test2");

  EXPECT_TRUE(retriever.retrieve(Uri::createFromString("package://test/foo")) != nullptr);
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(1u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected, mockRetriever->mRetrieve.front());
}

TEST(PackageResourceRetriever, retrieve_FallsBackOnSecondUri)
{
#ifdef _WIN32
  const char* expected1 = "file:///" DART_DATA_PATH"test1/foo";
  const char* expected2 = "file:///" DART_DATA_PATH"test2/foo";
#else
  const char* expected1 = "file://" DART_DATA_PATH"test1/foo";
  const char* expected2 = "file://" DART_DATA_PATH"test2/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_PATH"test2");

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(2u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected1, mockRetriever->mRetrieve[0]);
  EXPECT_EQ(expected2, mockRetriever->mRetrieve[1]);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
