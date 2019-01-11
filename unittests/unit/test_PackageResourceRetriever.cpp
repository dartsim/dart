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

#include <gtest/gtest.h>
#include "dart/config.hpp"
#include "dart/io/PackageResourceRetriever.hpp"
#include "TestHelpers.hpp"

using dart::common::Uri;
using dart::common::Resource;
using dart::common::ResourcePtr;
using dart::common::ResourceRetriever;
using dart::io::PackageResourceRetriever;

TEST(PackageResourceRetriever, exists_UnableToResolve_ReturnsFalse)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_DelegateFails_ReturnsFalse)
{
  // Additional slash is required for Windows because Windows file system
  // doesn't have a leading slash for an absolute path.
  // Reference: https://en.wikipedia.org/wiki/File_URI_scheme#Windows
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test");

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(1u, mockRetriever->mExists.size());
  EXPECT_EQ(expected, mockRetriever->mExists.front());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_UnsupportedUri_ReturnsFalse)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test");

  EXPECT_FALSE(retriever.exists(Uri::createFromString("foo://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_StripsTrailingSlash)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test/");

  EXPECT_TRUE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(1u, mockRetriever->mExists.size());
  EXPECT_EQ(expected, mockRetriever->mExists.front());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_FirstUriSucceeds)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test1/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test1/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test2");

  EXPECT_TRUE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(1u, mockRetriever->mExists.size());
  EXPECT_EQ(expected, mockRetriever->mExists.front());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, exists_FallsBackOnSecondUri)
{
#ifdef _WIN32
  const char* expected1 = "file:///" DART_DATA_LOCAL_PATH"test1/foo";
  const char* expected2 = "file:///" DART_DATA_LOCAL_PATH"test2/foo";
#else
  const char* expected1 = "file://" DART_DATA_LOCAL_PATH"test1/foo";
  const char* expected2 = "file://" DART_DATA_LOCAL_PATH"test2/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test2");

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
  ASSERT_EQ(2u, mockRetriever->mExists.size());
  EXPECT_EQ(expected1, mockRetriever->mExists[0]);
  EXPECT_EQ(expected2, mockRetriever->mExists[1]);
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, getFilePath_UnableToResolve_ReturnsEmptyString)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);

  EXPECT_EQ(retriever.getFilePath(Uri::createFromString("package://test/foo")), "");
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, getFilePath_DelegateFails_ReturnsEmptyString)
{
  // GTest breaks the string concatenation.
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test");

  EXPECT_EQ(retriever.getFilePath(
      Uri::createFromString("package://test/foo")), "");
  ASSERT_TRUE(mockRetriever->mExists.empty());
  EXPECT_EQ(expected, mockRetriever->mGetFilePath.front());
  ASSERT_EQ(1u, mockRetriever->mGetFilePath.size());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, getFilePath_UnsupportedUri_ReturnsEmptyString)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test");

  EXPECT_EQ(retriever.getFilePath(Uri::createFromString("foo://test/foo")), "");
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mGetFilePath.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, getFilePath_StripsTrailingSlash)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test/");

  EXPECT_EQ(retriever.getFilePath(
      Uri::createFromString("package://test/foo")), expected);
  ASSERT_TRUE(mockRetriever->mExists.empty());
  EXPECT_EQ(expected, mockRetriever->mGetFilePath.front());
  ASSERT_EQ(1u, mockRetriever->mGetFilePath.size());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, getFilePath_FirstUriSucceeds)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test1/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test1/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test2");

  EXPECT_EQ(retriever.getFilePath(
      Uri::createFromString("package://test/foo")), expected);
  ASSERT_TRUE(mockRetriever->mExists.empty());
  EXPECT_EQ(expected, mockRetriever->mGetFilePath.front());
  ASSERT_EQ(1u, mockRetriever->mGetFilePath.size());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, getFilePath_FallsBackOnSecondUri)
{
#ifdef _WIN32
  const char* expected1 = "file:///" DART_DATA_LOCAL_PATH"test1/foo";
  const char* expected2 = "file:///" DART_DATA_LOCAL_PATH"test2/foo";
#else
  const char* expected1 = "file://" DART_DATA_LOCAL_PATH"test1/foo";
  const char* expected2 = "file://" DART_DATA_LOCAL_PATH"test2/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test2");

  EXPECT_EQ(retriever.getFilePath(
      Uri::createFromString("package://test/foo")), "");
  ASSERT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(2u, mockRetriever->mGetFilePath.size());
  EXPECT_EQ(expected1, mockRetriever->mGetFilePath[0]);
  EXPECT_EQ(expected2, mockRetriever->mGetFilePath[1]);
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
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test");

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(1u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected, mockRetriever->mRetrieve.front());
}

TEST(PackageResourceRetriever, retrieve_UnsupportedUri_ReturnsNull)
{
  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test");

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("foo://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  EXPECT_TRUE(mockRetriever->mRetrieve.empty());
}

TEST(PackageResourceRetriever, retrieve_StripsTrailingSlash)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test/");

  EXPECT_TRUE(retriever.retrieve(Uri::createFromString("package://test/foo")) != nullptr);
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(1u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected, mockRetriever->mRetrieve.front());
}

TEST(PackageResourceRetriever, retrieve_FirstUriSucceeds)
{
#ifdef _WIN32
  const char* expected = "file:///" DART_DATA_LOCAL_PATH"test1/foo";
#else
  const char* expected = "file://" DART_DATA_LOCAL_PATH"test1/foo";
#endif

  auto mockRetriever = std::make_shared<PresentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test2");

  EXPECT_TRUE(retriever.retrieve(Uri::createFromString("package://test/foo")) != nullptr);
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(1u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected, mockRetriever->mRetrieve.front());
}

TEST(PackageResourceRetriever, retrieve_FallsBackOnSecondUri)
{
#ifdef _WIN32
  const char* expected1 = "file:///" DART_DATA_LOCAL_PATH"test1/foo";
  const char* expected2 = "file:///" DART_DATA_LOCAL_PATH"test2/foo";
#else
  const char* expected1 = "file://" DART_DATA_LOCAL_PATH"test1/foo";
  const char* expected2 = "file://" DART_DATA_LOCAL_PATH"test2/foo";
#endif

  auto mockRetriever = std::make_shared<AbsentResourceRetriever>();
  PackageResourceRetriever retriever(mockRetriever);
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test1");
  retriever.addPackageDirectory("test", DART_DATA_LOCAL_PATH"test2");

  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(mockRetriever->mExists.empty());
  ASSERT_EQ(2u, mockRetriever->mRetrieve.size());
  EXPECT_EQ(expected1, mockRetriever->mRetrieve[0]);
  EXPECT_EQ(expected2, mockRetriever->mRetrieve[1]);
}
