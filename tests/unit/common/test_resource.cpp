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

#include "../../helpers/gtest_utils.hpp"
#include "dart/common/diagnostics.hpp"
#include "dart/common/resource.hpp"
#include "dart/common/resource_retriever.hpp"
#include "dart/common/uri.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <cstring>

using namespace dart;
using namespace dart::common;

DART_SUPPRESS_DEPRECATED_BEGIN

namespace {

// Simple in-memory resource used to exercise Resource::readAll.
class StringResource : public Resource
{
public:
  explicit StringResource(std::string data) : mData(std::move(data)), mCursor(0)
  {
  }

  std::size_t getSize() override
  {
    return mData.size();
  }

  std::size_t tell() override
  {
    return mCursor;
  }

  bool seek(ptrdiff_t offset, SeekType origin) override
  {
    std::size_t base = 0;
    if (origin == SEEKTYPE_CUR)
      base = mCursor;
    else if (origin == SEEKTYPE_END)
      base = mData.size();

    const auto next = static_cast<long long>(base) + offset;
    if (next < 0 || next > static_cast<long long>(mData.size()))
      return false;

    mCursor = static_cast<std::size_t>(next);
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    const std::size_t bytes = size * count;
    if (bytes < mData.size())
      return 0;

    std::memcpy(buffer, mData.data(), mData.size());
    mCursor = mData.size();
    return 1;
  }

private:
  std::string mData;
  std::size_t mCursor;
};

class FailingResource : public Resource
{
public:
  std::size_t getSize() override
  {
    return 1u;
  }

  std::size_t tell() override
  {
    return 0u;
  }

  bool seek(ptrdiff_t, SeekType) override
  {
    return false;
  }

  std::size_t read(void*, std::size_t, std::size_t) override
  {
    return 0u;
  }
};

class EchoRetriever : public ResourceRetriever
{
public:
  explicit EchoRetriever(bool shouldSucceed = true)
    : mShouldSucceed(shouldSucceed)
  {
  }

  bool exists(const Uri& uri) override
  {
    mExistsQueries.push_back(uri.toString());
    return mShouldSucceed;
  }

  ResourcePtr retrieve(const Uri& uri) override
  {
    mRetrieveQueries.push_back(uri.toString());
    if (!mShouldSucceed)
      return nullptr;

    return std::make_shared<StringResource>("payload:" + uri.toString());
  }

  std::vector<std::string> mExistsQueries;
  std::vector<std::string> mRetrieveQueries;

private:
  bool mShouldSucceed;
};

} // namespace

//==============================================================================
TEST(ResourceTests, ReadAllReturnsFullPayload)
{
  StringResource resource("hello-resource");
  EXPECT_EQ(resource.readAll(), "hello-resource");
  EXPECT_EQ(resource.tell(), resource.getSize());
}

//==============================================================================
TEST(ResourceTests, ReadAllThrowsWhenUnderlyingReadFails)
{
  FailingResource resource;
  EXPECT_THROW(resource.readAll(), std::runtime_error);
}

//==============================================================================
TEST(ResourceRetrieverTests, ReadAllDelegatesToRetrieve)
{
  EchoRetriever retriever;
  const Uri uri("package://tests/asset.txt");

  EXPECT_TRUE(retriever.exists(uri));
  EXPECT_EQ(retriever.readAll(uri), "payload:package://tests/asset.txt");
  ASSERT_EQ(retriever.mRetrieveQueries.size(), 1u);
  EXPECT_EQ(retriever.mRetrieveQueries.front(), uri.toString());
}

//==============================================================================
TEST(ResourceRetrieverTests, ReadAllThrowsIfResourceMissing)
{
  EchoRetriever retriever(false);
  const Uri uri("file://does-not-exist");

  EXPECT_FALSE(retriever.exists(uri));
  EXPECT_THROW(retriever.readAll(uri), std::runtime_error);
}

//==============================================================================
TEST(ResourceRetrieverTests, DefaultGetFilePathIsEmpty)
{
  EchoRetriever retriever;
  const Uri uri("package://tests/asset.txt");
  EXPECT_EQ(retriever.getFilePath(uri), "");
}

DART_SUPPRESS_DEPRECATED_END
