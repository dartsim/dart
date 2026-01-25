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

#include <dart/common/resource.hpp>
#include <dart/common/resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

#include <cstring>

using namespace dart::common;

namespace {

/// Mock Resource that returns predefined content
class MockResource : public Resource
{
public:
  explicit MockResource(std::string content)
    : mContent(std::move(content)), mPosition(0)
  {
  }

  std::size_t getSize() override
  {
    return mContent.size();
  }

  std::size_t tell() override
  {
    return mPosition;
  }

  bool seek(ptrdiff_t offset, SeekType origin) override
  {
    std::size_t newPos = 0;
    switch (origin) {
      case SEEKTYPE_SET:
        newPos = static_cast<std::size_t>(offset);
        break;
      case SEEKTYPE_CUR:
        newPos = mPosition + static_cast<std::size_t>(offset);
        break;
      case SEEKTYPE_END:
        newPos = mContent.size() + static_cast<std::size_t>(offset);
        break;
    }
    if (newPos > mContent.size()) {
      return false;
    }
    mPosition = newPos;
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    const std::size_t totalBytes = size * count;
    const std::size_t available = mContent.size() - mPosition;
    const std::size_t toRead = std::min(totalBytes, available);

    if (toRead == 0) {
      return 0;
    }

    std::memcpy(buffer, mContent.data() + mPosition, toRead);
    mPosition += toRead;

    return toRead / size;
  }

private:
  std::string mContent;
  std::size_t mPosition;
};

/// Mock ResourceRetriever for testing readAll()
class MockResourceRetriever : public ResourceRetriever
{
public:
  std::string mockContent;
  bool shouldReturnNull = false;

  bool exists(const Uri& /*uri*/) override
  {
    return !shouldReturnNull;
  }

  ResourcePtr retrieve(const Uri& /*uri*/) override
  {
    if (shouldReturnNull) {
      return nullptr;
    }
    return std::make_shared<MockResource>(mockContent);
  }
};

} // namespace

//==============================================================================
TEST(ResourceRetriever, ReadAllSuccess)
{
  MockResourceRetriever retriever;
  retriever.mockContent = "hello world";

  Uri uri("mock://test");
  std::string result = retriever.readAll(uri);

  EXPECT_EQ(result, "hello world");
}

//==============================================================================
TEST(ResourceRetriever, ReadAllFailsWhenRetrieveReturnsNull)
{
  MockResourceRetriever retriever;
  retriever.shouldReturnNull = true;

  Uri uri("mock://test");

  EXPECT_THROW(retriever.readAll(uri), std::runtime_error);
}

//==============================================================================
TEST(ResourceRetriever, ReadAllEmptyContentThrows)
{
  MockResourceRetriever retriever;
  retriever.mockContent = "";

  Uri uri("mock://test");

  EXPECT_THROW(retriever.readAll(uri), std::runtime_error);
}

//==============================================================================
TEST(ResourceRetriever, ReadAllLargeContent)
{
  MockResourceRetriever retriever;
  retriever.mockContent = std::string(10240, 'X');

  Uri uri("mock://test");
  std::string result = retriever.readAll(uri);

  EXPECT_EQ(result.size(), 10240u);
  EXPECT_EQ(result, retriever.mockContent);
}

//==============================================================================
TEST(ResourceRetriever, ReadAllWithSpecialCharacters)
{
  MockResourceRetriever retriever;
  retriever.mockContent = "Line1\nLine2\r\nLine3\tTabbed\0NullChar";

  Uri uri("mock://test");
  std::string result = retriever.readAll(uri);

  EXPECT_EQ(result.size(), retriever.mockContent.size());
  EXPECT_EQ(result, retriever.mockContent);
}
