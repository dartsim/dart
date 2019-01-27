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
#include "dart/common/LocalResourceRetriever.hpp"
#include "TestHelpers.hpp"

using dart::common::Uri;
using dart::common::Resource;
using dart::common::LocalResourceRetriever;

#ifdef _WIN32
  #define FILE_SCHEME "file:///"
#else
  #define FILE_SCHEME "file://"
#endif

TEST(LocalResourceRetriever, exists_UnsupportedUri_ReturnsFalse)
{
  LocalResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists("unknown://test"));
}

TEST(LocalResourceRetriever, exists_FileUriDoesNotExist_ReturnsFalse)
{
  LocalResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(FILE_SCHEME DART_DATA_PATH "does/not/exist"));
}

TEST(LocalResourceRetriever, exists_PathDoesNotExist_ReturnsFalse)
{
  LocalResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(DART_DATA_PATH "does/not/exist"));
}

TEST(LocalResourceRetriever, exists_FileUriDoesExists_ReturnsTrue)
{
  LocalResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(FILE_SCHEME DART_DATA_PATH "skel/cube.skel"));
}

TEST(LocalResourceRetriever, exists_PathDoesExists_ReturnsTrue)
{
  LocalResourceRetriever retriever;
  EXPECT_TRUE(retriever.exists(DART_DATA_PATH "skel/cube.skel"));
}

TEST(LocalResourceRetriever, getFilePath_UnsupportedUri_ReturnsEmptyString)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(retriever.getFilePath("unknown://test"), "");
}

TEST(LocalResourceRetriever, getFilePath_FileUriDoesNotExist_ReturnsEmptyString)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(
      retriever.getFilePath(FILE_SCHEME DART_DATA_PATH "does/not/exist"), "");
}

TEST(LocalResourceRetriever, getFilePath_PathDoesNotExist_ReturnsEmptyString)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(retriever.getFilePath(DART_DATA_PATH "does/not/exist"), "");
}

TEST(LocalResourceRetriever, getFilePath_FileUriDoesExists_ReturnsPath)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(
      retriever.getFilePath(FILE_SCHEME DART_DATA_PATH "skel/cube.skel"),
      DART_DATA_PATH"skel/cube.skel");
}

TEST(LocalResourceRetriever, getFilePath_PathDoesExists_ReturnsPath)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(
      retriever.getFilePath(DART_DATA_PATH "skel/cube.skel"),
      DART_DATA_PATH"skel/cube.skel");
}

TEST(LocalResourceRetriever, retrieve_UnsupportedUri_ReturnsNull)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(nullptr, retriever.retrieve("unknown://test"));
}

TEST(LocalResourceRetriever, retrieve_FileUriDoesNotExist_ReturnsNull)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(nullptr, retriever.retrieve(FILE_SCHEME DART_DATA_PATH "does/not/exist"));
}

TEST(LocalResourceRetriever, retrieve_PathDoesNotExist_ReturnsNull)
{
  LocalResourceRetriever retriever;
  EXPECT_EQ(nullptr, retriever.retrieve(DART_DATA_PATH "does/not/exist"));
}

TEST(LocalResourceRetriever, retrieve_FileUri)
{
  LocalResourceRetriever retriever;
  auto resource = retriever.retrieve(FILE_SCHEME DART_DATA_PATH "test/hello_world.txt");
  ASSERT_TRUE(resource != nullptr);
}

TEST(LocalResourceRetriever, retrieve_Path)
{
  LocalResourceRetriever retriever;
  auto resource = retriever.retrieve(DART_DATA_PATH "test/hello_world.txt");
  ASSERT_TRUE(resource != nullptr);
}

TEST(LocalResourceRetriever, readAll)
{
  LocalResourceRetriever retriever;
  auto resource = retriever.retrieve(DART_DATA_PATH "test/hello_world.txt");
  ASSERT_TRUE(resource != nullptr);

  auto content = resource->readAll();
  ASSERT_TRUE(content == std::string("Hello World"));

  ASSERT_TRUE(retriever.readAll(DART_DATA_PATH "test/hello_world.txt")
              == std::string("Hello World"));
}

TEST(LocalResourceRetriever, retrieve_ResourceOperations)
{
  const std::string content = "Hello World";

  std::vector<char> buffer(100, '\0');

  LocalResourceRetriever retriever;
  auto resource = retriever.retrieve(DART_DATA_PATH "test/hello_world.txt");
  ASSERT_TRUE(resource != nullptr);

  EXPECT_EQ(content.size(), resource->getSize());

  // Relative seek.
  ASSERT_TRUE(resource->seek(2, Resource::SEEKTYPE_CUR));
  EXPECT_EQ(2u, resource->tell());

  // Absolute seek.
  ASSERT_TRUE(resource->seek(5, Resource::SEEKTYPE_SET));
  EXPECT_EQ(5u, resource->tell());

  // Seek to the end of the file.
  ASSERT_TRUE(resource->seek(0, Resource::SEEKTYPE_END));
  EXPECT_EQ(content.size(), resource->tell());

  // TODO: SEEKTYPE_END should require negative input.
  ASSERT_TRUE(resource->seek(-3, Resource::SEEKTYPE_END));
  EXPECT_EQ(content.size() - 3, resource->tell());

  // Reading a block that's too large should do nothing.
  ASSERT_TRUE(resource->seek(0, Resource::SEEKTYPE_SET));
  ASSERT_EQ(0u, resource->read(buffer.data(), content.size() + 1, 1));

  // Reading should only return full blocks.
  buffer.assign(buffer.size(), '\0');
  ASSERT_TRUE(resource->seek(0, Resource::SEEKTYPE_SET));
  ASSERT_EQ(1u, resource->read(buffer.data(), 8, 1));
  EXPECT_STREQ(content.substr(0, 8).c_str(), buffer.data());

  // Reading multiple blocks
  buffer.assign(buffer.size(), '\0');
  ASSERT_TRUE(resource->seek(0, Resource::SEEKTYPE_SET));
  ASSERT_EQ(2u, resource->read(buffer.data(), 4, 2));
  EXPECT_STREQ(content.substr(0, 8).c_str(), buffer.data());

  // Reading the whole file at once.
  buffer.assign(buffer.size(), '\0');
  ASSERT_TRUE(resource->seek(0, Resource::SEEKTYPE_SET));
  ASSERT_EQ(1u, resource->read(buffer.data(), content.size(), 1));
  EXPECT_STREQ(content.c_str(), buffer.data());
}
