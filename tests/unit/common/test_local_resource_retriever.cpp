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

#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/common/Uri.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <cstdio>

using namespace dart;
using namespace dart::common;

namespace {

class TempFile
{
public:
  explicit TempFile(const std::string& content)
  {
    mPath = createTempFile(content);
  }

  ~TempFile()
  {
    if (!mPath.empty())
      std::remove(mPath.c_str());
  }

  const std::string& path() const
  {
    return mPath;
  }

  Uri fileUri() const
  {
    return Uri("file://" + mPath);
  }

private:
  static std::string createTempFile(const std::string& content)
  {
    const auto dir = std::filesystem::temp_directory_path();
    static std::atomic<unsigned long long> counter{0};

    for (int attempt = 0; attempt < 16; ++attempt) {
      const auto stamp
          = std::chrono::steady_clock::now().time_since_epoch().count();
      const auto unique = counter.fetch_add(1, std::memory_order_relaxed);
      const auto path = dir
                        / ("dart-test-" + std::to_string(stamp) + "-"
                           + std::to_string(unique));

      if (std::filesystem::exists(path))
        continue;

      std::ofstream ofs(path, std::ios::binary);
      if (!ofs)
        continue;

      ofs << content;
      return path.string();
    }

    throw std::runtime_error("Failed to create temporary file for test.");
  }

  std::string mPath;
};

} // namespace

//==============================================================================
TEST(LocalResourceRetriever, ExistsReturnsTrueForExistingFile)
{
  TempFile file("test content");
  LocalResourceRetriever retriever;

  EXPECT_TRUE(retriever.exists(file.fileUri()));
}

TEST(LocalResourceRetriever, ExistsReturnsFalseForNonExistentFile)
{
  LocalResourceRetriever retriever;
  Uri nonExistent("file:///nonexistent/path/to/file.txt");

  EXPECT_FALSE(retriever.exists(nonExistent));
}

TEST(LocalResourceRetriever, ExistsReturnsFalseForNonFileUri)
{
  LocalResourceRetriever retriever;
  Uri httpUri("http://example.com/file.txt");

  EXPECT_FALSE(retriever.exists(httpUri));
}

TEST(LocalResourceRetriever, ExistsReturnsFalseForEmptyUri)
{
  LocalResourceRetriever retriever;
  Uri emptyUri;

  EXPECT_FALSE(retriever.exists(emptyUri));
}

//==============================================================================
TEST(LocalResourceRetriever, RetrieveReturnsResourceForExistingFile)
{
  const std::string content = "Hello, DART!";
  TempFile file(content);
  LocalResourceRetriever retriever;

  ResourcePtr resource = retriever.retrieve(file.fileUri());

  ASSERT_NE(resource, nullptr);
  EXPECT_GT(resource->getSize(), 0u);
}

TEST(LocalResourceRetriever, RetrieveReturnsNullptrForNonExistentFile)
{
  LocalResourceRetriever retriever;
  Uri nonExistent("file:///nonexistent/path/to/file.txt");

  ResourcePtr resource = retriever.retrieve(nonExistent);

  EXPECT_EQ(resource, nullptr);
}

TEST(LocalResourceRetriever, RetrieveReturnsNullptrForNonFileUri)
{
  LocalResourceRetriever retriever;
  Uri httpUri("http://example.com/file.txt");

  ResourcePtr resource = retriever.retrieve(httpUri);

  EXPECT_EQ(resource, nullptr);
}

TEST(LocalResourceRetriever, RetrievedResourceContentIsCorrect)
{
  const std::string expectedContent = "Test file content 12345";
  TempFile file(expectedContent);
  LocalResourceRetriever retriever;

  ResourcePtr resource = retriever.retrieve(file.fileUri());

  ASSERT_NE(resource, nullptr);

  std::size_t size = resource->getSize();
  std::vector<char> buffer(size);
  std::size_t bytesRead = resource->read(buffer.data(), size, 1);

  EXPECT_EQ(bytesRead, 1u);
  std::string actualContent(buffer.begin(), buffer.end());
  EXPECT_EQ(actualContent, expectedContent);
}

TEST(LocalResourceRetriever, RetrievedResourceCanSeek)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResourceRetriever retriever;

  ResourcePtr resource = retriever.retrieve(file.fileUri());

  ASSERT_NE(resource, nullptr);

  resource->seek(5, Resource::SEEKTYPE_SET);
  EXPECT_EQ(resource->tell(), 5u);

  char c;
  resource->read(&c, 1, 1);
  EXPECT_EQ(c, '5');
}

TEST(LocalResourceRetriever, RetrievedResourceReportsSize)
{
  const std::string content = "Exactly 24 bytes here!!";
  TempFile file(content);
  LocalResourceRetriever retriever;

  ResourcePtr resource = retriever.retrieve(file.fileUri());

  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->getSize(), content.size());
}

//==============================================================================
TEST(LocalResourceRetriever, GetFilePathReturnsPathForFileUri)
{
  TempFile file("content");
  LocalResourceRetriever retriever;

#if defined(__GNUC__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  std::string filePath = retriever.getFilePath(file.fileUri());
#if defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif

  EXPECT_FALSE(filePath.empty());
  EXPECT_EQ(filePath, file.path());
}

TEST(LocalResourceRetriever, GetFilePathReturnsEmptyForNonFileUri)
{
  LocalResourceRetriever retriever;
  Uri httpUri("http://example.com/file.txt");

#if defined(__GNUC__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  std::string filePath = retriever.getFilePath(httpUri);
#if defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif

  EXPECT_TRUE(filePath.empty());
}

//==============================================================================
TEST(LocalResourceRetriever, HandlesFileUriWithoutAuthority)
{
  TempFile file("content");
  LocalResourceRetriever retriever;

  Uri uriWithoutAuthority;
  uriWithoutAuthority.mScheme = "file";
  uriWithoutAuthority.mPath = file.path();

  EXPECT_TRUE(retriever.exists(uriWithoutAuthority));
}

TEST(LocalResourceRetriever, RetrieveEmptyFile)
{
  TempFile file("");
  LocalResourceRetriever retriever;

  ResourcePtr resource = retriever.retrieve(file.fileUri());

  ASSERT_NE(resource, nullptr);
  EXPECT_EQ(resource->getSize(), 0u);
}

TEST(LocalResourceRetriever, MultipleRetrievalsOfSameFile)
{
  const std::string content = "shared content";
  TempFile file(content);
  LocalResourceRetriever retriever;

  ResourcePtr resource1 = retriever.retrieve(file.fileUri());
  ResourcePtr resource2 = retriever.retrieve(file.fileUri());

  ASSERT_NE(resource1, nullptr);
  ASSERT_NE(resource2, nullptr);

  EXPECT_EQ(resource1->getSize(), resource2->getSize());
}
