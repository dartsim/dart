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

#include <dart/common/local_resource.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include <cstdio>

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
    if (!mPath.empty()) {
      std::remove(mPath.c_str());
    }
  }

  const std::string& path() const
  {
    return mPath;
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
                        / ("dart-localres-test-" + std::to_string(stamp) + "-"
                           + std::to_string(unique));

      if (std::filesystem::exists(path)) {
        continue;
      }

      std::ofstream ofs(path, std::ios::binary);
      if (!ofs) {
        continue;
      }

      ofs << content;
      return path.string();
    }

    throw std::runtime_error("Failed to create temporary file for test.");
  }

  std::string mPath;
};

} // namespace

//==============================================================================
TEST(LocalResourceTest, InvalidPath_IsGoodReturnsFalse)
{
  LocalResource resource("/nonexistent/path/to/file.txt");

  EXPECT_FALSE(resource.isGood());
}

TEST(LocalResourceTest, InvalidPath_GetSizeReturnsZero)
{
  LocalResource resource("/nonexistent/path/to/file.txt");

  EXPECT_EQ(resource.getSize(), 0u);
}

TEST(LocalResourceTest, InvalidPath_TellReturnsZero)
{
  LocalResource resource("/nonexistent/path/to/file.txt");

  EXPECT_EQ(resource.tell(), 0u);
}

TEST(LocalResourceTest, InvalidPath_ReadReturnsZero)
{
  LocalResource resource("/nonexistent/path/to/file.txt");

  char buffer[16];
  EXPECT_EQ(resource.read(buffer, 1, 16), 0u);
}

//==============================================================================
TEST(LocalResourceTest, ValidFile_IsGoodReturnsTrue)
{
  const std::string content = "Hello, DART!";
  TempFile file(content);
  LocalResource resource(file.path());

  EXPECT_TRUE(resource.isGood());
}

TEST(LocalResourceTest, ValidFile_GetSizeReturnsExactBytes)
{
  const std::string content = "Exactly 24 bytes here!!";
  TempFile file(content);
  LocalResource resource(file.path());

  EXPECT_EQ(resource.getSize(), content.size());
}

TEST(LocalResourceTest, ValidFile_TellStartsAtZero)
{
  const std::string content = "Test content";
  TempFile file(content);
  LocalResource resource(file.path());

  EXPECT_EQ(resource.tell(), 0u);
}

TEST(LocalResourceTest, ValidFile_TellAdvancesAfterRead)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  char buffer[5];
  std::size_t bytesRead = resource.read(buffer, 1, 5);

  EXPECT_EQ(bytesRead, 5u);
  EXPECT_EQ(resource.tell(), 5u);
}

TEST(LocalResourceTest, ValidFile_ReadReturnsCorrectContent)
{
  const std::string content = "Test file content";
  TempFile file(content);
  LocalResource resource(file.path());

  std::vector<char> buffer(content.size());
  std::size_t bytesRead = resource.read(buffer.data(), content.size(), 1);

  EXPECT_EQ(bytesRead, 1u);
  std::string actualContent(buffer.begin(), buffer.end());
  EXPECT_EQ(actualContent, content);
}

TEST(LocalResourceTest, ValidFile_EmptyFileHasZeroSize)
{
  TempFile file("");
  LocalResource resource(file.path());

  EXPECT_TRUE(resource.isGood());
  EXPECT_EQ(resource.getSize(), 0u);
}

//==============================================================================
TEST(LocalResourceTest, SeekOperations_SeekSetFromBeginning)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  EXPECT_TRUE(resource.seek(5, Resource::SEEKTYPE_SET));
  EXPECT_EQ(resource.tell(), 5u);

  char c;
  resource.read(&c, 1, 1);
  EXPECT_EQ(c, '5');
}

TEST(LocalResourceTest, SeekOperations_SeekCurFromCurrent)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  resource.seek(3, Resource::SEEKTYPE_SET);
  EXPECT_EQ(resource.tell(), 3u);

  EXPECT_TRUE(resource.seek(2, Resource::SEEKTYPE_CUR));
  EXPECT_EQ(resource.tell(), 5u);

  char c;
  resource.read(&c, 1, 1);
  EXPECT_EQ(c, '5');
}

TEST(LocalResourceTest, SeekOperations_SeekCurBackward)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  resource.seek(7, Resource::SEEKTYPE_SET);
  EXPECT_EQ(resource.tell(), 7u);

  EXPECT_TRUE(resource.seek(-4, Resource::SEEKTYPE_CUR));
  EXPECT_EQ(resource.tell(), 3u);

  char c;
  resource.read(&c, 1, 1);
  EXPECT_EQ(c, '3');
}

TEST(LocalResourceTest, SeekOperations_SeekEndFromEnd)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  EXPECT_TRUE(resource.seek(-3, Resource::SEEKTYPE_END));
  EXPECT_EQ(resource.tell(), 7u);

  char c;
  resource.read(&c, 1, 1);
  EXPECT_EQ(c, '7');
}

TEST(LocalResourceTest, SeekOperations_SeekToExactEnd)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  EXPECT_TRUE(resource.seek(0, Resource::SEEKTYPE_END));
  EXPECT_EQ(resource.tell(), content.size());
}

TEST(LocalResourceTest, SeekOperations_SeekToBeginning)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  resource.seek(5, Resource::SEEKTYPE_SET);

  EXPECT_TRUE(resource.seek(0, Resource::SEEKTYPE_SET));
  EXPECT_EQ(resource.tell(), 0u);
}

//==============================================================================
TEST(LocalResourceTest, InvalidSeekMode_ReturnsFalse)
{
  const std::string content = "Test content";
  TempFile file(content);
  LocalResource resource(file.path());

  auto invalidMode = static_cast<Resource::SeekType>(999);
  EXPECT_FALSE(resource.seek(0, invalidMode));
}

//==============================================================================
TEST(LocalResourceTest, ReadBeyondEnd_ReturnsZeroBytes)
{
  const std::string content = "Short";
  TempFile file(content);
  LocalResource resource(file.path());

  resource.seek(0, Resource::SEEKTYPE_END);

  char buffer[16];
  std::size_t bytesRead = resource.read(buffer, 1, 16);

  EXPECT_EQ(bytesRead, 0u);
}

TEST(LocalResourceTest, ReadBeyondEnd_PartialReadReturnsAvailableBytes)
{
  const std::string content = "12345";
  TempFile file(content);
  LocalResource resource(file.path());

  resource.seek(3, Resource::SEEKTYPE_SET);

  char buffer[10];
  std::size_t bytesRead = resource.read(buffer, 1, 10);

  EXPECT_EQ(bytesRead, 2u);
  EXPECT_EQ(buffer[0], '4');
  EXPECT_EQ(buffer[1], '5');
}

TEST(LocalResourceTest, ReadBeyondEnd_TellRemainsAtEnd)
{
  const std::string content = "Test";
  TempFile file(content);
  LocalResource resource(file.path());

  char buffer[10];
  resource.read(buffer, 1, 4);

  std::size_t bytesRead = resource.read(buffer, 1, 10);
  EXPECT_EQ(bytesRead, 0u);

  EXPECT_EQ(resource.tell(), content.size());
}

//==============================================================================
TEST(LocalResourceTest, GetSizePreservesPosition)
{
  const std::string content = "0123456789";
  TempFile file(content);
  LocalResource resource(file.path());

  resource.seek(5, Resource::SEEKTYPE_SET);
  EXPECT_EQ(resource.tell(), 5u);

  std::size_t size = resource.getSize();
  EXPECT_EQ(size, content.size());

  EXPECT_EQ(resource.tell(), 5u);
}

TEST(LocalResourceTest, MultipleReadsAdvancePosition)
{
  const std::string content = "ABCDEFGHIJ";
  TempFile file(content);
  LocalResource resource(file.path());

  char buffer[3];

  resource.read(buffer, 1, 3);
  EXPECT_EQ(resource.tell(), 3u);
  EXPECT_EQ(buffer[0], 'A');
  EXPECT_EQ(buffer[1], 'B');
  EXPECT_EQ(buffer[2], 'C');

  resource.read(buffer, 1, 3);
  EXPECT_EQ(resource.tell(), 6u);
  EXPECT_EQ(buffer[0], 'D');
  EXPECT_EQ(buffer[1], 'E');
  EXPECT_EQ(buffer[2], 'F');
}
