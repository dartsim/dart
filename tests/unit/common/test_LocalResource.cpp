/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "dart/common/LocalResource.hpp"

#include <gtest/gtest.h>

#include <fstream>

using namespace dart::common;

class LocalResourceTest : public ::testing::Test
{
protected:
  LocalResourceTest()
  {
    // Create a test file
    std::ofstream testFile("test.txt");
    testFile << "This is a test file.";
    testFile.close();
  }

  ~LocalResourceTest()
  {
    // Delete the test file
    std::remove("test.txt");
  }
};

TEST_F(LocalResourceTest, TestGetSize)
{
  LocalResource res("test.txt");
  std::string data = res.readAll();
  std::size_t fileSize = data.size();
  EXPECT_EQ(fileSize, res.getSize());
}

TEST_F(LocalResourceTest, TestTell)
{
  LocalResource res("test.txt");
  res.seek(10, Resource::SeekType::SET);
  std::size_t position = res.tell();
  EXPECT_EQ(position, 10);
}

TEST_F(LocalResourceTest, TestSeek)
{
  LocalResource res("test.txt");
  res.seek(10, Resource::SeekType::SET);
  std::size_t position = res.tell();
  EXPECT_EQ(position, 10);
}

TEST_F(LocalResourceTest, TestRead)
{
  LocalResource res("test.txt");
  char buffer[100];
  std::size_t bytesRead = res.read(buffer, 1, 100);
  EXPECT_EQ(bytesRead, res.getSize());
}

TEST_F(LocalResourceTest, TestReadAll)
{
  LocalResource res("test.txt");
  std::string data = res.readAll();
  EXPECT_EQ(data, "This is a test file.");
}

TEST_F(LocalResourceTest, TestIsGood)
{
  LocalResource res("test.txt");
  EXPECT_TRUE(res.isGood());
  LocalResource res2("nonexistentfile.txt");
  EXPECT_FALSE(res2.isGood());
}
