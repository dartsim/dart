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

#include "dart/common/diagnostics.hpp"
#include "dart/config.hpp"
#include "dart/utils/dart_resource_retriever.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <cstdlib>

using namespace dart;

DART_SUPPRESS_DEPRECATED_BEGIN

namespace {

std::filesystem::path makeTempDirectory(const std::string& tag)
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path()
         / (tag + "_" + std::to_string(now));
}

void setEnvironmentVariable(const std::string& name, const std::string& value)
{
#ifdef _WIN32
  _putenv_s(name.c_str(), value.c_str());
#else
  setenv(name.c_str(), value.c_str(), 1);
#endif
}

void unsetEnvironmentVariable(const std::string& name)
{
#ifdef _WIN32
  _putenv_s(name.c_str(), "");
#else
  unsetenv(name.c_str());
#endif
}

class ScopedEnvironmentVariable
{
public:
  ScopedEnvironmentVariable(std::string name, std::string value)
    : mName(std::move(name))
  {
    if (const char* existing = std::getenv(mName.c_str())) {
      mOriginalValue = existing;
    }
    setEnvironmentVariable(mName, value);
  }

  ~ScopedEnvironmentVariable()
  {
    if (mOriginalValue) {
      setEnvironmentVariable(mName, *mOriginalValue);
    } else {
      unsetEnvironmentVariable(mName);
    }
  }

private:
  std::string mName;
  std::optional<std::string> mOriginalValue;
};

} // namespace

//==============================================================================
TEST(DartResourceRetriever, ExistsAndGetFilePathAndRetrieve)
{
  auto retriever = utils::DartResourceRetriever::create();

  EXPECT_FALSE(retriever->exists("unknown://test"));
  EXPECT_FALSE(retriever->exists("unknown://sample/test"));
  EXPECT_FALSE(retriever->exists("dart://unknown/test"));
  EXPECT_FALSE(retriever->exists("dart://sample/does/not/exist"));
  EXPECT_TRUE(retriever->exists("dart://sample/sdf/test/shapes.sdf"));

  EXPECT_EQ(retriever->getFilePath("unknown://test"), "");
  EXPECT_EQ(retriever->getFilePath("unknown://sample/test"), "");
  EXPECT_EQ(retriever->getFilePath("dart://unknown/test"), "");
  EXPECT_EQ(retriever->getFilePath("dart://sample/does/not/exist"), "");
  EXPECT_EQ(
      retriever->getFilePath("dart://sample/sdf/test/shapes.sdf"),
      dart::config::dataPath("sdf/test/shapes.sdf"));

  EXPECT_EQ(nullptr, retriever->retrieve("unknown://test"));
  EXPECT_EQ(nullptr, retriever->retrieve("unknown://sample/test"));
  EXPECT_EQ(nullptr, retriever->retrieve("dart://unknown/test"));
  EXPECT_EQ(nullptr, retriever->retrieve("dart://sample/does/not/exist"));
  EXPECT_NE(nullptr, retriever->retrieve("dart://sample/sdf/test/shapes.sdf"));
}

//==============================================================================
TEST(DartResourceRetriever, LocalAndCustomDataDirectoryLookups)
{
  const auto tempDir = makeTempDirectory("dart_resource_retriever");
  ASSERT_TRUE(std::filesystem::create_directories(tempDir));

  const auto sampleFile = tempDir / "custom.txt";
  {
    std::ofstream out(sampleFile);
    out << "sample data";
  }

  std::shared_ptr<utils::DartResourceRetriever> retriever;
  {
    ScopedEnvironmentVariable dataPath("DART_DATA_PATH", tempDir.string());
    retriever = utils::DartResourceRetriever::create();
  }

  EXPECT_TRUE(retriever->exists("dart://sample/custom.txt"));
  EXPECT_NE(nullptr, retriever->retrieve("dart://sample/custom.txt"));

  std::error_code ec;
  EXPECT_TRUE(
      std::filesystem::equivalent(
          retriever->getFilePath("dart://sample/custom.txt"), sampleFile, ec));
  EXPECT_FALSE(ec);

  std::filesystem::remove_all(tempDir);
}

//==============================================================================
TEST(DartResourceRetriever, DirectUriLocalPathAndMalformedDataUri)
{
  const auto tempDir = makeTempDirectory("dart_resource_retriever_direct_uri");
  ASSERT_TRUE(std::filesystem::create_directories(tempDir));

  const auto localFile = tempDir / "local_resource.txt";
  {
    std::ofstream out(localFile);
    out << "local data";
  }

  auto retriever = utils::DartResourceRetriever::create();

  common::Uri localPathUri;
  localPathUri.mAuthority = "local";
  localPathUri.mPath = localFile.string();
  EXPECT_TRUE(retriever->exists(localPathUri));
  EXPECT_NE(nullptr, retriever->retrieve(localPathUri));
  EXPECT_EQ(retriever->getFilePath(localPathUri), localFile.string());

  common::Uri missingPathUri;
  missingPathUri.mScheme = "dart";
  missingPathUri.mAuthority = "sample";
  EXPECT_FALSE(retriever->exists(missingPathUri));
  EXPECT_EQ(nullptr, retriever->retrieve(missingPathUri));
  EXPECT_EQ(retriever->getFilePath(missingPathUri), "");

  std::error_code ec;
  std::filesystem::remove_all(tempDir, ec);
}

DART_SUPPRESS_DEPRECATED_END
