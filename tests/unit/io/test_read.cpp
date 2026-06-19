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

#include "dart/io/All.hpp"

#include <dart/config.hpp>

#include <dart/common/local_resource_retriever.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <string>
#include <utility>

using namespace dart;

namespace {

class StringResource final : public common::Resource
{
public:
  explicit StringResource(std::string content)
    : mContent(std::move(content)), mCursor(0)
  {
  }

  std::size_t getSize() override
  {
    return mContent.size();
  }

  std::size_t tell() override
  {
    return mCursor;
  }

  bool seek(ptrdiff_t offset, SeekType origin) override
  {
    ptrdiff_t base = 0;
    if (origin == SEEKTYPE_CUR) {
      base = static_cast<ptrdiff_t>(mCursor);
    } else if (origin == SEEKTYPE_END) {
      base = static_cast<ptrdiff_t>(mContent.size());
    }

    const auto next = base + offset;
    if (next < 0 || next > static_cast<ptrdiff_t>(mContent.size())) {
      return false;
    }

    mCursor = static_cast<std::size_t>(next);
    return true;
  }

  std::size_t read(void* buffer, std::size_t size, std::size_t count) override
  {
    if (size == 0 || count == 0) {
      return 0;
    }

    const auto available = mContent.size() - mCursor;
    const auto elements = std::min(count, available / size);
    const auto bytes = elements * size;
    std::copy_n(mContent.data() + mCursor, bytes, static_cast<char*>(buffer));
    mCursor += bytes;
    return elements;
  }

private:
  std::string mContent;
  std::size_t mCursor;
};

class StringResourceRetriever final : public common::ResourceRetriever
{
public:
  explicit StringResourceRetriever(std::string content)
    : mContent(std::move(content))
  {
  }

  bool exists(const common::Uri&) override
  {
    return true;
  }

  common::ResourcePtr retrieve(const common::Uri&) override
  {
    return std::make_shared<StringResource>(mContent);
  }

  std::string readAll(const common::Uri&) override
  {
    return mContent;
  }

private:
  std::string mContent;
};

io::ReadOptions optionsWithContent(std::string content)
{
  io::ReadOptions options;
  options.resourceRetriever
      = std::make_shared<StringResourceRetriever>(std::move(content));
  return options;
}

} // namespace

//==============================================================================
TEST(ReadUnit, ReadsConvertedSinglePendulumSdfFixture)
{
#if DART_HAS_SDFORMAT
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/single_pendulum.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_EQ(skeleton->getName(), "single_pendulum");
  EXPECT_NE(skeleton->getBodyNode("link 1"), nullptr);
  EXPECT_NE(skeleton->getJoint("joint 1"), nullptr);
#endif
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForMjcfSkeleton)
{
  const auto skeleton = io::readSkeleton("dart://sample/mjcf/openai/ant.xml");
  EXPECT_EQ(skeleton, nullptr);
}

#if DART_IO_HAS_URDF
//==============================================================================
TEST(ReadUnit, ReadsUrdfWithPackageDirectories)
{
  const common::Uri wamUri
      = common::Uri::createFromPath(config::dataPath("urdf/wam/wam.urdf"));
  const auto wamPackageDir = config::dataPath("urdf/wam");

  io::ReadOptions options;
  EXPECT_EQ(io::readSkeleton(wamUri, options), nullptr);

  options.addPackageDirectory("herb_description", "/does/not/exist");
  options.addPackageDirectory("herb_description", wamPackageDir);
  const auto skeleton = io::readSkeleton(wamUri, options);
  EXPECT_NE(skeleton, nullptr);
}
#endif

//==============================================================================
TEST(ReadUnit, ReturnsNullForNonExistentFile)
{
  const auto skeleton = io::readSkeleton("/nonexistent/path/robot.urdf");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForUnknownExtension)
{
  const auto skeleton = io::readSkeleton("/some/file.unknown");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForUnsupportedExplicitFormat)
{
  io::ReadOptions options;
  options.format = static_cast<io::ModelFormat>(-1);
  const common::Uri uri("memory://unsupported.xml");

  EXPECT_EQ(io::readSkeleton(uri, options), nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForInvalidAmbiguousXmlContent)
{
  struct InvalidXmlCase
  {
    const char* name;
    const char* content;
  };

  const std::array cases
      = {InvalidXmlCase{"empty", ""},
         InvalidXmlCase{"invalid", "<sdf>"},
         InvalidXmlCase{"rootless", "<?xml version=\"1.0\"?>"},
         InvalidXmlCase{"unknown_root", "<unknown />"}};

  for (const auto& testCase : cases) {
    const auto options = optionsWithContent(testCase.content);
    const common::Uri uri(std::string("memory://") + testCase.name + ".xml");

    EXPECT_EQ(io::readSkeleton(uri, options), nullptr) << testCase.name;
  }
}

//==============================================================================
TEST(ReadUnit, InfersSdfXmlRootForAmbiguousXmlUri)
{
#if DART_HAS_SDFORMAT
  const auto options = optionsWithContent(
      R"(<sdf version="1.7">
           <model name="memory_sdf">
             <link name="link">
               <inertial><mass>1.0</mass></inertial>
             </link>
           </model>
         </sdf>)");
  const common::Uri uri("memory://single_pendulum.xml");

  const auto skeleton = io::readSkeleton(uri, options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_EQ(skeleton->getName(), "memory_sdf");
#endif
}

//==============================================================================
TEST(ReadUnit, InfersUrdfFromAmbiguousXmlRootBeforeParseFailure)
{
  const auto options = optionsWithContent("<robot />");
  const common::Uri uri("memory://invalid_urdf.xml");

  EXPECT_EQ(io::readSkeleton(uri, options), nullptr);
}

//==============================================================================
TEST(ReadUnit, TryReadSkeletonReturnsErrorForInvalidPath)
{
  const auto result = io::tryReadSkeleton("/nonexistent/robot.urdf");
  EXPECT_FALSE(result.isOk());
  EXPECT_TRUE(result.isErr());
}

//==============================================================================
TEST(ReadUnit, TryReadSkeletonReturnsOkForValidFile)
{
#if DART_HAS_SDFORMAT
  const auto result
      = io::tryReadSkeleton("dart://sample/sdf/test/single_pendulum.sdf");
  EXPECT_TRUE(result.isOk());
  EXPECT_FALSE(result.isErr());
  EXPECT_NE(result.value(), nullptr);
#endif
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromSdfExtension)
{
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/single_pendulum.sdf");
#if DART_HAS_SDFORMAT
  EXPECT_NE(skeleton, nullptr);
#else
  EXPECT_EQ(skeleton, nullptr);
#endif
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromUrdfExtension)
{
#if DART_IO_HAS_URDF
  const auto skeleton
      = io::readSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  EXPECT_NE(skeleton, nullptr);
#endif
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromMjcfExtension)
{
  io::ReadOptions options;
  options.format = io::ModelFormat::Mjcf;
  const auto skeleton
      = io::readSkeleton("dart://sample/mjcf/openai/ant.xml", options);
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReadsConvertedTestShapesSdfFixture)
{
#if DART_HAS_SDFORMAT
  io::ReadOptions options;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton
      = io::readSkeleton("dart://sample/sdf/test/test_shapes.sdf", options);
  ASSERT_NE(skeleton, nullptr);
  EXPECT_EQ(skeleton->getName(), "ground skeleton");
#endif
}

//==============================================================================
TEST(ReadUnit, InfersFormatFromMjcfFileExtension)
{
  const auto skeleton = io::readSkeleton("/nonexistent/model.mjcf");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForNoExtensionFile)
{
  const auto skeleton = io::readSkeleton("/some/path/file_without_ext");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, ReturnsNullForDotBeforeSlash)
{
  const auto skeleton = io::readSkeleton("/some/path.dir/no_ext");
  EXPECT_EQ(skeleton, nullptr);
}

//==============================================================================
TEST(ReadUnit, PassesExplicitRetrieverThrough)
{
  auto retriever = std::make_shared<common::LocalResourceRetriever>();
  io::ReadOptions options;
  options.resourceRetriever = retriever;
  options.format = io::ModelFormat::Sdf;
  const auto skeleton = io::readSkeleton("/nonexistent/robot.sdf", options);
  EXPECT_EQ(skeleton, nullptr);
}
