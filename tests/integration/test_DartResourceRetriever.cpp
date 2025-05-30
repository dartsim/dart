/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/config.hpp"
#include "dart/utils/DartResourceRetriever.hpp"

#include <gtest/gtest.h>

#include <iostream>

using namespace dart;

//==============================================================================
TEST(DartResourceRetriever, ExistsAndGetFilePathAndRetrieve)
{
  auto retriever = utils::DartResourceRetriever::create();

  EXPECT_FALSE(retriever->exists("unknown://test"));
  EXPECT_FALSE(retriever->exists("unknown://sample/test"));
  EXPECT_FALSE(retriever->exists("dart://unknown/test"));
  EXPECT_FALSE(retriever->exists("dart://sample/does/not/exist"));
  EXPECT_TRUE(retriever->exists("dart://sample/skel/shapes.skel"));

  EXPECT_EQ(retriever->getFilePath("unknown://test"), "");
  EXPECT_EQ(retriever->getFilePath("unknown://sample/test"), "");
  EXPECT_EQ(retriever->getFilePath("dart://unknown/test"), "");
  EXPECT_EQ(retriever->getFilePath("dart://sample/does/not/exist"), "");
  EXPECT_EQ(
      retriever->getFilePath("dart://sample/skel/shapes.skel"),
      DART_DATA_PATH "skel/shapes.skel");

  EXPECT_EQ(nullptr, retriever->retrieve("unknown://test"));
  EXPECT_EQ(nullptr, retriever->retrieve("unknown://sample/test"));
  EXPECT_EQ(nullptr, retriever->retrieve("dart://unknown/test"));
  EXPECT_EQ(nullptr, retriever->retrieve("dart://sample/does/not/exist"));
  EXPECT_NE(nullptr, retriever->retrieve("dart://sample/skel/shapes.skel"));
}
