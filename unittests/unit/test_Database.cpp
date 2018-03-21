/*
 * Copyright (c) 2011-2018, The DART development contributors
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

#include "TestHelpers.hpp"

#include "dart/utils/OnlineDatabase.hpp"

using namespace dart;

#define DART_DATABASE_SERVER_URI                                               \
  "https://raw.githubusercontent.com/dartsim/data/test/"
#define DART_BASE_URI "dart://dartsim/data/"

//==============================================================================
TEST(Database, Basic)
{
  std::string outputPath = getenv("HOME");
  outputPath += "/.dart/models";

  auto database = std::make_shared<utils::OnlineDatabase>(
      DART_DATABASE_SERVER_URI, DART_BASE_URI, outputPath);

  std::cout << "Uri: " << database->getServerUri() << std::endl;
}

//==============================================================================
TEST(Database, HasModel)
{
  std::string outputPath = getenv("HOME");
  outputPath += "/.dart/models";

  auto database = std::make_shared<utils::OnlineDatabase>(
      DART_DATABASE_SERVER_URI, DART_BASE_URI, outputPath);

  EXPECT_FALSE(database->hasDataset("model://authority/path/filename"));
  EXPECT_TRUE(database->hasDataset("dart://dartsim/data/empty"));

  auto modelFile = database->getDataPath("dart://dartsim/data/empty");
  std::cout << modelFile << std::endl;

  std::cout << database->getDatasetPath("dart://dartsim/data/empty")
            << std::endl;
  std::cout << database->getDataPath("dart://dartsim/data/empty/model.skel")
            << std::endl;
}
