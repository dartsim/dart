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
#include "dart/io/CompositeResourceRetriever.hpp"
#include "TestHelpers.hpp"

using dart::common::Uri;
using dart::common::Resource;
using dart::common::ResourcePtr;
using dart::common::ResourceRetriever;
using dart::io::CompositeResourceRetriever;

TEST(CompositeResourceRetriever, exists_NothingRegistered_ReturnsFalse)
{
  CompositeResourceRetriever retriever;
  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));
}

TEST(CompositeResourceRetriever, exists_AllRetrieversFail_ReturnsFalse)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);

  EXPECT_FALSE(retriever.exists(Uri::createFromString("package://test/foo")));

  EXPECT_TRUE(retriever1->mRetrieve.empty());
  ASSERT_EQ(1u, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mRetrieve.empty());
  ASSERT_EQ(1u, retriever2->mExists.size());
  EXPECT_EQ("package://test/foo", retriever2->mExists.front());
}

TEST(CompositeResourceRetriever, exists_CompositeResourceRetrieverSucceeds_ReturnsTrue)
{
  auto retriever1 = std::make_shared<PresentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever2));
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(retriever.exists(Uri::createFromString("package://test/foo")));

  EXPECT_TRUE(retriever1->mRetrieve.empty());
  ASSERT_EQ(1u, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  EXPECT_TRUE(retriever2->mRetrieve.empty());
  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(CompositeResourceRetriever, exists_DefaultResourceRetrieverSucceeds_ReturnsTrue)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<PresentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(retriever.exists(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(retriever1->mRetrieve.empty());
  ASSERT_EQ(1u, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever2->mRetrieve.empty());
  ASSERT_EQ(1u, retriever1->mExists.size());
  EXPECT_EQ("package://test/foo", retriever1->mExists.front());

  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(CompositeResourceRetriever, getFilePath_NothingRegistered_ReturnsEmptyString)
{
  CompositeResourceRetriever retriever;
  EXPECT_EQ(retriever.getFilePath(Uri::createFromString("package://test/foo")), "");
}

TEST(CompositeResourceRetriever, getFilePath_AllRetrieversFail_ReturnsFalse)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);

  EXPECT_EQ(retriever.getFilePath(Uri::createFromString("package://test/foo")), "");

  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1u, retriever1->mGetFilePath.size());
  EXPECT_EQ("package://test/foo", retriever1->mGetFilePath.front());
  EXPECT_TRUE(retriever1->mRetrieve.empty());

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1u, retriever2->mGetFilePath.size());
  EXPECT_EQ("package://test/foo", retriever2->mGetFilePath.front());
  EXPECT_TRUE(retriever2->mRetrieve.empty());
}

TEST(CompositeResourceRetriever, getFilePath_CompositeResourceRetrieverSucceeds_ReturnsFilePath)
{
  auto retriever1 = std::make_shared<PresentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever2));
  retriever.addDefaultRetriever(retriever3);

  EXPECT_EQ(retriever.getFilePath(Uri::createFromString("package://test/foo")), "package://test/foo");

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1u, retriever1->mGetFilePath.size());
  EXPECT_EQ("package://test/foo", retriever1->mGetFilePath.front());
  EXPECT_TRUE(retriever1->mRetrieve.empty());

  EXPECT_TRUE(retriever2->mExists.empty());
  EXPECT_TRUE(retriever2->mGetFilePath.empty());
  EXPECT_TRUE(retriever2->mRetrieve.empty());
  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mGetFilePath.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(CompositeResourceRetriever, getFilePath_DefaultResourceRetrieverSucceeds_ReturnsFilePath)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<PresentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);
  retriever.addDefaultRetriever(retriever3);

  EXPECT_EQ(retriever.getFilePath(Uri::createFromString("package://test/foo")), "package://test/foo");

  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1u, retriever1->mGetFilePath.size());
  EXPECT_EQ("package://test/foo", retriever1->mGetFilePath.front());
  EXPECT_TRUE(retriever1->mRetrieve.empty());

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1u, retriever2->mGetFilePath.size());
  EXPECT_EQ("package://test/foo", retriever2->mGetFilePath.front());
  EXPECT_TRUE(retriever2->mRetrieve.empty());

  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mGetFilePath.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(CompositeResourceRetriever, retrieve_NothingRegistered_ReturnsNull)
{
  CompositeResourceRetriever retriever;
  EXPECT_EQ(nullptr, retriever.retrieve(Uri::createFromString("package://test/foo")));
}

TEST(CompositeResourceRetriever, retrieve_AllRetrieversFail_ReturnsNull)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);

  EXPECT_EQ(nullptr,
            retriever.retrieve(Uri::createFromString("package://test/foo")));

  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1u, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mRetrieve.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1u, retriever2->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever2->mRetrieve.front());
}

TEST(CompositeResourceRetriever, retrieve_CompositeResourceRetrieverSucceeds_ReturnsNonNull)
{
  auto retriever1 = std::make_shared<PresentResourceRetriever>();
  auto retriever2 = std::make_shared<AbsentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever2));
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(nullptr != retriever.retrieve(Uri::createFromString("package://test/foo")));

  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1u, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mRetrieve.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  EXPECT_TRUE(retriever2->mRetrieve.empty());
  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}

TEST(CompositeResourceRetriever, retrieve_DefaultResourceRetrieverSucceeds_ReturnsNonNull)
{
  auto retriever1 = std::make_shared<AbsentResourceRetriever>();
  auto retriever2 = std::make_shared<PresentResourceRetriever>();
  auto retriever3 = std::make_shared<AbsentResourceRetriever>();
  CompositeResourceRetriever retriever;

  EXPECT_TRUE(retriever.addSchemaRetriever("package", retriever1));
  retriever.addDefaultRetriever(retriever2);
  retriever.addDefaultRetriever(retriever3);

  EXPECT_TRUE(nullptr != retriever.retrieve(Uri::createFromString("package://test/foo")));
  EXPECT_TRUE(retriever1->mExists.empty());
  ASSERT_EQ(1u, retriever1->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever1->mRetrieve.front());

  EXPECT_TRUE(retriever2->mExists.empty());
  ASSERT_EQ(1u, retriever2->mRetrieve.size());
  EXPECT_EQ("package://test/foo", retriever2->mRetrieve.front());

  EXPECT_TRUE(retriever3->mExists.empty());
  EXPECT_TRUE(retriever3->mRetrieve.empty());
}
