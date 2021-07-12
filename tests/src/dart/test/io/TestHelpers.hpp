/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include <vector>

#include "dart/common/common.hpp"
#include "dart/io/io.hpp"

//==============================================================================
struct TestResource : public dart::common::Resource {
  size_t getSize() override {
    return 0;
  }

  size_t tell() override {
    return 0;
  }

  bool seek(ptrdiff_t /*_offset*/, SeekType /*_origin*/) override {
    return false;
  }

  size_t read(void* /*_buffer*/, size_t /*_size*/, size_t /*_count*/) override {
    return 0;
  }
};

//==============================================================================
struct PresentResourceRetriever : public dart::common::ResourceRetriever {
  bool exists(const dart::common::Uri& _uri) override {
    mExists.push_back(_uri.toString());
    return true;
  }

  std::string getFilePath(const dart::common::Uri& _uri) override {
    mGetFilePath.push_back(_uri.toString());
    return _uri.toString();
  }

  dart::common::ResourcePtr retrieve(const dart::common::Uri& _uri) override {
    mRetrieve.push_back(_uri.toString());
    return std::make_shared<TestResource>();
  }

  std::vector<std::string> mExists;
  std::vector<std::string> mGetFilePath;
  std::vector<std::string> mRetrieve;
};

//==============================================================================
struct AbsentResourceRetriever : public dart::common::ResourceRetriever {
  bool exists(const dart::common::Uri& _uri) override {
    mExists.push_back(_uri.toString());
    return false;
  }

  std::string getFilePath(const dart::common::Uri& _uri) override {
    mGetFilePath.push_back(_uri.toString());
    return "";
  }

  dart::common::ResourcePtr retrieve(const dart::common::Uri& _uri) override {
    mRetrieve.push_back(_uri.toString());
    return nullptr;
  }

  std::vector<std::string> mExists;
  std::vector<std::string> mGetFilePath;
  std::vector<std::string> mRetrieve;
};
