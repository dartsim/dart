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

#ifndef DART_IO_DARTRESOURCERETRIEVER_HPP_
#define DART_IO_DARTRESOURCERETRIEVER_HPP_

#include <unordered_map>
#include <vector>
#include "dart/common/ResourceRetriever.hpp"

namespace dart {
namespace io {

/// Retrieve local resources from sample data files given file URI. The scheme
/// and authority should be "file" and "sample", respectively.
///
/// Example of a sample data URI:
/// @code
/// "dart://sample/skel/shapes.skel"
///                \______________/
///                       |
///            file path with respect to
///            the sample data directory
/// @endcode
///
/// DartResourceRetriever searches files in the following order:
/// 1) Preprocessor, DART_DATA_LOCAL_PATH: Path to the data directory in the
///    source directory (e.g., [DART_SRC_ROOT]/data/).
/// 2) Preprocessor, DART_DATA_GLOBAL_PATH: Path to the data directory installed
///    in a system directory. The location can be varied depending on OS
///    (e.g., Linux: /usr/local/share/doc/dart/data/).
/// 3) environment variable, DART_DATA_PATH: Path to the data directory
///    specified by the user.
class DartResourceRetriever : public common::ResourceRetriever
{
public:
  template <typename... Args>
  static std::shared_ptr<DartResourceRetriever> create(Args&&... args)
  {
    return std::make_shared<DartResourceRetriever>(
          std::forward<Args>(args)...);
  }

  /// Constructor.
  DartResourceRetriever();

  /// Destructor.
  ~DartResourceRetriever() override = default;

  // Documentation inherited.
  bool exists(const common::Uri& uri) override;

  // Documentation inherited.
  common::ResourcePtr retrieve(const common::Uri& uri) override;

  // Documentation inherited.
  std::string getFilePath(const common::Uri& uri) override;

private:

  void addDataDirectory(const std::string& packageDirectory);

  bool resolveDataUri(const common::Uri& uri, std::string& relativePath) const;

private:
  common::ResourceRetrieverPtr mLocalRetriever;

  std::vector<std::string> mDataDirectories;
};

using DartResourceRetrieverPtr = std::shared_ptr<DartResourceRetriever>;

} // namespace io
} // namespace dart

#endif // ifndef DART_IO_DARTRESOURCERETRIEVER_HPP_
