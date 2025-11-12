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

#ifndef DART_UTILS_SCENEPARSER_HPP_
#define DART_UTILS_SCENEPARSER_HPP_

#include <dart/utils/FileContents.hpp>
#include <dart/utils/VskParser.hpp>
#include <dart/utils/mjcf/MjcfParser.hpp>
#include <dart/utils/sdf/SdfParser.hpp>

#include <dart/common/ResourceRetriever.hpp>
#include <dart/common/Uri.hpp>

#if defined(DART_SCENEPARSER_HAS_URDF)
  #include <dart/utils/urdf/DartLoader.hpp>
#endif

namespace dart {
namespace utils {
namespace SceneParser {

/// Supported scene description formats.
enum class Format
{
  Auto,
  Skel,
  Sdf,
  Urdf,
  Mjcf,
  Vsk
};

/// Options for the unified parser. Parser-specific options can be customized
/// through the dedicated fields.
struct Options
{
  /// Hint for the expected format. `Auto` (default) will attempt to detect the
  /// format from the URI extension and fall back to other parsers if needed.
  Format mFormatHint = Format::Auto;

  /// Resource retriever shared across all format-specific parsers unless they
  /// already provide one through their own options.
  common::ResourceRetrieverPtr mResourceRetriever = nullptr;

  /// Options forwarded to SdfParser.
  utils::SdfParser::Options mSdfOptions = utils::SdfParser::Options();

#if defined(DART_SCENEPARSER_HAS_URDF)
  /// Options forwarded to DartLoader (URDF).
  utils::DartLoader::Options mUrdfOptions = utils::DartLoader::Options();
#endif

  /// Options forwarded to MjcfParser.
  utils::MjcfParser::Options mMjcfOptions = utils::MjcfParser::Options();

  /// Options forwarded to VskParser.
  utils::VskParser::Options mVskOptions = utils::VskParser::Options();
};

/// Parses the file referenced by \p uri using the requested options and returns
/// every discovered object.
FileContents readFile(
    const common::Uri& uri, const Options& options = Options());

} // namespace SceneParser
} // namespace utils
} // namespace dart

#endif // DART_UTILS_SCENEPARSER_HPP_
