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

#ifndef DART_IO_USD_USD_PARSER_HPP_
#define DART_IO_USD_USD_PARSER_HPP_

#include <dart/dynamics/skeleton.hpp>

#include <dart/common/resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <dart/io/export.hpp>

namespace dart::io::usd {

/// Minimal OpenUSD (pxr) scene loader for the DART 7 model-loading front door.
///
/// Phase 1 maps a single USD primitive (the stage default prim, or the first
/// Xformable prim in traversal order) into a one-link `dynamics::Skeleton` so a
/// `.usd`/`.usda` asset can be added to a DART 7 `World` through
/// `dart::simulation::io::addSkeleton`. Mapping richer USD/USDPhysics structure
/// (joints, inertia, collision shapes) is later-phase work tracked in
/// `docs/dev_tasks/usd_scene_loader/`.
///
/// This type is only compiled when DART is built with DART_BUILD_IO_USD=ON,
/// which requires OpenUSD/pxr. When the toggle is OFF, the front door in
/// `dart/io/read.cpp` returns a "USD support is not available" diagnostic
/// instead of linking against this parser.
class DART_IO_API UsdParser
{
public:
  /// Reads a Skeleton from the USD stage referenced by `uri`.
  ///
  /// @param[in] uri URI of the USD stage to read.
  /// @param[in] resourceRetriever Retriever used to resolve `uri`. A default
  ///   local-file retriever is used when nullptr.
  /// @return The created Skeleton, or nullptr when the stage cannot be opened
  ///   or contains no prim that maps to a link.
  static dynamics::SkeletonPtr readSkeleton(
      const common::Uri& uri,
      const common::ResourceRetrieverPtr& resourceRetriever = nullptr);
};

} // namespace dart::io::usd

#endif // DART_IO_USD_USD_PARSER_HPP_
