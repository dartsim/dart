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

#include "dart/io/usd/usd_parser.hpp"

#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/logging.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/skeleton.hpp"

#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xformable.h>

#include <exception>
#include <memory>
#include <string>

namespace dart::io::usd {

namespace {

common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& retrieverOrNullptr)
{
  if (retrieverOrNullptr) {
    return retrieverOrNullptr;
  }
  return std::make_shared<common::LocalResourceRetriever>();
}

} // namespace

//==============================================================================
dynamics::SkeletonPtr UsdParser::readSkeleton(
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& resourceRetriever)
{
  const auto retriever = getRetriever(resourceRetriever);

  std::string content;
  try {
    content = retriever->readAll(uri);
  } catch (const std::exception& e) {
    DART_ERROR(
        "[dart::io::usd::UsdParser] Failed reading [{}]: {}",
        uri.toString(),
        e.what());
    return nullptr;
  }

  // Import the stage from memory so the DART resource retriever stays the single
  // source of truth for resolving the URI (works for .usda text layers).
  const pxr::SdfLayerRefPtr layer = pxr::SdfLayer::CreateAnonymous(".usda");
  if (!layer || !layer->ImportFromString(content)) {
    DART_ERROR(
        "[dart::io::usd::UsdParser] Failed parsing USD layer [{}]",
        uri.toString());
    return nullptr;
  }

  const pxr::UsdStageRefPtr stage = pxr::UsdStage::Open(layer);
  if (!stage) {
    DART_ERROR(
        "[dart::io::usd::UsdParser] Failed opening USD stage [{}]",
        uri.toString());
    return nullptr;
  }

  // Pick the prim that anchors the skeleton: the stage default prim when set,
  // otherwise the first Xformable prim in traversal order.
  pxr::UsdPrim rootPrim = stage->GetDefaultPrim();
  if (!rootPrim) {
    for (const pxr::UsdPrim& prim : stage->Traverse()) {
      if (prim.IsA<pxr::UsdGeomXformable>()) {
        rootPrim = prim;
        break;
      }
    }
  }

  if (!rootPrim) {
    DART_ERROR(
        "[dart::io::usd::UsdParser] No mappable prim found in USD stage [{}]",
        uri.toString());
    return nullptr;
  }

  const std::string name = rootPrim.GetName().GetString();
  auto skeleton = dynamics::Skeleton::create(name);

  // Phase 1: a single free-floating link for the root prim. Joint, inertia, and
  // collision-shape mapping from USDPhysics is later-phase work.
  auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(nullptr);
  pair.second->setName(name);

  return skeleton;
}

} // namespace dart::io::usd
