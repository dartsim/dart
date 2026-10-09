// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/utils/mjcf/MjcfParser.hpp>

#include <dart/common/ResourceRetriever.hpp>

#include <string>

namespace dart {
namespace python {

void MjcfParser(nb::module_& m)
{
  auto sm = m.def_submodule("MjcfParser");

  dartnb::dart_class<utils::MjcfParser::Options>(sm, "Options")
      .def(
          dartnb::init<
              const common::ResourceRetrieverPtr&,
              const std::string&,
              const std::string&>(),
          nb::arg("resourceRetretrieverOrNullptrriever").none() = nullptr,
          nb::arg("geomSkeletonNamePrefix") = "__geom_skel__",
          nb::arg("siteSkeletonNamePrefix") = "__site_skel__")
      .def_rw(
          "mRetriever",
          &utils::MjcfParser::Options::mRetriever,
          dartnb::setterArgument(&utils::MjcfParser::Options::mRetriever))
      .def_rw(
          "mGeomSkeletonNamePrefix",
          &utils::MjcfParser::Options::mGeomSkeletonNamePrefix,
          dartnb::setterArgument(
              &utils::MjcfParser::Options::mGeomSkeletonNamePrefix))
      .def_rw(
          "mSiteSkeletonNamePrefix",
          &utils::MjcfParser::Options::mSiteSkeletonNamePrefix,
          dartnb::setterArgument(
              &utils::MjcfParser::Options::mSiteSkeletonNamePrefix));

  // resource retriever APIs
  sm.def(
      "readWorld",
      &utils::MjcfParser::readWorld,
      nb::arg("uri"),
      nb::arg("options") = utils::MjcfParser::Options());
}

} // namespace python
} // namespace dart
