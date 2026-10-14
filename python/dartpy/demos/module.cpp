// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <dart/config.hpp>

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

namespace dart::python {

void bind_soft_worm(nb::module_& m);
void bind_world_history(nb::module_& m);
void bind_adaptive_soft_contact(nb::module_& m);
void bind_wam_ikfast(nb::module_& m);
#if HAVE_OCTOMAP
void bind_point_cloud_sensor(nb::module_& m);
#endif
#ifdef DARTPY_HAS_SIMBICON_DEMOS
void bind_simbicon(nb::module_& m);
void bind_soft_foot_simbicon(nb::module_& m);
#endif

void bind_demos(nb::module_& m)
{
  auto demos = m.def_submodule(
      "_demos",
      "Private demo models and scene helpers; outside the SemVer contract.");
  bind_soft_worm(demos);
  bind_world_history(demos);
  bind_adaptive_soft_contact(demos);
  bind_wam_ikfast(demos);
#if HAVE_OCTOMAP
  bind_point_cloud_sensor(demos);
#endif
#ifdef DARTPY_HAS_SIMBICON_DEMOS
  bind_simbicon(demos);
  bind_soft_foot_simbicon(demos);
#endif
}

} // namespace dart::python
