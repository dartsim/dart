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

#include "demo_models/HeightmapModel.hpp"

#include <limits>

namespace dart::python {

void bind_heightmap_model(nb::module_& demos)
{
  demos.def(
      "create_heightmap_shape",
      [](long long xResolution,
         long long yResolution,
         float xSize,
         float ySize,
         float zMin,
         float zMax) {
        const auto maxResolution = std::numeric_limits<std::size_t>::max();
        if (xResolution <= 0 || yResolution <= 0
            || static_cast<unsigned long long>(xResolution) > maxResolution
            || static_cast<unsigned long long>(yResolution) > maxResolution)
          throw nb::value_error(
              "Heightmap resolution must be positive and fit size_t");
        return dart_demos::createHeightmapShape(
            static_cast<std::size_t>(xResolution),
            static_cast<std::size_t>(yResolution),
            xSize,
            ySize,
            zMin,
            zMax);
      },
      nb::arg("x_resolution"),
      nb::arg("y_resolution"),
      nb::arg("x_size"),
      nb::arg("y_size"),
      nb::arg("z_min"),
      nb::arg("z_max"),
      "Create native float-random terrain; sizes must be positive and finite, "
      "and finite heights must be ordered.");
}

} // namespace dart::python
