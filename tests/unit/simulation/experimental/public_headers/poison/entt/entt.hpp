// Copyright (c) 2011, The DART development contributors
// All rights reserved.
//
// The list of contributors can be found at:
//   https://github.com/dartsim/dart/blob/main/LICENSE
//
// This file is provided under the "BSD-style" License.
//
// Poison header for the public-header self-containment smoke test.
//
// The smoke target adds this directory to its include path with the HIGHEST
// priority (BEFORE), so any `#include <entt/...>` reachable from a promoted
// public header resolves to this file first -- before the real EnTT headers in
// $CONDA_PREFIX/include -- and fails the build with the diagnostic below. This
// makes the guard robust even though EnTT lives in the same conda include
// directory as Eigen/spdlog (which the public surface legitimately uses).
#error                                                                         \
    "EnTT leaked into the dart::simulation::experimental public header surface. A promoted public header (transitively) includes <entt/...>; move that include behind a detail/internal header. See tests/unit/simulation/experimental/public_headers and scripts/audit_dart7_promotion_surface.py --strict."
