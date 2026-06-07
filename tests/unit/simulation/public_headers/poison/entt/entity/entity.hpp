// Copyright (c) 2011, The DART development contributors
// All rights reserved.
//
// The list of contributors can be found at:
//   https://github.com/dartsim/dart/blob/main/LICENSE
//
// This file is provided under the "BSD-style" License.
//
// Poison header for the public-header self-containment smoke test. See
// entt/entt.hpp in this directory for the rationale.
#error                                                                         \
    "EnTT leaked into the dart::simulation public header surface. A promoted public header (transitively) includes <entt/entity/entity.hpp>; move that include behind a detail/internal header. See tests/unit/simulation/public_headers and scripts/audit_dart7_promotion_surface.py --strict."
