/*
 * Compatibility header for the legacy dart::optimizer namespace.
 * Use <dart/math/optimization/All.hpp> instead.
 */

#pragma once

#include <dart/common/Deprecated.hpp>

#pragma message("dart/optimizer/All.hpp is deprecated; include "
                "<dart/math/optimization/All.hpp> instead.")

#include <dart/math/optimization/All.hpp>
