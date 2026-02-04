/*
 * Compatibility header for the legacy dart::optimizer namespace.
 * Use <dart/math/optimization/All.hpp> instead.
 */

#pragma once

#include <dart/common/deprecated.hpp>

#ifndef DART_SUPPRESS_OPTIMIZER_DEPRECATED_HEADER_WARNING
  #pragma message("dart/optimizer/all.hpp is deprecated; include "
                "<dart/math/optimization/All.hpp> instead.")
#endif

#include <dart/math/optimization/All.hpp>
