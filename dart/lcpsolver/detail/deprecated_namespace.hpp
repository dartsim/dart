#ifndef DART_LCPSOLVER_DETAIL_DEPRECATEDNAMESPACE_HPP_
#define DART_LCPSOLVER_DETAIL_DEPRECATEDNAMESPACE_HPP_

#include "dart/common/deprecated.hpp"

#if !defined(DART_LCPSOLVER_SUPPRESS_DEPRECATED_HEADER_WARNING)
  #if defined(_MSC_VER)
    #pragma message(                                                           \
        __FILE__                                                               \
        " is deprecated and will be removed in DART 8.0; include "             \
        "dart/math/lcp equivalents instead")
  #else
    #warning                                                                   \
        "dart/lcpsolver headers are deprecated and will be removed in DART 8.0; include dart/math/lcp equivalents instead"
  #endif
#endif

namespace dart {
namespace math {
namespace [[deprecated(
    "dart::math::lcp is deprecated; include dart/math/lcp headers and use "
    "dart::math instead. Will be removed in DART 8.0.")]] lcp {
using namespace ::dart::math;
} // namespace lcp
} // namespace math
namespace [[deprecated(
    "dart::lcpsolver is deprecated; include dart/math/lcp "
    "headers and use dart::math instead. Will be removed in "
    "DART 8.0.")]] lcpsolver {
using namespace math::lcp;
} // namespace lcpsolver
} // namespace dart

#endif // DART_LCPSOLVER_DETAIL_DEPRECATEDNAMESPACE_HPP_
