#ifndef DART_LCPSOLVER_DETAIL_DEPRECATEDNAMESPACE_HPP_
#define DART_LCPSOLVER_DETAIL_DEPRECATEDNAMESPACE_HPP_

#include "dart/common/Deprecated.hpp"
#include <dart/common/Namespace.hpp>

#if !defined(DART_LCPSOLVER_SUPPRESS_DEPRECATED_HEADER_WARNING)
  #if defined(_MSC_VER)
    #pragma message(                                                           \
        __FILE__ " is deprecated; include dart/math/lcp equivalents instead")
  #else
    #warning                                                                   \
        "dart/lcpsolver headers are deprecated; include dart/math/lcp equivalents instead"
  #endif
#endif

DART_INLINE_NAMESPACE_BEGIN
namespace math {
namespace lcp {
} // namespace lcp
} // namespace math

namespace
    [[deprecated("dart::lcpsolver is deprecated; include dart/math/lcp headers "
                 "and use dart::math instead")]] lcpsolver {
using namespace math::lcp;
} // namespace lcpsolver
DART_INLINE_NAMESPACE_END

#endif // DART_LCPSOLVER_DETAIL_DEPRECATEDNAMESPACE_HPP_
