#ifndef DART_LCPSOLVER_DANTZIG_LCP_H_
#define DART_LCPSOLVER_DANTZIG_LCP_H_

#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"

namespace dart::lcpsolver::dantzig {

inline bool dSolveLCP(
    int n,
    dReal* A,
    dReal* x,
    dReal* b,
    dReal* w,
    int nub,
    dReal* lo,
    dReal* hi,
    int* findex,
    bool earlyTermination = false)
{
  return SolveLCP<dReal>(n, A, x, b, w, nub, lo, hi, findex, earlyTermination);
}

} // namespace dart::lcpsolver::dantzig

#endif // DART_LCPSOLVER_DANTZIG_LCP_H_
