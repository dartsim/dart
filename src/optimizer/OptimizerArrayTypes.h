#ifndef OPTIMIZER_OPTIMIZER_ARRAY_TYPES_H
#define OPTIMIZER_OPTIMIZER_ARRAY_TYPES_H

#include <vector>

namespace optimizer {
    typedef std::vector<std::vector<double> * > * VVD;
    typedef std::vector<std::vector<int> * > * VVI;
    typedef std::vector<std::vector<bool> * > * VVB;
} // namespace optimizer

#endif // #ifndef OPTIMIZER_OPTIMIZER_ARRAY_TYPES_H

