#include "Var.h"

namespace optimizer {

    Var::Var(double val, double upper, double lower)
        :mVal(val), mUpper(upper), mLower(lower), mWeight(1.0) {
    }

    void Var::setWeight(double weight) {
        mWeight = weight;
        mVal *= weight;
        mUpper *= weight;
        mLower *= weight;
    }

} // namespace optimizer
