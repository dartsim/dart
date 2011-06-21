/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "Var.h"
#include <glog/logging.h>

namespace optimizer {

    Var::Var(double val, double lower, double upper)
        :mVal(val), mLower(lower), mUpper(upper), mWeight(1.0) {
        CHECK_LT(mLower, mUpper);
    }

    void Var::setWeight(double weight) {
        mWeight = weight;
        mVal *= weight;
        mLower *= weight;
        mUpper *= weight;
    }

} // namespace optimizer
