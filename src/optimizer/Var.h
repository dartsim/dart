/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_VAR_H
#define OPTIMIZER_VAR_H

namespace optimizer {
    struct Var {
    public:
        Var(double val, double upper, double lower);
        void setWeight(double weight);
    public:
        double mVal;
        double mUpper;
        double mLower;
        double mWeight;
    };
    
} // namespace optimizer

#endif // #ifndef OPTIMIZER_VAR_H

