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

