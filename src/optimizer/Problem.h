/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_PROBLEM_H
#define OPTIMIZER_PROBLEM_H

#include <vector>

namespace optimizer {
    class Var;
    class ConstraintBox;
    class ObjectiveBox;
    
    class Problem {
    public:
        Problem();
        virtual ~Problem();
        virtual void update(double* coefs);

        int getNumVariables() const { return mVariables.size(); }
        void addVariable(double value, double lower, double upper);
        void createBoxes();

        ConstraintBox* conBox() const;
        ObjectiveBox* objBox() const;
        std::vector<Var *>& vars();

    private:
        ConstraintBox* mConBox;
        ObjectiveBox* mObjBox;
        std::vector<Var *> mVariables;
        
    };
} // namespace optimizer

#endif // #ifndef OPTIMIZER_PROBLEM_H

