/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_CONSTRAINT_H
#define OPTIMIZER_CONSTRAINT_H

#include <vector>
#include <Eigen/Dense>
#include "OptimizerArrayTypes.h"

namespace optimizer {
    class Var;


    class Constraint {
    public:
        Constraint(std::vector<Var *>& var);
        virtual ~Constraint() {}

    public:
        virtual Eigen::VectorXd EvalC() = 0;
        virtual double EvalG();

        virtual void FillJ(VVD, int index) {}
        virtual void FillJ(VVD, VVB, int index) {}
        virtual void FilldG(std::vector<double>& dG){}
        virtual void FillddG(Eigen::MatrixXd& ddG, int index) {}
	
        virtual void AllocateMem() {}
        virtual void UpdateParams() {}

        std::vector<Var *>& mVariables;
        int mIndex; // index of the variable this constraint is concerning
        int mNumRows; // number of rows of this constraint
        bool mSlack; // [-1e3, 1e3]
        Eigen::VectorXd mConstTerm; // constraint value, usually zero for equality constraint
        bool mActive; // is this constraint active
        Eigen::VectorXd mWeight; 
        std::vector<int> mConfigIndecies;
        int mEquality; // 1: >=0; -1: <=0; 0: =0
        Eigen::VectorXd mCompletion;
    };

} // namespace optimizer

#endif // #ifndef OPTIMIZER_CONSTRAINT_H

