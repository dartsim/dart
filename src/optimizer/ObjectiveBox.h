/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_OBJECTIVE_BOX_H
#define OPTIMIZER_OBJECTIVE_BOX_H

#include <vector>
#include <Eigen/Dense>

namespace optimizer {

    class Constraint;

    class ObjectiveBox {
    public:
        ObjectiveBox(int numDofs);
        virtual ~ObjectiveBox();
	
        void Add(Constraint *newObjective);
        void Clear();
        int TakeOut(Constraint *obj);
        int IsInBox(Constraint*);
	
        void EvalG();
        void EvaldG();
        void EvalddG();
	
        //Must be called befob re using Constraints
        void SetNumDofs(int numDofs);
        void ReallocateMem();

        int mNumDofs; //number of Model DOFs
        std::vector<Constraint *> mObjectives;
        double mG;
        std::vector<double> mdG;
        Eigen::MatrixXd mddG;
    };

} // namespace optimizer

#endif // #ifndef OPTIMIZER_OBJECTIVE_BOX_H

