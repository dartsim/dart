/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#ifndef OPTIMIZER_CONSTRAINT_BOX_H
#define OPTIMIZER_CONSTRAINT_BOX_H

#include <vector>

namespace optimizer {
    class Constraint;

    class ConstraintBox {
    public:
        ConstraintBox(int numDofs);
        virtual ~ConstraintBox();
	
        void Add(Constraint *newConstraint);
        void Clear();
        int TakeOut(Constraint *target);
        int IsInBox(Constraint *testConstraint);	//return index of component if true

        int getNumConstraints() const { return mConstraints.size(); }
        Constraint * getConstraint(int index) const { return mConstraints[index]; }

        void EvalJ();
        void EvalC();
        void ReallocateMem();

        //Must be called before using ConstraintBox
        int getNumDofs() const { return mNumDofs; }
        void SetNumDofs(int numDofs);
        int getNumTotalRows() const { return mNumTotalRows; }

        int mNumDofs; //number of Model DOFs
        int mNumTotalRows;
        std::vector<Constraint *> mConstraints;
        std::vector<double> mC;
        std::vector< std::vector<double> *> mJ; //Jacobian
        std::vector< std::vector<bool> *> mJMap; //Show nonzero elements of Jacobian
    };
} // namespace optimizer

#endif // #ifndef OPTIMIZER_CONSTRAINT_BOX_H

