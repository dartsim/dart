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
	
        void add(Constraint *_newobj);
        void clear();
        int remove(Constraint *_obj);
        int isInBox(Constraint *_obj);

        int getNumConstraints() const { return mObjectives.size(); }
        Constraint * getConstraint(int index) const { return mObjectives[index]; }
	
        void evalObj();
        void evalObjGrad();
        void evalObjHess();
	
        //Must be called befob re using Constraints
        void setNumDofs(int _numdofs);
        void reallocateMem();

        int mNumDofs; //number of Model DOFs
        std::vector<Constraint *> mObjectives;
        double mObj;
        std::vector<double> mObjGrad;
        Eigen::MatrixXd mObjHess;
    };

} // namespace optimizer

#endif // #ifndef OPTIMIZER_OBJECTIVE_BOX_H

