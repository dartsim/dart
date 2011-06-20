/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "Problem.h"

using namespace std;

#include "Var.h"
#include "Constraint.h"
#include "ConstraintBox.h"
#include "ObjectiveBox.h"

namespace optimizer {

    Problem::Problem()
        : mConBox(NULL), mObjBox(NULL) {
    }
    
    Problem::~Problem() {
        if (mConBox) {
            delete mConBox;
        }

        if (mObjBox) {
            delete mObjBox;
        }

        for (int i = 0; mVariables.size(); ++i) {
            delete mVariables[i];
        }
        mVariables.clear();
    }

    void Problem::AddVariable(double value, double lower, double upper) {
        Var* var = new Var(value, upper, lower);
        mVariables.push_back(var);
    }
    
    void Problem::createBoxes() {
        mConBox = new ConstraintBox(this->mVariables.size());
        mObjBox = new ObjectiveBox(this->mVariables.size());
    }
    

} // namespace optimizer
