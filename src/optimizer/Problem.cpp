/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "Problem.h"

#include <iostream>
using namespace std;

#include "Var.h"
#include "Constraint.h"
#include "ConstraintBox.h"
#include "ObjectiveBox.h"


namespace optimizer {

    Problem::Problem()
        : mConBox(NULL), mObjBox(NULL) {
        mVariables.clear();
    }
    
    Problem::~Problem() {
        if (mConBox) {
            delete mConBox;
        }

        if (mObjBox) {
            delete mObjBox;
        }

        for (unsigned int i = 0; i < mVariables.size(); ++i) {
            delete mVariables[i];
        }
        mVariables.clear();
    }

    void Problem::update(double* coefs) {
    }

    void Problem::addVariable(double value, double lower, double upper) {
        Var* var = new Var(value, lower, upper);
        mVariables.push_back(var);
    }
    
    void Problem::createBoxes() {
        mConBox = new ConstraintBox(this->mVariables.size());
        mObjBox = new ObjectiveBox(this->mVariables.size());
    }
    
    ConstraintBox* Problem::conBox() const {
        return mConBox;
    }

    ObjectiveBox* Problem::objBox() const {
        return mObjBox;
    }

    std::vector<Var *>& Problem::vars() {
        return mVariables;
    }

} // namespace optimizer
