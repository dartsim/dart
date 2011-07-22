/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "ObjectiveBox.h"
#include "Constraint.h"
using namespace Eigen;
#include <iostream>
using namespace std;

namespace optimizer {

    ObjectiveBox::ObjectiveBox(int numDofs) {
        setNumDofs(numDofs);
        mObj = 0.0;
    }
    
    ObjectiveBox::~ObjectiveBox() {
        clear();
    }

    void ObjectiveBox::add(Constraint *newObjective) {
        mObjectives.push_back(newObjective);
    }

    void ObjectiveBox::clear() {
        mObjectives.clear();
    }

    int ObjectiveBox::remove(Constraint *obj) {
        int index = -1;
        for(unsigned int i = 0; i < mObjectives.size(); i++){
            if(mObjectives[i] == obj){
                index = i;
                break;
            }
        }

        if(index == -1) {
            return index;
        }

        //deallocate memory for Constraint
        mObjectives.erase(mObjectives.begin() + index);

        return index;
    }

    int ObjectiveBox::isInBox(Constraint *testObj)  {
        for(unsigned int i = 0; i < mObjectives.size(); i++) {
            if(mObjectives[i] == testObj)
                return i;
        }
        return -1;
    }

    void ObjectiveBox::setNumDofs(int numDofs) {

        mNumDofs = numDofs;
        mObj = 0;
        mObjGrad.resize(numDofs);

        for(int i = 0; i < numDofs; i++) {
            mObjGrad[i] = 0.0;
        }
    }

    void ObjectiveBox::evalObjGrad() {
        for(unsigned int i = 0; i < mObjectives.size(); i++) {
            if(mObjectives[i]->mActive){
                mObjectives[i]->fillObjGrad(mObjGrad);
            }
        }
    }

    void ObjectiveBox::evalObjHess() {
    }

    void ObjectiveBox::evalObj() {
        mObj = 0;
        for(unsigned int i = 0; i < mObjectives.size(); i++)
            if(mObjectives[i]->mActive)
                mObj += mObjectives[i]->evalObj();
    }

    void ObjectiveBox::reallocateMem() {
        for(unsigned int i = 0; i < mObjectives.size(); i++)
            mObjectives[i]->allocateMem();
    }

    
} // namespace optimizer
