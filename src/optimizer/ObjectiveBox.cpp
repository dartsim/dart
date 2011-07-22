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
        SetNumDofs(numDofs);
        mG = 0.0;
    }
    
    ObjectiveBox::~ObjectiveBox() {
        Clear();
    }

    void ObjectiveBox::Add(Constraint *newObjective) {
        mObjectives.push_back(newObjective);
    }

    void ObjectiveBox::Clear() {
        mObjectives.clear();
    }

    int ObjectiveBox::TakeOut(Constraint *obj) {
        int index = -1;
        for(int i = 0; i < mObjectives.size(); i++){
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

    int ObjectiveBox::IsInBox(Constraint *testObj)  {
        for(int i = 0; i < mObjectives.size(); i++) {
            if(mObjectives[i] == testObj)
                return i;
        }
        return -1;
    }

    void ObjectiveBox::SetNumDofs(int numDofs) {

        mNumDofs = numDofs;
        mG = 0;
        mdG.resize(numDofs);

        for(int i = 0; i < numDofs; i++) {
            mdG[i] = 0.0;
        }
    }

    void ObjectiveBox::EvaldG() {
        for(int i = 0; i < mObjectives.size(); i++) {
            if(mObjectives[i]->mActive){
                mObjectives[i]->FilldG(mdG);
            }
        }
    }

    void ObjectiveBox::EvalddG() {
    }

    void ObjectiveBox::EvalG() {
        mG = 0;
        for(int i = 0; i < mObjectives.size(); i++)
            if(mObjectives[i]->mActive)
                mG += mObjectives[i]->EvalG();
    }

    void ObjectiveBox::ReallocateMem() {
        for(int i = 0; i < mObjectives.size(); i++)
            mObjectives[i]->AllocateMem();
    }

    
} // namespace optimizer
