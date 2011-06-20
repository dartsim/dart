/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "ConstraintBox.h"
#include "Constraint.h"

#include <Eigen/Dense>
using namespace Eigen;

namespace optimizer {

    ConstraintBox::ConstraintBox(int numDofs) {
        mNumConstrs=0;
        SetNumDofs(numDofs);
    }

    ConstraintBox::~ConstraintBox() {
        Clear();
    }

    void ConstraintBox::Add(Constraint *newConstraint) {
        mConstraints.push_back(newConstraint);
        mNumConstrs += newConstraint->mNumRows;

        for(int i = 0; i < newConstraint->mNumRows; i++) {
            mC.push_back(0.0);
        }

        for(int j = 0; j < newConstraint->mNumRows; j++){
            std::vector<double> *val = new std::vector<double>;
            val->resize(mNumDofs);
            for(int x = 0; x < val->size(); x++)
                (*val)[x] = 0.0;
            mJ.push_back(val);

            std::vector<bool> *val2 = new std::vector<bool>;
            val2->resize(mNumDofs);
            for(int x = 0; x < val2->size(); x++)
                (*val2)[x] = 0;
            mJMap.push_back(val2);
        }
    }

    void ConstraintBox::Clear()  {
        int count = 0;
        for(int i=0; i<mConstraints.size(); i++){
            for(int j=0; j<mConstraints[i]->mNumRows; j++){
                delete mJ[count];
                delete mJMap[count++];
            }
        }
        mConstraints.clear();
        mJ.clear();
        mJMap.clear();
        mC.clear();

        mNumConstrs = 0;
    }

    int ConstraintBox::TakeOut(Constraint *target) {
        int nConstr = mConstraints.size();
        int index = -1;
        int count = 0;
        for(int i = 0; i < nConstr; i++){
            if(mConstraints[i] == target){
                index = i;
                break;
            }
            count += mConstraints.at(i)->mNumRows;
        }

        if(index == -1)
            return index;

        int length = target->mNumRows;

        //deallocate memory for Constraint
        mConstraints.erase(mConstraints.begin() + index);
        mC.erase(mC.begin() + count, mC.begin() + count + length);
        for(int j = 0; j < length; j++){
            delete mJ[count+j];
            delete mJMap[count+j];
        }

        mNumConstrs -= length ;
        mJ.erase(mJ.begin() + count, mJ.begin() + count + length);
        mJMap.erase(mJMap.begin() + count, mJMap.begin() + count + length);

        return index;
    }

    int ConstraintBox::IsInBox(Constraint *testConstraint) {
        for(int i = 0; i < mConstraints.size(); i++){
            if(mConstraints[i] == testConstraint)
                return i;
        }
        return -1;
    }

    void ConstraintBox::EvalJ() {
        int count = 0;
        for(int i = 0; i < mConstraints.size(); i++){
            if(mConstraints[i]->mActive)
                mConstraints[i]->FillJ(&mJ, &mJMap, count);
            count += mConstraints[i]->mNumRows;
        }
    }

    void ConstraintBox::EvalC() {
        int count = 0;
        for(int i = 0; i < mConstraints.size(); i++){
            if(!mConstraints[i]->mActive){
                count += mConstraints[i]->mNumRows;
                continue;
            }
            VectorXd constraintVal = mConstraints[i]->EvalC();
            int n = constraintVal.size();

            for(int j = 0; j < n; j++) {
                mC[count++] = constraintVal[j];
            }
        }
    }

    void ConstraintBox::SetNumDofs(int numDofs) {
        mNumDofs = numDofs;
        for(int i = 0; i < mNumConstrs; i++){
            if(mJ[i]->size()!=mNumDofs){
                mJ[i]->resize(numDofs);
                mJMap[i]->resize(numDofs);
                for(int j = 0; j < numDofs; j++){
                    mJ[i]->at(j) = 0.0;
                    mJMap[i]->at(j) = 0;
                }
            }
        }
    }

    void ConstraintBox::ReallocateMem() {
        for(int i = 0; i < mConstraints.size(); i++) {
            mConstraints[i]->AllocateMem();
        }
    }


} // namespace optimizer

