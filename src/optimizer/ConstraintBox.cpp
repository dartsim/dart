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
        mNumTotalRows=0;
        setNumDofs(numDofs);
    }

    ConstraintBox::~ConstraintBox() {
        clear();
    }

    void ConstraintBox::add(Constraint *newConstraint) {
        mConstraints.push_back(newConstraint);
        mNumTotalRows += newConstraint->mNumRows;

        for(int i = 0; i < newConstraint->mNumRows; i++) {
            mCon.push_back(0.0);
        }

        for(int j = 0; j < newConstraint->mNumRows; j++){
            std::vector<double> *val = new std::vector<double>;
            val->resize(mNumDofs);
            for(unsigned int x = 0; x < val->size(); x++)
                (*val)[x] = 0.0;
            mJac.push_back(val);

            std::vector<bool> *val2 = new std::vector<bool>;
            val2->resize(mNumDofs);
            for(unsigned int x = 0; x < val2->size(); x++)
                (*val2)[x] = 0;
            mJacMap.push_back(val2);
        }
    }

    void ConstraintBox::clear()  {
        int count = 0;
        for(unsigned int i=0; i<mConstraints.size(); i++){
            for(int j=0; j<mConstraints[i]->mNumRows; j++){
                delete mJac[count];
                delete mJacMap[count++];
            }
        }
        mConstraints.clear();
        mJac.clear();
        mJacMap.clear();
        mCon.clear();

        mNumTotalRows = 0;
    }

    int ConstraintBox::remove(Constraint *target) {
        unsigned int nConstr = mConstraints.size();
        int index = -1;
        int count = 0;
        for(unsigned int i = 0; i < nConstr; i++){
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
        mCon.erase(mCon.begin() + count, mCon.begin() + count + length);
        for(int j = 0; j < length; j++){
            delete mJac[count+j];
            delete mJacMap[count+j];
        }

        mNumTotalRows -= length ;
        mJac.erase(mJac.begin() + count, mJac.begin() + count + length);
        mJacMap.erase(mJacMap.begin() + count, mJacMap.begin() + count + length);

        return index;
    }

    int ConstraintBox::isInBox(Constraint *testConstraint) {
        for(unsigned int i = 0; i < mConstraints.size(); i++){
            if(mConstraints[i] == testConstraint)
                return i;
        }
        return -1;
    }

    void ConstraintBox::evalJac() {
        int count = 0;
        for(unsigned int i = 0; i < mConstraints.size(); i++){
            if(mConstraints[i]->mActive)
                mConstraints[i]->fillJac(&mJac, &mJacMap, count);
            count += mConstraints[i]->mNumRows;
        }
    }

    void ConstraintBox::evalCon() {
        int count = 0;
        for(unsigned int i = 0; i < mConstraints.size(); i++){
            if(!mConstraints[i]->mActive){
                count += mConstraints[i]->mNumRows;
                continue;
            }
            VectorXd constraintVal = mConstraints[i]->evalCon();
            unsigned int n = constraintVal.size();

            for(unsigned int j = 0; j < n; j++) {
                mCon[count++] = constraintVal[j];
            }
        }
    }

    void ConstraintBox::setNumDofs(int numDofs) {
        mNumDofs = numDofs;
        for(int i = 0; i < mNumTotalRows; i++){
            if(mJac[i]->size()!=mNumDofs){
                mJac[i]->resize(numDofs);
                mJacMap[i]->resize(numDofs);
                for(int j = 0; j < numDofs; j++){
                    mJac[i]->at(j) = 0.0;
                    mJacMap[i]->at(j) = 0;
                }
            }
        }
    }

    void ConstraintBox::reallocateMem() {
        for(unsigned int i = 0; i < mConstraints.size(); i++) {
            mConstraints[i]->allocateMem();
        }
    }


} // namespace optimizer

