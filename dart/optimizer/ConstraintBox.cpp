/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "ConstraintBox.h"
#include "Constraint.h"

#include <Eigen/Dense>

namespace dart {
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

    for (int i = 0; i < newConstraint->mNumRows; i++) {
        mCon.push_back(0.0);
    }

    for (int j = 0; j < newConstraint->mNumRows; j++){
        std::vector<double> *val = new std::vector<double>;
        val->resize(mNumDofs);
        for (unsigned int x = 0; x < val->size(); x++)
            (*val)[x] = 0.0;
        mJac.push_back(val);

        std::vector<bool> *val2 = new std::vector<bool>;
        val2->resize(mNumDofs);
        for (unsigned int x = 0; x < val2->size(); x++)
            (*val2)[x] = 0;
        mJacMap.push_back(val2);
    }
}

void ConstraintBox::clear()  {
    int count = 0;
    for (unsigned int i=0; i<mConstraints.size(); i++){
        for (int j=0; j<mConstraints[i]->mNumRows; j++){
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
    for (unsigned int i = 0; i < nConstr; i++){
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
    for (int j = 0; j < length; j++){
        delete mJac[count+j];
        delete mJacMap[count+j];
    }

    mNumTotalRows -= length ;
    mJac.erase(mJac.begin() + count, mJac.begin() + count + length);
    mJacMap.erase(mJacMap.begin() + count, mJacMap.begin() + count + length);

    return index;
}

int ConstraintBox::isInBox(Constraint *testConstraint) {
    for (unsigned int i = 0; i < mConstraints.size(); i++){
        if(mConstraints[i] == testConstraint)
            return i;
    }
    return -1;
}

void ConstraintBox::evalJac() {
    int count = 0;
    for (unsigned int i = 0; i < mConstraints.size(); i++){
        if(mConstraints[i]->mActive)
            mConstraints[i]->fillJac(&mJac, &mJacMap, count);
        count += mConstraints[i]->mNumRows;
    }
}

void ConstraintBox::evalCon() {
    int count = 0;
    for (unsigned int i = 0; i < mConstraints.size(); i++){
        if(!mConstraints[i]->mActive){
            count += mConstraints[i]->mNumRows;
            continue;
        }
        Eigen::VectorXd constraintVal = mConstraints[i]->evalCon();
        unsigned int n = constraintVal.size();

        for (unsigned int j = 0; j < n; j++) {
            mCon[count++] = constraintVal[j];
        }
    }
}

void ConstraintBox::setNumDofs(int numDofs) {
    mNumDofs = numDofs;
    for (int i = 0; i < mNumTotalRows; i++){
        if(mJac[i]->size()!=mNumDofs){
            mJac[i]->resize(numDofs);
            mJacMap[i]->resize(numDofs);
            for (int j = 0; j < numDofs; j++){
                mJac[i]->at(j) = 0.0;
                mJacMap[i]->at(j) = 0;
            }
        }
    }
}

void ConstraintBox::reallocateMem() {
    for (unsigned int i = 0; i < mConstraints.size(); i++) {
        mConstraints[i]->allocateMem();
    }
}

} // namespace optimizer
} // namespace dart

