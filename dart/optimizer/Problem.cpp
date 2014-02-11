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

#include "Problem.h"

#include <iostream>

#include "Var.h"
#include "Constraint.h"
#include "ConstraintBox.h"
#include "ObjectiveBox.h"

namespace dart {
namespace optimizer {

Problem::Problem()
    : mConBox(NULL), mObjBox(NULL) {
    mVariables.clear();
}

Problem::~Problem() {
    delete mConBox;

    delete mObjBox;

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
} // namespace dart
