#include "LandingProblem.h"

#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;

#include "kinematics/BodyNode.h"
#include "kinematics/Marker.h"
using namespace kinematics;
#include "dynamics/SkeletonDynamics.h"
using namespace dynamics;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "optimizer/Constraint.h"

namespace optimizer {
    LandingProblem::LandingProblem(dynamics::SkeletonDynamics *_skel)
        : Problem(), mSkel(_skel){
        initProblem();
    }

    LandingProblem::~LandingProblem() {
        // delete constraint and objective boxes
    }

    void LandingProblem::initProblem() {
        // add variables
        VectorXd pose;
        mSkel->getPose(pose);
        for (int i = 0; i < mSkel->getNumDofs(); i++) {
            addVariable(pose[i], -10.0, 10.0);
        }
        // Create Con and Obj Boxes
        createBoxes();
    }

    void LandingProblem::update(double* coefs) {
        VectorXd pose(mVariables.size());
        for (unsigned int i = 0; i < mVariables.size(); i++) {
            pose(i) = mVariables[i]->mVal;
        }
        mSkel->setPose(pose, true, true);
    }

    Constraint* LandingProblem::getConstraint(int index) const {
        return mConstraints[index];
    }

    std::vector<Constraint*> & LandingProblem::getConstraints() {
        return mConstraints;
    }

} // namespace optimizer
