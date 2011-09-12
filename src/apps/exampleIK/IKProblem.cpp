#include "IKProblem.h"

#include <iostream>
using namespace std;

#include <glog/logging.h>
using namespace google;

#include "kinematics/FileInfoSkel.hpp"
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Marker.h"
using namespace kinematics;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "PositionConstraint.h"
#include "utils/Paths.h"

namespace optimizer {
    IKProblem::IKProblem(const char* const filenameSkel)
        : Problem(), mFileInfoSkel(NULL) {
        initProblem(filenameSkel);
    }

    IKProblem::~IKProblem() {
        delete mFileInfoSkel;
    }

    void IKProblem::initProblem(const char* const filenameSkel) {
        mFileInfoSkel = new FileInfoSkel<Skeleton>();


        string ext( filenameSkel );
        ext = ext.substr(ext.length() - 3);
        bool result = mFileInfoSkel->loadFile(filenameSkel);
        CHECK(result);

        // add variables
        for (int i = 0; i < getSkel()->getNumDofs(); i++) {
            addVariable((double)i / 100.0, -10.0, 10.0);
        }
        LOG(INFO) << "add # " << getNumVariables() << " Variables";

        // Create Con and Obj Boxes
        createBoxes();

        // add positional constraints
        mConstraints.clear();
        for (int i = 0; i < getSkel()->getNumMarkers(); i++) {
            Marker* marker = getSkel()->getMarker(i);
            BodyNode* node = marker->getNode();
            Eigen::Vector3d offset = marker->getLocalCoords();
            PositionConstraint* p = new PositionConstraint(
                this->vars(), getSkel(), node, offset, Eigen::Vector3d::Zero());
            objBox()->add(p);
            mConstraints.push_back(p);
        }

        LOG(INFO) << "# Constraints = " << conBox()->getNumConstraints();
        LOG(INFO) << "# Objectives = " << objBox()->getNumConstraints();

        LOG(INFO) << "initProblem OK";
    }

    void IKProblem::update(double* coefs) {
        Eigen::VectorXd pose(mVariables.size());
        for (unsigned int i = 0; i < mVariables.size(); ++i) {
            pose(i) = mVariables[i]->mVal;
        }
        // cout << "IKProblem::update()" << endl;
        // cout << pose.transpose() << endl;
        getSkel()->setPose(pose,true,true);
    }

    Skeleton* IKProblem::getSkel() const {
        return mFileInfoSkel->getSkel();
    }

    PositionConstraint* IKProblem::getConstraint(int index) const {
        return mConstraints[index];
    }

} // namespace optimizer
