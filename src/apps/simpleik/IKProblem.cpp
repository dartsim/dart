#include "IKProblem.h"

#include <iostream>
using namespace std;

#include <glog/logging.h>
using namespace google;

#include "model3d/FileInfoSkel.hpp"
#include "model3d/Skeleton.h"
#include "model3d/BodyNode.h"
#include "model3d/Marker.h"
using namespace model3d;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "PositionConstraint.h"
#include "utils/Paths.h"

namespace optimizer {
    IKProblem::IKProblem()
        : Problem(), mFileInfoSkel(NULL) {
        initProblem();
    }

    IKProblem::~IKProblem() {
        delete mFileInfoSkel;
    }

    void IKProblem::initProblem() {
        mFileInfoSkel = new FileInfoSkel<Skeleton>();
        bool result = mFileInfoSkel->loadFile(GROUNDZERO_DATA_PATH"skel/SehoonVSK3.vsk", model3d::VSK);
        CHECK(result);

        // add variables
        for (int i = 0; i < getSkel()->getNumDofs(); i++) {
            addVariable((double)i / 100.0, -10.0, 10.0);
        }
        LOG(INFO) << "add # " << getNumVariables() << " Variables";

        // Create Con and Obj Boxes
        createBoxes();

        // add positional constraints
        for (int i = 0; i < getSkel()->getNumHandles(); i++) {
            Marker* marker = getSkel()->getHandle(i);
            BodyNode* node = marker->getNode();
            Eigen::Vector3d offset = marker->getLocalCoords();
            PositionConstraint* p = new PositionConstraint(
                this->vars(), getSkel(), node, offset, Eigen::Vector3d::Zero());
            objBox()->add(p);
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
        getSkel()->setState(pose);
    }

    Skeleton* IKProblem::getSkel() const {
        return mFileInfoSkel->getSkel();
    }


} // namespace optimizer
