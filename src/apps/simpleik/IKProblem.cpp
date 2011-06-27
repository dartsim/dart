#include "IKProblem.h"

#include <iostream>
using namespace std;

#include <glog/logging.h>
using namespace google;

#include "model3d/FileInfoModel.h"
#include "model3d/Skeleton.h"
#include "model3d/BodyNode.h"
#include "model3d/Marker.h"
using namespace model3d;

#include "optimizer/Var.h"
#include "optimizer/ObjectiveBox.h"
#include "optimizer/ConstraintBox.h"
using namespace optimizer;
#include "PositionConstraint.h"


namespace optimizer {
    IKProblem::IKProblem()
        : Problem(), mFileInfoModel(NULL) {
        initProblem();
    }

    IKProblem::~IKProblem() {
        delete mFileInfoModel;
    }

    void IKProblem::initProblem() {
        mFileInfoModel = new FileInfoModel();
        bool result = mFileInfoModel->loadFile("./SehoonVSK3.vsk", FileInfoModel::VSK);
        CHECK(result);

        // Add variables
        for (int i = 0; i < getModel()->getNumDofs(); i++) {
            addVariable((double)i / 100.0, -10.0, 10.0);
        }
        LOG(INFO) << "Add # " << getNumVariables() << " Variables";

        // Create Con and Obj Boxes
        createBoxes();

        // Add positional constraints
        for (int i = 0; i < getModel()->getNumHandles(); i++) {
            Marker* marker = getModel()->getHandle(i);
            BodyNode* node = marker->getNode();
            Eigen::Vector3d offset = marker->getLocalCoords();
            PositionConstraint* p = new PositionConstraint(
                this->vars(), getModel(), node, offset, Eigen::Vector3d::Zero());
            objBox()->Add(p);
        }

        LOG(INFO) << "# Constraints = " << conBox()->getNumConstraints();
        LOG(INFO) << "# Objectives = " << objBox()->getNumConstraints();

    

        LOG(INFO) << "initProblem OK";
    }

    void IKProblem::update(double* coefs) {
        Eigen::VectorXd pose(mVariables.size());
        for (int i = 0; i < mVariables.size(); ++i) {
            pose(i) = mVariables[i]->mVal;
        }
        // cout << "IKProblem::update()" << endl;
        // cout << pose.transpose() << endl;
        getModel()->setState(pose);
    }

    Skeleton* IKProblem::getModel() const {
        return mFileInfoModel->getModel();
    }


} // namespace optimizer
