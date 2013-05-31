#include <algorithm>
#include <cmath>
#include <fcl/collision.h>

#include "renderer/LoadOpengl.h"
#include "math/UtilsMath.h"

#include "kinematics/BodyNode.h"

#include "collision/CollisionNode.h"
#include "collision/fcl_mesh/CollisionShapes.h"
#include "collision/fcl_mesh/FCLMESHCollisionNode.h"
#include "collision/fcl_mesh/FCLMESHCollisionDetector.h"

using namespace std;
using namespace Eigen;
using namespace dart_math;

namespace collision
{

FCLMESHCollisionDetector::~FCLMESHCollisionDetector() {
}

void FCLMESHCollisionDetector::addCollisionSkeletonNode(kinematics::BodyNode *_bd,
                                                    bool _bRecursive)
{
    if (_bRecursive == false || _bd->getNumChildJoints() == 0)
    {
        FCLMESHCollisionNode* csnode = new FCLMESHCollisionNode(_bd);
        csnode->setBodyNodeID(mCollisionNodes.size());
        mCollisionNodes.push_back(csnode);
        mBodyCollisionMap[_bd] = csnode;
        mActiveMatrix.push_back(vector<bool>(mCollisionNodes.size() - 1));

        for(unsigned int i = 0; i < mCollisionNodes.size() - 1; i++)
        {
            //if(mCollisionSkeletonNodeList[i]->mBodyNode->getParentNode() == _bd || _bd->getParentNode() == mCollisionSkeletonNodeList[i]->mBodyNode) {
            if(mCollisionNodes[i]->getBodyNode()->getSkel() == _bd->getSkel()) {
                mActiveMatrix.back()[i] = false;
            }
            else
            {
                mActiveMatrix.back()[i] = true;
            }
        }
    }
    else
    {
        addCollisionSkeletonNode(_bd, false);

        for (int i = 0; i < _bd->getNumChildJoints(); i++)
            addCollisionSkeletonNode(_bd->getChildNode(i), true);
    }
}

CollisionNode*FCLMESHCollisionDetector::createCollisionNode(kinematics::BodyNode* _bodyNode)
{
    CollisionNode* collisionNode = NULL;

    collisionNode = new FCLMESHCollisionNode(_bodyNode);

    return collisionNode;
}

bool FCLMESHCollisionDetector::checkCollision(bool _checkAllCollisions,
                                       bool _calculateContactPoints)
{
    int num_max_contact = 100;
    clearAllContacts();
    mNumTriIntersection = 0;

    FCLMESHCollisionNode* FCLMESHCollisionNode1 = NULL;
    FCLMESHCollisionNode* FCLMESHCollisionNode2 = NULL;

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        mCollisionNodes[i]->getBodyNode()->setColliding(false);
    }

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        FCLMESHCollisionNode1 = static_cast<FCLMESHCollisionNode*>(mCollisionNodes[i]);

        for (int j = i + 1; j < mCollisionNodes.size(); j++)
        {
            FCLMESHCollisionNode2 = static_cast<FCLMESHCollisionNode*>(mCollisionNodes[j]);

            if (!mActiveMatrix[j][i])
            {
                continue;
            }
            const int numTriIntersection
                    = FCLMESHCollisionNode1->checkCollision(
                          FCLMESHCollisionNode2,
                          _calculateContactPoints ? &mContacts : NULL,
                          num_max_contact);

            mNumTriIntersection += numTriIntersection;

            if(numTriIntersection > 0)
            {
                mCollisionNodes[i]->getBodyNode()->setColliding(true);
                mCollisionNodes[j]->getBodyNode()->setColliding(true);
            }

            if(!_checkAllCollisions && mNumTriIntersection > 0)
            {
                return true;
            }
        }
    }

    return (mNumTriIntersection > 0);
}

void FCLMESHCollisionDetector::draw() {
    for(int i=0;i<mCollisionNodes.size();i++)
        static_cast<FCLMESHCollisionNode*>(mCollisionNodes[i])->drawCollisionSkeletonNode();
}

void FCLMESHCollisionDetector::activatePair(const kinematics::BodyNode* node1, const kinematics::BodyNode* node2) {
    int nodeId1 = getCollisionSkeletonNode(node1)->getBodyNodeID();
    int nodeId2 = getCollisionSkeletonNode(node2)->getBodyNodeID();
    if(nodeId1 < nodeId2) {
        swap(nodeId1, nodeId2);
    }
    mActiveMatrix[nodeId1][nodeId2] = true;
}

void FCLMESHCollisionDetector::deactivatePair(const kinematics::BodyNode* node1, const kinematics::BodyNode* node2) {
    int nodeId1 = getCollisionSkeletonNode(node1)->getBodyNodeID();
    int nodeId2 = getCollisionSkeletonNode(node2)->getBodyNodeID();
    if(nodeId1 < nodeId2) {
        swap(nodeId1, nodeId2);
    }
    mActiveMatrix[nodeId1][nodeId2] = false;
}
}
