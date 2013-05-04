#include <algorithm>
#include <cmath>
#include <fcl/collision.h>

#include "renderer/LoadOpengl.h"
#include "math/UtilsMath.h"

#include "kinematics/BodyNode.h"

#include "collision/CollisionNode.h"
#include "collision/fcl/CollisionShapes.h"
#include "collision/fcl/FCLCollisionNode.h"
#include "collision/fcl/FCLCollisionDetector.h"

using namespace std;
using namespace Eigen;
using namespace math;

namespace collision
{

FCLCollisionDetector::~FCLCollisionDetector() {
    for(int i=0;i<mCollisionNodes.size();i++)
        delete mCollisionNodes[i];
}

void FCLCollisionDetector::addCollisionSkeletonNode(kinematics::BodyNode *_bd,
                                                    bool _bRecursive)
{
    if (_bRecursive == false || _bd->getNumChildJoints() == 0)
    {
        FCLCollisionNode* csnode = new FCLCollisionNode(_bd);
        csnode->mBodyNodeID = mCollisionNodes.size();
        mCollisionNodes.push_back(csnode);
        mBodyCollisionMap[_bd] = csnode;
        mActiveMatrix.push_back(vector<bool>(mCollisionNodes.size() - 1));

        for(unsigned int i = 0; i < mCollisionNodes.size() - 1; i++)
        {
            //if(mCollisionSkeletonNodeList[i]->mBodyNode->getParentNode() == _bd || _bd->getParentNode() == mCollisionSkeletonNodeList[i]->mBodyNode) {
            if(mCollisionNodes[i]->mBodyNode->getSkel() == _bd->getSkel()) {
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

CollisionNode*FCLCollisionDetector::createCollisionNode(kinematics::BodyNode* _bodyNode)
{
    CollisionNode* collisionNode = NULL;

    collisionNode = new FCLCollisionNode(_bodyNode);

    return collisionNode;
}

bool FCLCollisionDetector::checkCollision(bool _checkAllCollisions,
                                       bool _calculateContactPoints)
{
    int num_max_contact = 100;
    clearAllContacts();
    mNumTriIntersection = 0;

    FCLCollisionNode* fclCollisionNode1 = NULL;
    FCLCollisionNode* fclCollisionNode2 = NULL;

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        mCollisionNodes[i]->mBodyNode->setColliding(false);
    }

    for (int i = 0; i < mCollisionNodes.size(); i++)
    {
        fclCollisionNode1 = static_cast<FCLCollisionNode*>(mCollisionNodes[i]);

        for (int j = i + 1; j < mCollisionNodes.size(); j++)
        {
            fclCollisionNode2 = static_cast<FCLCollisionNode*>(mCollisionNodes[j]);

            if (!mActiveMatrix[j][i])
            {
                continue;
            }
            const int numTriIntersection
                    = fclCollisionNode1->checkCollision(
                          fclCollisionNode2,
                          _calculateContactPoints ? &mContacts : NULL,
                          num_max_contact);

            mNumTriIntersection += numTriIntersection;

            if(numTriIntersection > 0)
            {
                mCollisionNodes[i]->mBodyNode->setColliding(true);
                mCollisionNodes[j]->mBodyNode->setColliding(true);
            }

            if(!_checkAllCollisions && mNumTriIntersection > 0)
            {
                return true;
            }
        }
    }

    return (mNumTriIntersection > 0);
}

void FCLCollisionDetector::draw() {
    for(int i=0;i<mCollisionNodes.size();i++)
        static_cast<FCLCollisionNode*>(mCollisionNodes[i])->drawCollisionSkeletonNode();
}

void FCLCollisionDetector::activatePair(const kinematics::BodyNode* node1, const kinematics::BodyNode* node2) {
    int nodeId1 = getCollisionSkeletonNode(node1)->mBodyNodeID;
    int nodeId2 = getCollisionSkeletonNode(node2)->mBodyNodeID;
    if(nodeId1 < nodeId2) {
        swap(nodeId1, nodeId2);
    }
    mActiveMatrix[nodeId1][nodeId2] = true;
}

void FCLCollisionDetector::deactivatePair(const kinematics::BodyNode* node1, const kinematics::BodyNode* node2) {
    int nodeId1 = getCollisionSkeletonNode(node1)->mBodyNodeID;
    int nodeId2 = getCollisionSkeletonNode(node2)->mBodyNodeID;
    if(nodeId1 < nodeId2) {
        swap(nodeId1, nodeId2);
    }
    mActiveMatrix[nodeId1][nodeId2] = false;
}
}
