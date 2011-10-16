#include "collision_skeleton.h"
#include "collision_shapes.h"
#include "kinematics/Shape.h"



namespace collision_checking{

    
CollisionSkeletonNode::CollisionSkeletonNode(kinematics::BodyNode* _bodyNode)
{
    bodyNode = _bodyNode;
    cdmesh = createCube<RSS>(bodyNode->getShape()->getDim()[0], bodyNode->getShape()->getDim()[1], bodyNode->getShape()->getDim()[2]);
    
}
CollisionSkeletonNode::~CollisionSkeletonNode()
{
    delete cdmesh;
}

int CollisionSkeletonNode::checkCollision(CollisionSkeletonNode* otherNode, std::vector<ContactPoint>& result, int num_max_contact)
{
    Eigen::MatrixXd worldTrans(4, 4);
    Eigen::MatrixXd matCOM;
    

    BVH_CollideResult res;
    Vec3f R1[3], T1, R2[3], T2;

    matCOM.setIdentity(4, 4);

    for(int k=0;k<3;k++)
    {
        matCOM(k, 3) = bodyNode->getLocalCOM()[k];
    }
    worldTrans = bodyNode->getWorldTransform()*matCOM;
    evalRT(worldTrans, R1, T1);


    matCOM.setIdentity(4, 4);

    for(int k=0;k<3;k++)
    {
        matCOM(k, 3) = otherNode->bodyNode->getLocalCOM()[k];
    }
    worldTrans = otherNode->bodyNode->getWorldTransform()*matCOM;
    evalRT(worldTrans, R2, T2);

    res.num_max_contacts = num_max_contact;
    collide(*cdmesh, R1, T1, *otherNode->cdmesh, R2, T2, &res);

    for(int i=0;i<res.numPairs();i++)
    {
        ContactPoint pair;
        pair.bd1 = bodyNode;
        pair.bd2 = otherNode->bodyNode;
        Vec3f v;
        
        v = res.collidePairs()[i].contact_point;
        pair.point = Eigen::Vector3d(v[0], v[1], v[2]);
        v = res.collidePairs()[i].normal;
        pair.normal = Eigen::Vector3d(v[0], v[1], v[2]);
        result.push_back(pair);
    }

    int collisionNum = res.numPairs();
    return collisionNum;

}

void CollisionSkeletonNode::evalRT(Eigen::MatrixXd mat, Vec3f R[3], Vec3f& T)
{
    //mat.transposeInPlace();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            R[i][j] = mat(j, i);
    for(int i=0;i<3;i++)
        T[i] = mat(i, 3);
}


SkeletonCollision::~SkeletonCollision()
{
    for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
        delete mCollisionSkeletonNodeList[i];
}

void SkeletonCollision::addCollisionSkeletonNode(kinematics::BodyNode *_bd, bool _bRecursive)
{
    if(_bRecursive == false || _bd->getNumChildJoints() ==0)
    {
        mCollisionSkeletonNodeList.push_back(new CollisionSkeletonNode(_bd));
    }
    else
    {
        addCollisionSkeletonNode(_bd, false);
        for(int i=0; i<_bd->getNumChildJoints();i++)
            addCollisionSkeletonNode(_bd->getChildNode(i), true);
    }

}

void SkeletonCollision::checkCollision(bool bConsiderGround)
{

    int num_max_contact = 100;
    clearAllContacts();
    
    for(int i=0; i<mCollisionSkeletonNodeList.size();i++)
        for(int j=i+1;j<mCollisionSkeletonNodeList.size();j++)
        {
             if(mCollisionSkeletonNodeList[i]->bodyNode->getParentNode()==mCollisionSkeletonNodeList[j]->bodyNode||
                 mCollisionSkeletonNodeList[j]->bodyNode->getParentNode()==mCollisionSkeletonNodeList[i]->bodyNode)
                  continue;
           
            
            
            mCollisionSkeletonNodeList[i]->checkCollision(mCollisionSkeletonNodeList[j], mContactPointList, num_max_contact);
            
    
        }
}


}