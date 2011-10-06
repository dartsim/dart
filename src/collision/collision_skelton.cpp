#include "collision_skelton.h"
#include "collision_shapes.h"
#include "kinematics/Primitive.h"



namespace collision_checking{

    
CollisionSkeltonNode::CollisionSkeltonNode(kinematics::BodyNode* _bodyNode)
{
    bodyNode = _bodyNode;
    cdmesh = createCube<RSS>(bodyNode->getPrimitive()->getDim()[0], bodyNode->getPrimitive()->getDim()[1], bodyNode->getPrimitive()->getDim()[2]);
    
}
CollisionSkeltonNode::~CollisionSkeltonNode()
{
    delete cdmesh;
}

int CollisionSkeltonNode::checkCollision(CollisionSkeltonNode* otherNode, std::vector<ContactPair>& result, int num_max_contact)
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
        ContactPair pair;
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

void CollisionSkeltonNode::evalRT(Eigen::MatrixXd mat, Vec3f R[3], Vec3f& T)
{
    //mat.transposeInPlace();
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            R[i][j] = mat(j, i);
    for(int i=0;i<3;i++)
        T[i] = mat(i, 3);
}


SkeltonCollision::~SkeltonCollision()
{
    for(int i=0;i<mCollisionSkeltonNodeList.size();i++)
        delete mCollisionSkeltonNodeList[i];
}

void SkeltonCollision::addCollisionSkeltonNode(kinematics::BodyNode *_bd, bool _bRecursive)
{
    if(_bRecursive == false || _bd->getNumChildJoints() ==0)
    {
        mCollisionSkeltonNodeList.push_back(new CollisionSkeltonNode(_bd));
    }
    else
    {
        addCollisionSkeltonNode(_bd, false);
        for(int i=0; i<_bd->getNumChildJoints();i++)
            addCollisionSkeltonNode(_bd->getChildNode(i), true);
    }

}

void SkeltonCollision::checkCollision(bool bConsiderGround)
{

    int num_max_contact = 100;
    clearAllContacts();
    
    for(int i=0; i<mCollisionSkeltonNodeList.size();i++)
        for(int j=i+1;j<mCollisionSkeltonNodeList.size();j++)
        {
             if(mCollisionSkeltonNodeList[i]->bodyNode->getParentNode()==mCollisionSkeltonNodeList[j]->bodyNode||
                 mCollisionSkeltonNodeList[j]->bodyNode->getParentNode()==mCollisionSkeltonNodeList[i]->bodyNode)
                  continue;
           
            
            
            mCollisionSkeltonNodeList[i]->checkCollision(mCollisionSkeltonNodeList[j], mContactList, num_max_contact);
            
    
        }
}


}