#include "collision_skeleton.h"
#include "collision_shapes.h"
#include "kinematics/Shape.h"
#include <cmath>
#include "utils/LoadOpengl.h"



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
    using namespace std;

    Eigen::MatrixXd worldTrans(4, 4);
    Eigen::MatrixXd matCOM;
    

    BVH_CollideResult res;
    Vec3f R1[3], T1, R2[3], T2;


    evalRT();
    otherNode->evalRT();


    res.num_max_contacts = num_max_contact;
    collide(*cdmesh, mR, mT, *otherNode->cdmesh, otherNode->mR, otherNode->mT, &res);
    
    int start, size;
    start = result.size();
    size = 0;

    int numCoplanarContacts = 0;
    int numNoContacts = 0;

    for(int i=0;i<res.numPairs();i++)
    {
        ContactPoint pair1, pair2;
        pair1.bd1 = bodyNode;
        pair1.bd2 = otherNode->bodyNode;

        pair1.bdID1 = this->bodynodeID;
        pair1.bdID2 = otherNode->bodynodeID;
        
        pair1.collisionSkeletonNode1 = this;
        pair1.collisionSkeletonNode2 = otherNode;

        


        Vec3f v;

        pair1.triID1 = res.id1(i);
        pair1.triID2 = res.id2(i);

        
        pair1.penetrationDepth = res.collidePairs()[i].penetration_depth;

        pair2 = pair1;
        
        int contactResult = evalContactPosition(res, otherNode, i, pair1.point, pair2.point, pair1.contactTri);
        pair2.contactTri=pair1.contactTri;
        if(contactResult==COPLANAR_CONTACT)numCoplanarContacts++;
        if(contactResult==NO_CONTACT)numNoContacts++;
         if(contactResult == NO_CONTACT||contactResult== COPLANAR_CONTACT)continue;
         v = res.collidePairs()[i].normal;
        pair1.normal = Eigen::Vector3d(v[0], v[1], v[2]);
        pair2.normal = Eigen::Vector3d(v[0], v[1], v[2]);
        
        result.push_back(pair1);
        result.push_back(pair2);

        size+=2;

    }

    const double ZERO = 0.001;
    const double ZERO2 = ZERO*ZERO;

    int cur = start;
    std::vector<int> deleteIDs;
    
    size = result.size()-start;
    deleteIDs.clear();

    for(int i=start;i<start+size;i++)
        for(int j=i+1;j<start+size;j++)
        {
            Eigen::Vector3d diff = result[i].point - result[j].point;
            if(diff.dot(diff)<3*ZERO2){
                deleteIDs.push_back(i);
                break;
            }
        }

    for(int i =deleteIDs.size()-1; i>=0;i--)
        result.erase(result.begin()+deleteIDs[i]);
    
    size = result.size()-start;
    deleteIDs.clear();
    
    
    bool bremove;
    
    for(int i=start;i<start+size;i++)
    {
        bremove = false;
        for(int j=start;j<start+size;j++)
        {
            if(j==i)continue;
            if(bremove)break;
            for(int k=start;k<start+size;k++)
            {
                if(i==j||i==k)continue;
                Eigen::Vector3d  v = (result[i].point-result[j].point).cross(result[i].point-result[k].point);
                if(v.dot(v)<ZERO2&&
                    ((result[i].point-result[j].point).dot(result[i].point-result[k].point)<0))
                {bremove = true;break;}
                
                
            }
        }
        if(bremove)deleteIDs.push_back(i);
    }
    
    for(int i =deleteIDs.size()-1; i>=0;i--)
       result.erase(result.begin()+deleteIDs[i]);

  
    
    int collisionNum = res.numPairs();
    return numCoplanarContacts*100+numNoContacts;
    return collisionNum;

}

void CollisionSkeletonNode::evalRT()
{
    Eigen::MatrixXd worldTrans(4, 4);
    Eigen::MatrixXd matCOM;
    matCOM.setIdentity(4, 4);


    for(int k=0;k<3;k++)
    {
        matCOM(k, 3) = bodyNode->getLocalCOM()[k];
    }
    worldTrans = bodyNode->getWorldTransform()*matCOM;
    mWorldTrans = worldTrans;

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            mR[i][j] = worldTrans(i, j);


    for(int i=0;i<3;i++)
        mT[i] = worldTrans(i, 3);
}

int CollisionSkeletonNode::evalContactPosition( BVH_CollideResult& result,  CollisionSkeletonNode* other, int idx, Eigen::Vector3d& contactPosition1, Eigen::Vector3d& contactPosition2, ConatctTriangle& contactTri )
{
    int id1, id2;
    Triangle tri1, tri2;
    CollisionSkeletonNode* node1 = this;
    CollisionSkeletonNode* node2 = other;
    id1 = result.id1(idx);
    id2 = result.id2(idx);
    tri1 = node1->cdmesh->tri_indices[id1];
    tri2 = node2->cdmesh->tri_indices[id2];

    Vec3f v1, v2, v3, p1, p2, p3;
    v1 = node1->cdmesh->vertices[tri1[0]];
    v2 = node1->cdmesh->vertices[tri1[1]];
    v3 = node1->cdmesh->vertices[tri1[2]];

    p1 = node2->cdmesh->vertices[tri2[0]];
    p2 = node2->cdmesh->vertices[tri2[1]];
    p3 = node2->cdmesh->vertices[tri2[2]];

    Vec3f contact1, contact2;


     v1 = node1->TransformVertex(v1);
     v2 = node1->TransformVertex(v2);
     v3 = node1->TransformVertex(v3);
     p1 = node2->TransformVertex(p1);
     p2 = node2->TransformVertex(p2);
     p3 = node2->TransformVertex(p3);
    int testRes;
    testRes = FFtest(v1, v2, v3, p1, p2, p3, contact1, contact2);
    contactTri.v1=v1;
    contactTri.v2=v2;
    contactTri.v3=v3;
    contactTri.u1=p1;
    contactTri.u2=p2;
    contactTri.u3=p3;
    contactPosition1 = Eigen::Vector3d(contact1[0], contact1[1], contact1[2]);
    contactPosition2 = Eigen::Vector3d(contact2[0], contact2[1], contact2[2]);
    return testRes;
}

void CollisionSkeletonNode::drawCollisionTriangle(int tri)
{
    Triangle Tri = cdmesh->tri_indices[tri];
    glVertex3f(cdmesh->vertices[Tri[0]][0], cdmesh->vertices[Tri[0]][1], cdmesh->vertices[Tri[0]][2]);
    glVertex3f(cdmesh->vertices[Tri[1]][0], cdmesh->vertices[Tri[1]][1], cdmesh->vertices[Tri[1]][2]);
    glVertex3f(cdmesh->vertices[Tri[2]][0], cdmesh->vertices[Tri[2]][1], cdmesh->vertices[Tri[2]][2]);
}

void CollisionSkeletonNode::drawCollisionSkeletonNode(bool bTrans)
{
    evalRT();

     double M[16];
     for(int i=0;i<4;i++)
         for(int j=0;j<4;j++)
             M[j*4+i] = mWorldTrans(i, j);
     Vec3f v1, v2, v3;
     glPushMatrix();
     if(bTrans)glMultMatrixd(M);
     glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
     glBegin(GL_TRIANGLES);
     for(int i=0;i<cdmesh->num_tris;i++)
     {
         Triangle tri = cdmesh->tri_indices[i];
          glVertex3f(cdmesh->vertices[tri[0]][0], cdmesh->vertices[tri[0]][1], cdmesh->vertices[tri[0]][2]);
          glVertex3f(cdmesh->vertices[tri[1]][0], cdmesh->vertices[tri[1]][1], cdmesh->vertices[tri[1]][2]);
          glVertex3f(cdmesh->vertices[tri[2]][0], cdmesh->vertices[tri[2]][1], cdmesh->vertices[tri[2]][2]);

     }
     glEnd();
     glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
     glPopMatrix();




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
        CollisionSkeletonNode* csnode = new CollisionSkeletonNode(_bd);
        csnode->bodynodeID = mCollisionSkeletonNodeList.size();
        mCollisionSkeletonNodeList.push_back(csnode);
        mbodyNodeHash[_bd]=csnode;
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
    int numCollision = 0;
    
    for(int i=0; i<mCollisionSkeletonNodeList.size();i++)
        for(int j=i+1;j<mCollisionSkeletonNodeList.size();j++)
        {
             if(mCollisionSkeletonNodeList[i]->bodyNode->getParentNode()==mCollisionSkeletonNodeList[j]->bodyNode||
                 mCollisionSkeletonNodeList[j]->bodyNode->getParentNode()==mCollisionSkeletonNodeList[i]->bodyNode)
                  continue;
             if( mCollisionSkeletonNodeList[i]->bodyNode->getSkel() == mCollisionSkeletonNodeList[j]->bodyNode->getSkel())
             continue;                       

            
            
            numCollision+=mCollisionSkeletonNodeList[i]->checkCollision(mCollisionSkeletonNodeList[j], mContactPointList, num_max_contact);
            
    
        }
    mNumTriIntersection = numCollision;
}

void SkeletonCollision::draw()
{
    for(int i=0;i<mCollisionSkeletonNodeList.size();i++)
        mCollisionSkeletonNodeList[i]->drawCollisionSkeletonNode();

    
}




}
