#include "MyWindow.h"
#include "dynamics/SkeletonDynamics.h"
#include "utils/UtilsMath.h"
#include "yui/GLFuncs.h"
#include <cstdio>
#include "model3d/BodyNode.h"
#include "model3d/Primitive.h"

using namespace std;
using namespace Eigen;
using namespace model3d;


void evalRT(MatrixXd mat, Vec3f R[3], Vec3f& T)
{
	//mat.transposeInPlace();
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			R[i][j] = mat(j, i);
	for(int i=0;i<3;i++)
		T[i] = mat(i, 3);
}
template<class BV>
void drawBVHModel(BVHModel<BV>& model)
{

glBegin(GL_TRIANGLES);
for(int i=0;i<model.num_tris;i++)
{
	Triangle tri = model.tri_indices[i];
	glVertex3f(model.vertices[tri[0]][0], model.vertices[tri[0]][1], model.vertices[tri[0]][2]);
	glVertex3f(model.vertices[tri[1]][0], model.vertices[tri[1]][1], model.vertices[tri[1]][2]);
	glVertex3f(model.vertices[tri[2]][0], model.vertices[tri[2]][1], model.vertices[tri[2]][2]);
}
glEnd();

}

void MyWindow::initDyn()
{
    // set random initial conditions
    mDofs.resize(mModel->getNumDofs());
	printf("%d\n", mModel->getNumDofs());
    mDofVels.resize(mModel->getNumDofs());
    for(unsigned int i=0; i<mModel->getNumDofs(); i++){
        mDofs[i] = utils::random(-0.1,0.1);
        mDofVels[i] = utils::random(-0.1,0.1);
		
    }
    mModel->setPose(mDofs,false,false);

	mBox = createCube<RSS>(0.05, 0.05, 0.05);

	printf("Node: %d\n", mModel->getNumNodes());
	for(int i=0;i<mModel->getNumNodes();i++)
	{
		BodyNode* node = mModel->getNode(i);
		BVHModel<RSS>* cdmesh = createCube<RSS>(node->getPrimitive()->getDim()[0], node->getPrimitive()->getDim()[1], node->getPrimitive()->getDim()[2]);
		mBody.push_back(cdmesh);
		
	}
}

void MyWindow::displayTimer(int _val)
{
    int numIter = mDisplayTimeout / (mTimeStep*1000);
    for(int i=0; i<numIter; i++){
        mModel->setPose(mDofs,false,false);
        mModel->computeDynamics(mGravity, mDofVels, true);
        VectorXd qddot = -mModel->mM.fullPivHouseholderQr().solve(mModel->mCg); 
        mModel->clampRotation(mDofs,mDofVels);
        mDofVels += qddot*mTimeStep;
        mDofs += mDofVels*mTimeStep;
    }

    mFrame += numIter;   
    glutPostRedisplay();
    if(mRunning)	
        glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
}

void MyWindow::draw()
{
    //mModel->draw(mRI);
    //if(mShowMarker) mModel->drawHandles(mRI);
	glDisable(GL_LIGHTING);

	Vec3f R1[3];

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			R1[i][j]=0;
	for(int i=0;i<3;i++)
		R1[i][i]=1;

	Vec3f T1 = Vec3f(0, -0.4, 0);
	Vec3f R2[3], T2;

	bool bCollide = false;
	for(int i=0;i<mModel->getNumNodes();i++)
	{
		BVH_CollideResult res;
		evalRT(mModel->getNode(i)->getWorldTransform(), R2, T2);
		collide(*mBox, R1, T1, *mBody[i], R2, T2, &res);
		if(res.numPairs()>0){
			//printf("c\n");
			glColor3f(1,0,0);
			bCollide=true;
		}
		else glColor3f(0.9, 0.9, 0.9);

		glPushMatrix();
		glMultMatrixd(mModel->getNode(i)->getWorldTransform().data());
		drawBVHModel(*mBody[i]);
		glPopMatrix();
	}

	glPushMatrix();
	glTranslatef(0, -0.4, 0);
	if(bCollide)glColor3f(1,0,0);
	else glColor3f(0.9, 0.9, 0.9);
	drawBVHModel(*mBox);
	glPopMatrix();

    // display the frame count in 2D text
    char buff[64];
    sprintf(buff,"%d",mFrame);
    string frame(buff);
    glDisable(GL_LIGHTING);
    glColor3f(0.0,0.0,0.0);
    yui::drawStringOnScreen(0.02f,0.02f,frame);
    glEnable(GL_LIGHTING);
}

void MyWindow::keyboard(unsigned char key, int x, int y)
{
    switch(key){
    case ' ': // use space key to play or stop the motion
        mRunning = !mRunning;
        if(mRunning)
            glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
        break;
    case 'r': // reset the motion to the first frame
        mFrame = 0;
        break;
    case 'h': // show or hide markers
        mShowMarker = !mShowMarker;
        break;
    default:
        Win3D::keyboard(key,x,y);
    }
    glutPostRedisplay();
}


