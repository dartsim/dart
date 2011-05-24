#ifndef _MYWIN3D_
#define _MYWIN3D_

#include "Win3D.h"
#include "OpenGLRenderInterface.h"
#include "skeleton.h"

class MyWin3D : public Win3D
{
public:
	MyWin3D(model3d::Skeleton* skel) : mSkel(skel){
		mBackground[0] = 1.0;
		mBackground[1] = 1.0;
		mBackground[2] = 1.0;
	}

	virtual void draw(){
/*		glPushMatrix();
		glColor3f(0.5,0.5,0.5);
		glutSolidCube(0.5);
		glPopMatrix();
*/
		mSkel->draw(&mRenderer, Vector4d(0.0,0.0,0.0,0.0), true);
	}

protected:
	model3d::Skeleton* mSkel;
	Renderer::OpenGLRenderInterface mRenderer;
};

#endif
