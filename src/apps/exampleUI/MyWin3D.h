#ifndef _MYWIN3D_
#define _MYWIN3D_

#include "YUI/Win3D.h"

class MyWin3D : public Win3D
{
public:
	MyWin3D(){
		mBackground[0] = 1.0;
		mBackground[1] = 1.0;
		mBackground[2] = 1.0;
	}

	virtual void draw(){
		glPushMatrix();
		glColor3f(0.5,0.5,0.5);
		glutSolidCube(0.5);
		glPopMatrix();
	}
};

#endif
