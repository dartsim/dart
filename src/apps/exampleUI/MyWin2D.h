#ifndef _MYWIN2D_
#define _MYWIN2D_

#include "YUI/Win2D.h"

class MyWin2D : public Win2D
{
public:
	MyWin2D(){ 
		mBackground[0] = 1.0;
		mBackground[1] = 1.0;
		mBackground[2] = 1.0;
	}

	virtual void draw(){
		glPushMatrix();
		glColor3f(0.5,0.5,0.5);
		glBegin(GL_QUADS);
		glVertex2f(-mWinWidth/4.0,-mWinHeight/4.0);
		glVertex2f(-mWinWidth/4.0,mWinHeight/4.0);
		glVertex2f(mWinWidth/4.0,mWinHeight/4.0);
		glVertex2f(mWinWidth/4.0,-mWinHeight/4.0);
		glEnd();
		glPopMatrix();
	}
};

#endif
