#ifndef _MYWINDOW_
#define _MYWINDOW_

#include "YUI/Win3D.h"
#include "model3d/FileInfoDof.h"
#include "renderer/OpenGLRenderInterface.h"

class MyWindow : public Win3D
{
public:
	MyWindow(model3d::FileInfoDof& mot): Win3D(), mMotion(mot)
	{
		mBackground[0] = 1.0;
		mBackground[1] = 1.0;
		mBackground[2] = 1.0;
		mBackground[3] = 1.0;
		
		mPlaying = false;
		bMarker = false;
		bShowProgress = false;

		mPersp = 30.f;
		mTrans[2] = -1.f;
	}

	virtual void draw();
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void displayTimer(int val);
	virtual void move(int x, int y);
	void computeMax();
	
protected:	

	int mMaxFrame;
	bool mPlaying;
	int mFrame;
	bool bMarker;
	bool bShowProgress;
	model3d::FileInfoDof& mMotion;
	Renderer::OpenGLRenderInterface mRenderer;
};

#endif
