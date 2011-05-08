#ifndef _WIN3D_
#define _WIN3D_

#include "GlutWindow.h"
#include "Trackball.h"
#include <Eigen/Eigen>

class Win3D : public GlutWindow
{
public:
	Win3D();

	virtual void initWindow(int w, int h, const char* name);
	virtual void resize(int w, int h);
	virtual void render();

	virtual void keyboard(unsigned char key, int x, int y);
	virtual void click(int button, int state, int x, int y);
	virtual void drag(int x, int y);

	virtual void Capturing();
	virtual void initGL();
	virtual void initLights();
	
	virtual void draw()=0;

protected:
	Trackball mTrackBall;
	Eigen::Vector3d mTrans;
	Eigen::Vector3d mEye;
	float mZoom;
	float mPersp;

	bool mRotate;
	bool mTranslate;
	bool mZooming;
};

#endif
