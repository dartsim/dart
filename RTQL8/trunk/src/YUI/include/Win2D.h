#ifndef _WIN2D_
#define _WIN2D_

#include "GlutWindow.h"

class Win2D : public GlutWindow
{
protected:
	bool mTranslate;
	double mTransX;
	double mTransY;
public:
	Win2D();

	virtual void resize(int w, int h);
	virtual void render();
	
	virtual void keyboard(unsigned char key, int x, int y);
	virtual void click(int button, int state, int x, int y);
	virtual void drag(int x, int y);
	
	virtual void initGL();
	virtual void draw()=0;
};

#endif
