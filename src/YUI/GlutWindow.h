#ifndef _GLUTWINDOW_
#define _GLUTWINDOW_

#include "utils/LoadOpengl.h"
#include <vector>

class GlutWindow
{
public:
	GlutWindow();
	virtual void initWindow(int w, int h, const char* name);

// callback functions	
	static void reshape(int w, int h);
	static void keyEvent(unsigned char key, int x, int y);
	static void specKeyEvent( int key, int x, int y );
	static void mouseClick(int button, int state, int x, int y);
	static void mouseDrag(int x, int y);
	static void mouseMove(int x, int y);
	static void refresh();
	static void refreshTimer(int);
	static void runTimer(int);
	
	static GlutWindow* current();
	static std::vector<GlutWindow*> mWindows;
	static std::vector<int> mWinIDs;

protected:
// callback implementation	
	virtual void resize(int w, int h)=0;
	virtual void render()=0;
	virtual void keyboard(unsigned char key, int x, int y){}
	virtual void specKey( int key, int x, int y){}
	virtual void click(int button, int state, int x, int y){}
	virtual void drag(int x, int y){}
	virtual void move(int x, int y){}
	virtual void displayTimer(int);
	virtual void simTimer(int){}

	virtual bool screenshot();

	int mWinWidth;
	int mWinHeight;
	int mMouseX;
	int mMouseY;
	double mDisplayTimeout;
	bool mMouseDown;
	bool mMouseDrag;
	bool mCapture;
	double mBackground[4];
};

#endif
