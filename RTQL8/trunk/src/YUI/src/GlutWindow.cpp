#if !WIN32
#include <dirent.h>
#endif

#include <iostream>
#include "GlutWindow.h"

std::vector<GlutWindow*> GlutWindow::mWindows;
std::vector<int> GlutWindow::mWinIDs;

GlutWindow::GlutWindow()
{
	mWinWidth = 0;
	mWinHeight = 0;
	mMouseX = 0;
	mMouseY = 0;
	mDisplayTimeout = 1000.0/30.0;
	mMouseDown = false;
	mMouseDrag = false;
	mCapture = false;
	mBackground[0] = 0.3;
	mBackground[1] = 0.3;
	mBackground[2] = 0.3;
	mBackground[3] = 1.0;
}

void GlutWindow::initWindow(int w, int h, const char* name)
{
	mWindows.push_back(this);

	mWinWidth = w;
	mWinHeight = h;
	
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_ACCUM);
	glutInitWindowPosition( 150, 100 );
	glutInitWindowSize( w, h );
	mWinIDs.push_back(glutCreateWindow( name ));
	
	glutDisplayFunc( refresh );
	glutReshapeFunc( reshape );
	glutKeyboardFunc( keyEvent );
	glutSpecialFunc( specKeyEvent );
	glutMouseFunc( mouseClick );
	glutMotionFunc( mouseDrag );
	//glutTimerFunc ( mDisplayTimeout, refreshTimer, 0 );
	//glutTimerFunc ( mDisplayTimeout, runTimer, 0 );
}

void GlutWindow::reshape(int w, int h)
{
	current()->resize(w,h);
}

void GlutWindow::keyEvent(unsigned char key, int x, int y)
{
	current()->keyboard(key, x, y);
}

void GlutWindow::specKeyEvent(int key, int x, int y)
{
	current()->specKey(key, x, y);
}

void GlutWindow::mouseClick(int button, int state, int x, int y)
{
	current()->click(button, state, x, y);
}

void GlutWindow::mouseDrag(int x, int y)
{
	current()->drag(x, y);
}

void GlutWindow::refresh()
{
	current()->render();
}

void GlutWindow::refreshTimer(int val)
{
	current()->displayTimer(val);
}

void GlutWindow::displayTimer(int val)
{
	glutPostRedisplay();
	glutTimerFunc(mDisplayTimeout, refreshTimer, val);
}

void GlutWindow::runTimer(int val)
{
	current()->simTimer(val);
}

bool GlutWindow::screenshot()
{
	static int count=0;
	char fileBase[32]="capture/";

#if !WIN32
	// test open the directory
	DIR* dp = opendir(fileBase);
	// create the directory if it doesn't exist
	if( dp == NULL ){
		char cmd[256];
		sprintf(cmd, "mkdir %s", fileBase);
		system(cmd);
	}else closedir(dp);
#endif	
	char fname[64];
	sprintf(fname, "%s%.4d.tga", fileBase, count++); 

	// read the pixels
	int w = glutGet(GLUT_WINDOW_WIDTH);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	int numPixels = w*h;
	unsigned char *pixels = new unsigned char[numPixels*3*sizeof(unsigned char)];
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	// swap red and blue, because TGA format is stupid
	int i;
	for (i=0; i < numPixels; i++) {
		pixels[i * 3 + 0] ^= pixels[i * 3 + 2];
		pixels[i * 3 + 2] ^= pixels[i * 3 + 0];
		pixels[i * 3 + 0] ^= pixels[i * 3 + 2];
	}

	// open the file
	FILE *fptr;
	fptr = fopen(fname, "wb");
	if (fptr == NULL) {
		return false;
	}

	// create tga header
	putc(0,fptr);
	putc(0,fptr);
	putc(2,fptr);                         // uncompressed RGB
	putc(0,fptr); putc(0,fptr);
	putc(0,fptr); putc(0,fptr);
	putc(0,fptr);
	putc(0,fptr); putc(0,fptr);           // X origin
	putc(0,fptr); putc(0,fptr);           // y origin
	putc((w & 0x00FF),fptr);
	putc((w & 0xFF00) / 256,fptr);
	putc((h & 0x00FF),fptr);
	putc((h & 0xFF00) / 256,fptr);
	putc(24,fptr);                        // 24 bit bitmap
	putc(0,fptr);

	// write the data
	fwrite(pixels, w*h*3*sizeof(char), 1, fptr);
	fclose(fptr);

	delete []pixels;

	return true;
}

inline GlutWindow* GlutWindow::current()
{
	int id = glutGetWindow();
	for(int i=0; i<mWinIDs.size(); i++){
		if(mWinIDs.at(i) == id){
			return mWindows.at(i);
		}
	}
	std::cout<<"An unknown error occured!"<<std::endl;
	exit(0);
}
