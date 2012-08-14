#ifndef GL_COMMON_H
#define GL_COMMON_H

// Cross-platform GL libraries

// Windows
#ifdef WIN32
#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

//	#include <GL/glew.h>
//	#define GLUT_DISABLE_ATEXIT_HACK
//	#include <gl/GLut.h>
#endif

// Mac OS X
#ifdef __APPLE__
    #include <OpenGL/OpenGL.h>
    #include <GLUT/GLUT.h>
	#include <sys/time.h>
//	#define TRUE true
//	#define FALSE false
#endif

// LINUX
#ifdef linux
	#include <GL/glu.h>
//	#include <GL/glew.h>
//	#include <GL/glut.h>
	#include <unistd.h>
#ifndef TRUE
	#define TRUE true
#endif
#ifndef FALSE
	#define FALSE false
#endif
	typedef unsigned long DWORD;
#endif



#endif
