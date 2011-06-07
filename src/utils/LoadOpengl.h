#ifndef SRC_UTILS_LOAD_OPENGL_H
#define SRC_UTILS_LOAD_OPENGL_H

#if WIN32
#include <GL/glut.h>
#elif defined(__linux)
#include <GL/glut.h>
#elif APPLE
#include <Glut/glut.h>
#else
#error "Load OpenGL Error: What's your operating system?"
#endif

#endif // #ifndef SRC_UTILS_LOAD_OPENGL_H

