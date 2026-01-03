--- dart/gui/LoadOpengl.hpp.orig	2024-06-25 05:13:56 UTC
+++ dart/gui/LoadOpengl.hpp
@@ -42,6 +42,9 @@
 #elif defined(__linux__)
   #include <GL/gl.h>
   #include <GL/glu.h>
+#elif defined(__FreeBSD__)
+  #include <GL/gl.h>
+  #include <GL/glu.h>
 #elif defined(__APPLE__)
   #include <OpenGL/gl.h>
   #include <OpenGL/glu.h>
