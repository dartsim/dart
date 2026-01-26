--- dart/gui/glut/LoadGlut.hpp.orig	2024-06-25 05:13:56 UTC
+++ dart/gui/glut/LoadGlut.hpp
@@ -39,6 +39,8 @@
   #include <cstdlib> // To disable glut::exit() function
 #elif defined(__linux__)
   #include <GL/glut.h>
+#elif defined(__FreeBSD__)
+  #include <GL/glut.h>
 #elif defined(__APPLE__)
   #include <GLUT/glut.h>
 #else
