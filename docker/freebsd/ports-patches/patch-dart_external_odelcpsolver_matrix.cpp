--- dart/external/odelcpsolver/matrix.cpp.orig	2018-11-20 16:43:32 UTC
+++ dart/external/odelcpsolver/matrix.cpp
@@ -23,6 +23,8 @@
 //#include "config.h"
 #ifdef __APPLE__
   #include <malloc/malloc.h>
+#elif defined(__FreeBSD__)
+  #include <stdlib.h>
 #else
   #include <malloc.h>
 #endif
