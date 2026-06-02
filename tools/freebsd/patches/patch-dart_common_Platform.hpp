--- dart/common/Platform.hpp.orig	2021-11-04 20:43:53 UTC
+++ dart/common/Platform.hpp
@@ -43,6 +43,15 @@
     #define DART_ARCH_32BITS 1
   #endif
 
+#elif defined(__FreeBSD__)
+
+#define DART_OS_FREEBSD      1
+#if __x86_64__ || __ppc64__
+#define DART_ARCH_64BITS   1
+#else
+#define DART_ARCH_32BITS   1
+#endif
+
 #elif defined(__APPLE__)
 
   #define DART_OS_MACOS 1
