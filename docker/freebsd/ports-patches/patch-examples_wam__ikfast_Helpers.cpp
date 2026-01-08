--- examples/wam_ikfast/Helpers.cpp.orig	2024-06-25 05:13:56 UTC
+++ examples/wam_ikfast/Helpers.cpp
@@ -116,7 +116,7 @@ void setupEndEffectors(const dart::dynamics::SkeletonP
 
   std::stringstream ss;
   ss << DART_SHARED_LIB_PREFIX << "wamIk";
-#if (DART_OS_LINUX || DART_OS_MACOS) && !defined(NDEBUG)
+#if (DART_OS_LINUX || DART_OS_FREEBSD || DART_OS_MACOS) && !defined(NDEBUG)
   ss << "d";
 #endif
   ss << "." << DART_SHARED_LIB_EXTENSION;
