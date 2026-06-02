--- examples/wam_ikfast/Helpers.cpp.orig	2024-06-25 05:13:56 UTC
+++ examples/wam_ikfast/Helpers.cpp
@@ -109,7 +109,7 @@ void setupEndEffectors(const dart::dynamics::SkeletonP
 
   std::stringstream ss;
   ss << DART_SHARED_LIB_PREFIX << "wamIk";
-#if (DART_OS_LINUX || DART_OS_MACOS) && DART_BUILD_MODE_DEBUG
+#if (DART_OS_LINUX || DART_OS_FREEBSD || DART_OS_MACOS) && DART_BUILD_MODE_DEBUG
   ss << "d";
 #endif
   ss << "." << DART_SHARED_LIB_EXTENSION;
