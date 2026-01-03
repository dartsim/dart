--- dart/common/SharedLibrary.hpp.orig	2024-06-25 05:13:56 UTC
+++ dart/common/SharedLibrary.hpp
@@ -40,7 +40,7 @@
 #include <memory>
 #include <string>
 
-#if DART_OS_LINUX
+#if DART_OS_LINUX || DART_OS_FREEBSD
 
   #define DYNLIB_HANDLE void*
 
@@ -58,6 +58,8 @@ static constexpr const char* DART_SHARED_LIB_EXTENSION
 
 #if DART_OS_LINUX
 static constexpr const char* DART_SHARED_LIB_EXTENSION = "so";
+#elif DART_OS_FREEBSD
+static constexpr const char* DART_SHARED_LIB_EXTENSION = "so";
 #elif DART_OS_MACOS
 static constexpr const char* DART_SHARED_LIB_EXTENSION = "dylib";
 #elif DART_OS_WINDOWS
@@ -67,6 +69,8 @@ static constexpr const char* DART_SHARED_LIB_EXTENSION
 #endif
 
 #if DART_OS_LINUX
+static constexpr const char* DART_SHARED_LIB_PREFIX = "lib";
+#elif DART_OS_FREEBSD
 static constexpr const char* DART_SHARED_LIB_PREFIX = "lib";
 #elif DART_OS_MACOS
 static constexpr const char* DART_SHARED_LIB_PREFIX = "lib";
