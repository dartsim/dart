--- cmake/dart_defs.cmake.orig
+++ cmake/dart_defs.cmake
@@ -543,7 +543,7 @@
         -Wno-dangling-pointer
       )
     endif()
-    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
+    set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG")
     set(
       CMAKE_CXX_FLAGS_DEBUG
       "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls"
@@ -570,7 +570,7 @@
     if("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
     endif()
-    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
+    set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG")
     set(
       CMAKE_CXX_FLAGS_DEBUG
       "-g -fno-omit-frame-pointer -fno-inline-functions -fno-optimize-sibling-calls"
