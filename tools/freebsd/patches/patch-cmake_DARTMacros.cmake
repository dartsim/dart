--- cmake/DARTMacros.cmake.orig	2024-06-26 09:54:39 UTC
+++ cmake/DARTMacros.cmake
@@ -370,6 +370,8 @@ function(dart_build_tests)
       ${target_name} PRIVATE ${_ARG_INCLUDE_DIRS}
     )
 
+    find_package(GTest REQUIRED)
+
     # Link libraries
     target_link_libraries(${target_name} PRIVATE GTest::gtest GTest::gtest_main)
     target_link_libraries(
