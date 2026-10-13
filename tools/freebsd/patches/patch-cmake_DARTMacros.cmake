--- cmake/DARTMacros.cmake.orig
+++ cmake/DARTMacros.cmake
@@ -405,6 +405,8 @@
     # Include directories
     target_include_directories(${target_name} PRIVATE ${_ARG_INCLUDE_DIRS})
 
+    find_package(GTest REQUIRED)
+
     # Link libraries
     target_link_libraries(${target_name} PRIVATE GTest::gtest GTest::gtest_main)
     target_link_libraries(${target_name} PRIVATE ${_ARG_LINK_LIBRARIES})
