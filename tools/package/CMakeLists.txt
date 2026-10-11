cmake_minimum_required(VERSION 3.22.1)
project(DARTPackageConsumer LANGUAGES C CXX)

find_package(DART REQUIRED CONFIG COMPONENTS ${DART_TEST_COMPONENTS})
add_executable(package_core core.cpp)
target_link_libraries(package_core PRIVATE dart)
add_executable(package_factory factory.cpp)
target_link_libraries(package_factory PRIVATE dart)
foreach(component ${DART_TEST_COMPONENTS})
  if(NOT component STREQUAL dart)
    target_link_libraries(package_core PRIVATE dart-${component})
  endif()
endforeach()
enable_testing()
add_test(NAME package_core COMMAND package_core)
add_test(NAME package_factory COMMAND package_factory)

if(
  io IN_LIST DART_TEST_COMPONENTS
  OR utils IN_LIST DART_TEST_COMPONENTS
  OR utils-urdf IN_LIST DART_TEST_COMPONENTS
  OR utils-assets IN_LIST DART_TEST_COMPONENTS
)
  add_executable(package_io io.cpp)
  foreach(io_component io utils-assets utils-urdf utils)
    if(io_component IN_LIST DART_TEST_COMPONENTS)
      target_link_libraries(package_io PRIVATE dart-${io_component})
      break()
    endif()
  endforeach()
  if(utils-assets IN_LIST DART_TEST_COMPONENTS)
    target_compile_definitions(package_io PRIVATE DART_PACKAGE_HAS_UTILS_ASSETS)
  endif()
  add_test(NAME package_io COMMAND package_io "${DART_DIR}/../../doc/dart/data")
endif()

if(
  gui IN_LIST DART_TEST_COMPONENTS
  OR gui-osg IN_LIST DART_TEST_COMPONENTS
  OR external-imgui IN_LIST DART_TEST_COMPONENTS
)
  add_executable(package_gui gui.cpp)
  target_link_libraries(package_gui PRIVATE dart-gui)
  add_test(NAME package_gui COMMAND package_gui)
endif()
