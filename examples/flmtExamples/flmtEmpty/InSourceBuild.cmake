get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

find_package(filament REQUIRED)
find_package(Threads)
if(NOT Threads_FOUND)
  return()
endif()

if(NOT TARGET dart-utils-urdf OR NOT TARGET dart-gui-flmt)
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${srcs})
target_link_libraries(${example_name} dart dart-utils-urdf dart-gui-flmt)
if(BUILD_SHARED_LIBS)
  target_link_libraries(${example_name} ${filament_LIBRARIES})
endif()
set_target_properties(${example_name}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
#target_compile_options(${example_name} PUBLIC "-lc++")
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${example_name} PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${example_name} ${CMAKE_THREAD_LIBS_INIT})
endif()

dart_add_example(${example_name})
