get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

if(NOT TARGET dart-io-urdf
  OR NOT TARGET dart-gui
  OR NOT TARGET dart-collision-ode
  OR NOT TARGET dart-collision-bullet)
  return()
endif()

set(TinyDNN_USE_SERIALIZER ON)
find_package(TinyDNN QUIET)
if(NOT TinyDNN_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Failed to find TinyDNN. humanJointLimits is disabled.")
  endif()
  return()
endif()

find_package(Threads QUIET)
if(NOT Threads_FOUND)
  if(DART_VERBOSE)
    message(STATUS "Failed to find Threads. humanJointLimits is disabled.")
  endif()
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${srcs})
target_compile_options(${example_name} PUBLIC -std=c++14)
target_link_libraries(${example_name}
  PUBLIC
    dart-io-urdf
    dart-gui
    dart-collision-ode
    dart-collision-bullet
    ${TinyDNN_LIBRARIES}
)
set_target_properties(${example_name}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Thread
if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(${example_name} PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${example_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
endif()

dart_add_example(${example_name})
dart_format_add(${srcs})
