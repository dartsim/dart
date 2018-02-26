get_filename_component(example_name ${CMAKE_CURRENT_LIST_DIR} NAME)

if(NOT TARGET dart-utils-urdf
  OR NOT TARGET dart-gui
  OR NOT TARGET dart-collision-ode
  OR NOT TARGET dart-collision-bullet)
  return()
endif()

find_package(TinyDNN)
if(NOT TinyDNN_FOUND)
  return()
endif()

find_package(Threads)
if(NOT Threads_FOUND)
  return()
endif()

file(GLOB srcs "*.cpp" "*.hpp")

add_executable(${example_name} ${srcs})
target_compile_options(${example_name} PUBLIC -std=c++14)
target_link_libraries(${example_name}
  PUBLIC
    dart-utils-urdf dart-gui dart-collision-ode dart-collision-bullet
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
