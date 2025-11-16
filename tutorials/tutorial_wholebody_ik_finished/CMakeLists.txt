# Copyright (c) 2011-2025, The DART development contributors

get_filename_component(tutorial_name ${CMAKE_CURRENT_LIST_DIR} NAME)

file(GLOB tutorial_srcs "*.cpp" "*.hpp")
if(MSVC)
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
endif()
dart_add_tutorial(${tutorial_name} ${tutorial_srcs})
