find_package(filament QUIET)
dart_check_optional_package(filament "dart-gui-flmt" "filament" "0.0.0")

#find_package(Threads)
#if(NOT Threads_FOUND)
#  return()
#endif()
find_package(SDL2 REQUIRED)

# Compiler check
if("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU"
    OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  message(WARNING
    "The GCC compiler is not supported for dart-gui-flmt, please use Clang for "
    "both C and C++. Skipping building dart-gui-flmt.")
#  return()
endif()

#==============================================================================
# Build materials
#==============================================================================
#set(MATERIAL_SRCS
#  materials/bakedColor.mat
#  materials/bakedTexture.mat
#  materials/aiDefaultMat.mat
#  materials/aiDefaultTrans.mat
#  materials/depthVisualizer.mat
#  materials/groundShadow.mat
#  materials/sandboxCloth.mat
#  materials/sandboxLit.mat
#  materials/sandboxLitFade.mat
#  materials/sandboxLitTransparent.mat
#  materials/sandboxSubsurface.mat
#  materials/sandboxUnlit.mat
#  materials/transparentColor.mat
#)

#if (CMAKE_CROSSCOMPILING)
#  set(IMPORT_EXECUTABLES ${FILAMENT}/${IMPORT_EXECUTABLES_DIR}/ImportExecutables-${CMAKE_BUILD_TYPE}.cmake)
#  include(${IMPORT_EXECUTABLES})
#endif()

#set(MATERIAL_BINS)
#set(GENERATION_ROOT ${CMAKE_CURRENT_BINARY_DIR})
#file(MAKE_DIRECTORY "${GENERATION_ROOT}/generated/material/")

#set(MATC_TARGET desktop)

## Optimize materials for performance in Release mode.
#set(MATC_FLAGS -a all)
#if (CMAKE_BUILD_TYPE MATCHES Release)
#    set(MATC_FLAGS -O ${MATC_FLAGS})
#endif()

#foreach (mat_src ${MATERIAL_SRCS})
#    get_filename_component(localname "${mat_src}" NAME_WE)
#    get_filename_component(fullname "${mat_src}" ABSOLUTE)
#    set(output_path "${GENERATION_ROOT}/generated/material/${localname}.inc")
#    add_custom_command(
#            OUTPUT ${output_path}
#            COMMAND matc ${MATC_FLAGS} -p ${MATC_TARGET} -m material -f header
#            -o ${output_path} ${fullname}
#            MAIN_DEPENDENCY ${mat_src}
#            DEPENDS matc
#            COMMENT "Compiling material ${mat_src} to ${output_path}"
#    )
#    list(APPEND MATERIAL_BINS ${output_path})
#endforeach()

#add_custom_target(sample_materials DEPENDS ${MATERIAL_BINS})

#include_directories(${CMAKE_CURRENT_BINARY_DIR})



if(UNIX AND NOT APPLE AND NOT ANDROID AND NOT WEBGL)
  set(LINUX TRUE)
endif()

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
if (WIN32)
  list(APPEND srcs linux/NativeWindowHelperWindows.cpp)
  list(APPEND srcs win32/Path.cpp)
endif()
if (LINUX OR ANDROID)
  list(APPEND srcs linux/NativeWindowHelperLinux.cpp)
  list(APPEND srcs linux/Path.cpp)
endif()
if (APPLE)
  list(APPEND srcs darwin/NativeWindowHelperCocoa.mm)
  list(APPEND APP_LIBS "-framework Cocoa")
  list(APPEND srcs darwin/Path.cpp)
endif()

# Set local target name
set(target_name ${PROJECT_NAME}-gui-flmt)
set(component_name gui-flmt)

# Add target
dart_add_library(${target_name} ${hdrs} ${srcs} ${detail_hdrs} ${detail_srcs})
target_include_directories(
  ${target_name} SYSTEM
  PUBLIC
    ${filament_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)
target_link_libraries(
  ${target_name}
  PUBLIC
    dart
    dart-gui
    ${filament_LIBRARIES}
    ${SDL2_LIBRARIES}
    ${APP_LIBS}
)
target_compile_options(${target_name} PUBLIC -std=c++14)
#add_dependencies(${target_name} sample_materials)
target_compile_options(${target_name} PUBLIC "-lc++")
# Thread
#if(THREADS_HAVE_PTHREAD_ARG)
#  target_compile_options(${target_name} PUBLIC "-pthread")
#endif()
#if(CMAKE_THREAD_LIBS_INIT)
#  target_link_libraries(${target_name} PUBLIC ${CMAKE_THREAD_LIBS_INIT})
#endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} gui)

# Generate header for this namespace
dart_get_filename_components(header_names "gui_filament_headers headers" ${hdrs})
dart_generate_include_header_list(
  gui_filament_headers
  "dart/gui/filament/"
  "gui_filament_headers headers"
  ${header_names}
)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/filament.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/filament.hpp
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/filament.hpp
  DESTINATION include/dart/gui/filament
  COMPONENT headers
)

dart_format_add(${hdrs} ${detail_hdrs} ${srcs} ${detail_srcs})
