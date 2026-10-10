cmake_minimum_required(VERSION 3.22.1)
project(DARTEmbeddedConsumer LANGUAGES C CXX)

get_filename_component(
  DART_SOURCE_ROOT
  "../../.."
  ABSOLUTE
  BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
)
list(APPEND CMAKE_MODULE_PATH "${DART_SOURCE_ROOT}/cmake")
set(DART_USE_SYSTEM_BULLET ON)
set(DART_USE_SYSTEM_ODE ON)
if(DART_TEST_PARENT_ALIASES)
  find_package(Bullet MODULE REQUIRED)
  add_library(parent_bullet INTERFACE IMPORTED)
  set_target_properties(
    parent_bullet
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${BULLET_LIBRARIES}"
  )
  add_library(Bullet ALIAS parent_bullet)
  find_package(ODE MODULE REQUIRED)
  add_library(parent_ode INTERFACE IMPORTED)
  set_target_properties(
    parent_ode
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${ODE_LIBRARIES}"
  )
  add_library(ODE::ODE ALIAS parent_ode)
endif()
include(DARTFindBullet)
include(DARTFindODE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(DART_BUILD_IO OFF CACHE BOOL "" FORCE)
set(DART_BUILD_GUI OFF CACHE BOOL "" FORCE)
set(DART_BUILD_DARTPY OFF CACHE BOOL "" FORCE)
add_subdirectory("${DART_SOURCE_ROOT}" dart)
add_executable(embedded_core ../core.cpp)
target_link_libraries(embedded_core PRIVATE dart)
