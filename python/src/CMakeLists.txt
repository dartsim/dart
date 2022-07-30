# Copyright (c) 2011-2022, The DART development contributors

execute_process(COMMAND ${Python_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT IS_ABSOLUTE PYTHON_SITE_PACKAGES)
  set(PYTHON_SITE_PACKAGES "${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE_PACKAGES}")
endif()

file(GLOB_RECURSE dartpy_headers "*.h" "*.hpp")
file(GLOB_RECURSE dartpy_sources "*.cpp")

# Build a Python extension module
pybind11_add_module(${pybind_module} MODULE ${dartpy_headers} ${dartpy_sources})

target_include_directories(
  ${pybind_module} SYSTEM PUBLIC ${Python_INCLUDE_DIRS} ${pybind11_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
  ${pybind_module}
  PUBLIC
    ${dart_component_targets}
    ${Python_LIBRARIES}
)

# Remove debug postfix for dartpy
set_target_properties(${pybind_module} PROPERTIES DEBUG_POSTFIX "")

# Get the path to the bind module
set(PYBIND_MODULE $<TARGET_FILE:${pybind_module}>)

# Custom target to install (copy) the bind module. This target may require
# `sudo` if the destination is a system directory.
set(install_comment "Installing ${pybind_module}...")
if(BUILD_SHARED_LIBS)
  string(
    CONCAT
      install_comment "${install_comment}\n"
      "NOTE: ${pybind_module} is built against the DART's shared libraries. "
      "Install the shared libraries to be able to import ${pybind_module}."
  )
endif()
install(TARGETS ${pybind_module}
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}/"
)

dart_clang_format_add(${dartpy_headers} ${dartpy_sources})
