# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

if(DART_PROFILE_BACKEND_REMOTERY)

  # Remotery requires DART_DOWNLOAD_DEPENDENT_PACKAGES to be ON
  if(NOT DART_DOWNLOAD_DEPENDENT_PACKAGES)
    message(FATAL_ERROR "DART_PROFILE_BACKEND_REMOTERY requires DART_DOWNLOAD_DEPENDENT_PACKAGES to be ON")
  endif()

  set(working_dir ${CMAKE_CURRENT_BINARY_DIR}/.deps)
  set(remotery_dir ${working_dir}/remotery)
  set(remotery_src_dir ${working_dir}/remotery-src)
  set(remotery_build_dir ${working_dir}/remotery-build)

  # Fetch Remotery
  dart_fetch_git_repo(
    PROJECT_NAME remotery
    WORKING_DIR ${working_dir}
    GIT_URL https://github.com/Celtoys/Remotery.git
    GIT_TAG v1.2.1
  )

  set(target_name dart${DART_MAJOR_VERSION}-profile)

  add_library(${target_name} STATIC
    ${remotery_src_dir}/lib/Remotery.c
  )
  target_include_directories(${target_name} PUBLIC
      $<BUILD_INTERFACE:${remotery_src_dir}/lib>
  )
  set_source_files_properties(
    ${remotery_src_dir}/lib/Remotery.h
    ${remotery_src_dir}/lib/Remotery.c
    PROPERTIES
      COMPILE_FLAGS "-w"
  )
  target_compile_options(${target_name} PRIVATE -w -fPIC)

  install(
    TARGETS ${target_name}
    EXPORT ${target_name}-targets
    PUBLIC_HEADER  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    INCLUDES       DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    RUNTIME        DESTINATION ${CMAKE_INSTALL_BINDIR}
    ARCHIVE        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY        DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
  install(
    EXPORT ${target_name}-targets
    FILE ${target_name}-targets.cmake
    DESTINATION
      ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}${DART_VERSION_MAJOR}
  )

  # Add a target to open the Remotery profiler in a web browser
  if(TARGET ${target_name})
    if(APPLE)
      set(open_command_name "open")
    else()
      set(open_command_name "xdg-open")
    endif()

    find_program(
      open_command
      NAMES ${open_command_name}
      DOC "Path to ${open_command_name}"
    )
    set(index_path ${remotery_src_dir}/vis/index.html)
    if(open_command)
      add_custom_target(
        open_profiler
        COMMAND ${open_command} ${index_path}
        COMMENT "Opening Web profiler in a web browser: ${open_command} ${index_path}"
      )
    endif()
  endif()

endif()
