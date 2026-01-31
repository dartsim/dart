set(target_name ${PROJECT_NAME}-gui-raylib)

# When building in-source, raylib is already available via FetchContent.
# Only call find_package for out-of-source (standalone) builds.
if(NOT TARGET raylib AND NOT TARGET raylib::raylib)
  find_package(raylib CONFIG REQUIRED)
endif()

if(NOT TARGET raylib::raylib)
  if(TARGET raylib)
    add_library(raylib::raylib ALIAS raylib)
  endif()
endif()

file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

dart_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC dart-gui raylib::raylib)

install(FILES ${hdrs} DESTINATION include/dart/gui/raylib COMPONENT headers)

dart_format_add(${hdrs} ${srcs})
