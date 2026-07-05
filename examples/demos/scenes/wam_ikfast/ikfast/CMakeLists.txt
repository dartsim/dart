# Builds the WAM arm's ikfast-generated analytical IK code as its own shared
# library, dlopen'd at runtime by dart::dynamics::SharedLibraryIkFast (see
# ../WamIkFastScene.cpp). This is a straight copy of the generated code from
# examples/wam_ikfast/ikfast/ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp --
# duplicated (not shared via the original example's own `wamIk` CMake target)
# so this scene has no build dependency on examples/wam_ikfast/, which
# Phase 5 of the consolidated-demos plan may retire.
#
# The target/library name is "dartDemosWamIk" rather than the original
# example's "wamIk" purely to avoid a duplicate-target-name / duplicate-
# output-file collision with examples/wam_ikfast/ikfast's own `wamIk` target,
# which lives in the same CMake project tree until Phase 5 retires it.

set(CMAKE_DEBUG_POSTFIX "d")

add_library(
  dart_demos_wam_ikfast
  SHARED
  ikfast71.Transform6D.4_6_9_10_11_12_f8.cpp
)
target_link_libraries(dart_demos_wam_ikfast PUBLIC dart)
target_compile_definitions(
  dart_demos_wam_ikfast
  PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
)
target_compile_options(dart_demos_wam_ikfast PRIVATE -w)
# dart_demos_ikfast_output_dir is set by the parent examples/demos/CMakeLists.txt
# (to that project's own build dir) before this subdirectory is added, so this
# child scope inherits it. WamIkFastScene.cpp is compiled with the same path as
# DART_DEMOS_IKFAST_DIR so its dlopen lookup finds the library here, identically
# in the in-source and standalone builds.
set_target_properties(
  dart_demos_wam_ikfast
  PROPERTIES
    OUTPUT_NAME "dartDemosWamIk"
    LIBRARY_OUTPUT_DIRECTORY "${dart_demos_ikfast_output_dir}"
    ARCHIVE_OUTPUT_DIRECTORY "${dart_demos_ikfast_output_dir}"
)
