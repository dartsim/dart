# Copyright (c) 2011, The DART development contributors

# Reference collision libraries use static Registrar objects to keep legacy
# factory keys available during comparison tests. Tests that do not directly
# reference symbols from these libraries need --no-as-needed to prevent the
# linker from stripping them.
# Force-link a reference collision shared library even if no symbols are
# directly referenced. Without this, --as-needed (default on Linux) strips the
# library and prevents its static Registrar from running.
#
# Usage:
#   dart_force_link_collision_backend(target backend)          # plain mode
#   dart_force_link_collision_backend(target backend PRIVATE)  # keyword mode
function(dart_force_link_collision_backend target backend)
  if(NOT TARGET ${backend})
    return()
  endif()
  set(keyword "${ARGV2}") # optional: PRIVATE, PUBLIC, etc.
  if(MSVC OR APPLE)
    if(keyword)
      target_link_libraries(${target} ${keyword} ${backend})
    else()
      target_link_libraries(${target} ${backend})
    endif()
  else()
    if(keyword)
      target_link_libraries(
        ${target}
        ${keyword}
        -Wl,--no-as-needed
        ${backend}
        -Wl,--as-needed
      )
    else()
      target_link_libraries(
        ${target}
        -Wl,--no-as-needed
        ${backend}
        -Wl,--as-needed
      )
    endif()
  endif()
endfunction()

# ==============================================================================
# Collision Tests
# ==============================================================================
dart_add_test("integration" INTEGRATION_collision_Issue2470 collision/test_issue2470.cpp)

if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-test-reference-fcl)
  dart_add_test(
    "integration"
    INTEGRATION_collision_MeshContactRegression
    collision/test_mesh_contact_regression.cpp
  )
  target_link_libraries(
    INTEGRATION_collision_MeshContactRegression
    dart-test-reference-fcl
  )
endif()
if(DART_BUILD_COLLISION_REFERENCE_TESTS AND TARGET dart-test-reference-fcl)
  dart_add_test(
    "integration"
    INTEGRATION_collision_FclPrimitiveContactMatrix
    collision/test_fcl_primitive_contact_matrix.cpp
  )
  target_link_libraries(
    INTEGRATION_collision_FclPrimitiveContactMatrix
    dart-test-reference-fcl
  )
  dart_add_test(
    "integration"
    INTEGRATION_collision_FclPrimitiveContactMatrixFcl
    collision/test_fcl_primitive_contact_matrix_fcl.cpp
  )
  target_link_libraries(
    INTEGRATION_collision_FclPrimitiveContactMatrixFcl
    dart-test-reference-fcl
  )
  # Suppress <ciso646> deprecation warning from octomap headers (GCC 15+)
  # This file directly includes <fcl/fcl.h> which pulls in octomap's OcTreeKey.h
  if(
    CMAKE_CXX_COMPILER_ID STREQUAL "GNU"
    AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 15
  )
    target_compile_options(
      INTEGRATION_collision_FclPrimitiveContactMatrixFcl
      PRIVATE -Wno-cpp
    )
  endif()
endif()

dart_add_test(
  "integration"
  INTEGRATION_collision_native_collision_consistency
  collision/test_native_collision_consistency.cpp
)
target_link_libraries(
  INTEGRATION_collision_native_collision_consistency
  dart-collision-native
)
if(DART_BUILD_COLLISION_REFERENCE_TESTS)
  target_compile_definitions(
    INTEGRATION_collision_native_collision_consistency
    PRIVATE DART_ENABLE_COLLISION_REFERENCE_TESTS=1
  )
  if(TARGET dart-test-reference-fcl)
    target_link_libraries(
      INTEGRATION_collision_native_collision_consistency
      dart-test-reference-fcl
    )
  endif()
  if(TARGET dart-test-reference-bullet)
    target_link_libraries(
      INTEGRATION_collision_native_collision_consistency
      dart-test-reference-bullet
    )
  endif()
  if(TARGET dart-test-reference-ode)
    target_link_libraries(
      INTEGRATION_collision_native_collision_consistency
      dart-test-reference-ode
    )
  endif()
endif()

if(
  DART_BUILD_COLLISION_REFERENCE_TESTS
  AND TARGET dart-test-reference-bullet
  AND TARGET dart-test-reference-ode
  AND TARGET dart-test-reference-fcl
)
  dart_add_test(
    "integration"
    INTEGRATION_collision_BoxGroundContactParity
    collision/test_box_ground_contact_parity.cpp
  )
  target_link_libraries(
    INTEGRATION_collision_BoxGroundContactParity
    dart-collision-native
    dart-test-reference-bullet
    dart-test-reference-ode
    dart-test-reference-fcl
  )
endif()

# ==============================================================================
# Constraint Tests
# ==============================================================================
# DART 6 constraint-world parity tests live on release-6.*.

# ==============================================================================
# Dynamics Tests
# ==============================================================================
if(TARGET dart-utils-urdf AND TARGET dart-io)
  dart_add_test("integration" INTEGRATION_dynamics_ForwardKinematics dynamics/test_forward_kinematics.cpp)
  target_link_libraries(INTEGRATION_dynamics_ForwardKinematics dart-io)

  dart_add_test("integration" INTEGRATION_dynamics_AtlasIK dynamics/test_atlas_ik.cpp)
  target_link_libraries(INTEGRATION_dynamics_AtlasIK dart-io)
endif()

# ==============================================================================
# IO Tests
# ==============================================================================
if(TARGET dart-utils)
  if(DART_ENABLE_SDFORMAT)
    dart_add_test("integration" INTEGRATION_io_SdfParser io/test_sdf_parser.cpp)
    target_link_libraries(INTEGRATION_io_SdfParser dart-utils)
    if(TARGET dart-io)
      target_link_libraries(INTEGRATION_io_SdfParser dart-io)
    endif()

    dart_add_test("integration" INTEGRATION_io_SdfWriter io/test_sdf_writer.cpp)
    target_link_libraries(INTEGRATION_io_SdfWriter dart-utils)
  endif()
endif()

if(TARGET dart-io)
  dart_add_test("integration" INTEGRATION_io_Read io/test_read.cpp)
  target_link_libraries(INTEGRATION_io_Read dart-io)
endif()

# OpenUSD loader integration test. Only built when DART_BUILD_IO_USD=ON (which
# requires OpenUSD/pxr); it compiles out cleanly in the default build. The
# loader is reached through the dart::simulation::io::addSkeleton front door, so
# the test links dart-simulation in addition to dart-io.
if(DART_BUILD_IO_USD AND TARGET dart-io)
  dart_add_test("integration" INTEGRATION_io_Usd io/test_usd_parser.cpp)
  target_link_libraries(INTEGRATION_io_Usd dart-io dart-simulation)
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" INTEGRATION_io_UrdfParser io/test_urdf_parser.cpp)
    target_link_libraries(INTEGRATION_io_UrdfParser dart-utils-urdf)
  endif()

  # Additional IO tests
  if(TARGET dart-io)
    dart_add_test("integration" INTEGRATION_io_UrdfMaterialParsing io/test_urdf_material_parsing.cpp)
    target_link_libraries(INTEGRATION_io_UrdfMaterialParsing dart-io)
  endif()

  dart_add_library(SharedLibraryWamIkFast io/shared_library_wam_ik_fast.hpp io/shared_library_wam_ik_fast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(
    SharedLibraryWamIkFast
    PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
  )
  if(NOT MSVC)
    # Suppress warnings in auto-generated IKFast code without fighting MSVC's /W
    target_compile_options(SharedLibraryWamIkFast PRIVATE -w)
  endif()

  dart_add_library(GeneratedWamIkFast io/generated_wam_ik_fast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(
    GeneratedWamIkFast
    PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
  )
  if(NOT MSVC)
    target_compile_options(GeneratedWamIkFast PRIVATE -w)
  endif()

  if(BUILD_SHARED_LIBS AND TARGET dart-io)
    dart_add_test("integration" INTEGRATION_io_IkFast io/test_ik_fast.cpp)
    target_link_libraries(INTEGRATION_io_IkFast dart-io)
    add_dependencies(
      INTEGRATION_io_IkFast
      GeneratedWamIkFast
      SharedLibraryWamIkFast
    )
    target_compile_definitions(
      INTEGRATION_io_IkFast
      PRIVATE DART_IKFAST_GENERATED_LIB_PATH="$<TARGET_FILE:GeneratedWamIkFast>"
    )
  endif()
endif()

dart_format_add(
  io/generated_wam_ik_fast.cpp
  io/shared_library_wam_ik_fast.hpp
  io/shared_library_wam_ik_fast.cpp
)

# ==============================================================================
# Optimization Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    optimization/test_inverse_kinematics.cpp
)

dart_add_test("integration" INTEGRATION_optimization_Optimizer optimization/test_optimizer.cpp)

# ==============================================================================
# Utils Tests
# ==============================================================================
dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    utils/test_aspect.cpp
    utils/test_frames.cpp
    utils/test_signal.cpp
)

dart_add_test("integration" INTEGRATION_utils_LocalResourceRetriever utils/test_local_resource_retriever.cpp)
dart_add_test("integration" INTEGRATION_utils_Subscriptions utils/test_subscriptions.cpp)

if(TARGET dart-utils)
  dart_add_test("integration" INTEGRATION_utils_CompositeResourceRetriever utils/test_composite_resource_retriever.cpp)
  target_link_libraries(INTEGRATION_utils_CompositeResourceRetriever dart-utils)

  dart_add_test("integration" INTEGRATION_utils_DartResourceRetriever utils/test_dart_resource_retriever.cpp)
  target_link_libraries(INTEGRATION_utils_DartResourceRetriever dart-utils)

  dart_add_test("integration" INTEGRATION_utils_PackageResourceRetriever utils/test_package_resource_retriever.cpp)
  target_link_libraries(INTEGRATION_utils_PackageResourceRetriever dart-utils)
endif()
