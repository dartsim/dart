# Copyright (c) 2011, The DART development contributors

dart_build_tests(
  TYPE integration
  TARGET_PREFIX INTEGRATION
  LINK_LIBRARIES dart
  SOURCES
    test_Aspect.cpp
    test_Building.cpp
    test_Common.cpp
    test_Concurrency.cpp
    test_Constraint.cpp
    test_Frames.cpp
    test_Friction.cpp
    test_InvalidDynamicsInputs.cpp
    test_InverseKinematics.cpp
    test_NameManagement.cpp
    test_Signal.cpp
    test_StepAllocation.cpp
)
target_sources(INTEGRATION_StepAllocation PRIVATE AllocationCounting.cpp)
target_link_libraries(INTEGRATION_StepAllocation PRIVATE dart-utils)
if(DART_CODECOV)
  # Coverage instrumentation allocates; keep the allocation counters inert so
  # coverage runs do not report gcov-runtime allocations as DART allocations.
  target_compile_definitions(INTEGRATION_StepAllocation PRIVATE DART_CODECOV=1)
endif()
dart_format_add(AllocationCounting.cpp AllocationCounting.hpp)

dart_add_test("integration" test_CollisionGroups)
foreach(collision_engine dart-collision-bullet dart-collision-ode)
  if(TARGET ${collision_engine})
    target_link_libraries(test_CollisionGroups ${collision_engine})
  endif()
endforeach()
dart_add_test("integration" test_ConstraintSolver)
target_sources(test_ConstraintSolver PRIVATE AllocationCounting.cpp)
if(DART_CODECOV)
  target_compile_definitions(test_ConstraintSolver PRIVATE DART_CODECOV=1)
endif()
set(
  fbf_author_turntable_spec_source
  "${DART_SOURCE_DIR}/examples/demos/scenes/FbfAuthorTurntableSpec.hpp"
)
set(
  fbf_author_painleve_spec_source
  "${DART_SOURCE_DIR}/examples/demos/scenes/FbfAuthorPainleveSpec.hpp"
)
set(
  fbf_author_card_house_spec_source
  "${DART_SOURCE_DIR}/examples/demos/scenes/FbfAuthorCardHouseSpec.hpp"
)
set(
  fbf_author_masonry_arch_spec_source
  "${DART_SOURCE_DIR}/examples/demos/scenes/FbfAuthorMasonryArchSpec.hpp"
)
set(
  fbf_exact_solver_options_source
  "${DART_SOURCE_DIR}/dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
)
set_property(
  DIRECTORY
  APPEND
  PROPERTY
    CMAKE_CONFIGURE_DEPENDS
      "${fbf_author_turntable_spec_source}"
      "${fbf_author_painleve_spec_source}"
      "${fbf_author_card_house_spec_source}"
      "${fbf_author_masonry_arch_spec_source}"
      "${fbf_exact_solver_options_source}"
)
file(
  SHA256 "${fbf_author_turntable_spec_source}"
  fbf_author_turntable_spec_sha256
)
file(
  SHA256 "${fbf_author_painleve_spec_source}"
  fbf_author_painleve_spec_sha256
)
file(
  SHA256 "${fbf_author_card_house_spec_source}"
  fbf_author_card_house_spec_sha256
)
file(
  SHA256 "${fbf_author_masonry_arch_spec_source}"
  fbf_author_masonry_arch_spec_sha256
)
file(
  SHA256 "${fbf_exact_solver_options_source}"
  fbf_exact_solver_options_sha256
)
dart_add_test("integration" test_ExactCoulombFbfPaperFixtures)
target_compile_definitions(
  test_ExactCoulombFbfPaperFixtures
  PRIVATE
    DART_FBF_AUTHOR_TURNTABLE_SPEC_SHA256="${fbf_author_turntable_spec_sha256}"
    DART_FBF_AUTHOR_PAINLEVE_SPEC_SHA256="${fbf_author_painleve_spec_sha256}"
    DART_FBF_AUTHOR_CARD_HOUSE_SPEC_SHA256="${fbf_author_card_house_spec_sha256}"
    DART_FBF_EXACT_SOLVER_OPTIONS_SHA256="${fbf_exact_solver_options_sha256}"
)
dart_add_test("integration" test_FbfAuthorMasonryArchSpec)
target_compile_definitions(
  test_FbfAuthorMasonryArchSpec
  PRIVATE
    DART_FBF_AUTHOR_MASONRY_ARCH_SPEC_SHA256="${fbf_author_masonry_arch_spec_sha256}"
)
dart_add_test("integration" test_ContactConstraint)
dart_add_test("integration" test_SplitImpulse)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_SplitImpulse dart-collision-bullet)
  target_link_libraries(
    INTEGRATION_StepAllocation
    PRIVATE dart-collision-bullet
  )
endif()
if(TARGET dart-collision-ode)
  target_link_libraries(INTEGRATION_StepAllocation PRIVATE dart-collision-ode)
endif()
dart_add_test("integration" test_IslandDeactivation)
dart_add_test("integration" test_Issue719)
dart_add_test("integration" test_GenericJoints)
dart_add_test("integration" test_Inertia)
dart_add_test("integration" test_Lemke)
dart_add_test("integration" test_LocalResourceRetriever)
dart_add_test("integration" test_Optimizer)
dart_add_test("integration" test_ScrewJoint)
dart_add_test("integration" test_Subscriptions)

if(TARGET dart-utils)
  dart_add_test("integration" test_CompositeResourceRetriever)
  target_link_libraries(test_CompositeResourceRetriever dart-utils)

  dart_add_test("integration" test_DartResourceRetriever)
  target_link_libraries(test_DartResourceRetriever dart-utils)

  dart_add_test("integration" test_FileInfoWorld)
  target_link_libraries(test_FileInfoWorld dart-utils)

  dart_add_test("integration" test_PackageResourceRetriever)
  target_link_libraries(test_PackageResourceRetriever dart-utils)

  dart_add_test("integration" test_MjcfParser)
  target_link_libraries(test_MjcfParser dart-utils)

  dart_add_test("integration" test_SdfParser)
  target_link_libraries(test_SdfParser dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_SdfParser dart-collision-bullet)
  endif()

  dart_add_test("integration" test_MimicConstraint)
  target_link_libraries(test_MimicConstraint dart-utils)

  dart_add_test("integration" test_SkelParser)
  target_link_libraries(test_SkelParser dart-utils)

  if(NOT MSVC)
    dart_add_test("integration" test_VskParser)
    target_link_libraries(test_VskParser dart-utils)
  endif()
endif()

if(TARGET dart-utils-urdf)
  if(NOT MSVC)
    dart_add_test("integration" test_DartLoader)
    target_link_libraries(test_DartLoader dart-utils-urdf)
  endif()

  dart_add_library(SharedLibraryWamIkFast SharedLibraryWamIkFast.hpp SharedLibraryWamIkFast.cpp)
  target_link_libraries(SharedLibraryWamIkFast PUBLIC dart)
  target_compile_definitions(
    SharedLibraryWamIkFast
    PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
  )
  target_compile_options(SharedLibraryWamIkFast PRIVATE -w)

  dart_add_library(GeneratedWamIkFast GeneratedWamIkFast.cpp)
  target_link_libraries(GeneratedWamIkFast PUBLIC dart)
  target_compile_definitions(
    GeneratedWamIkFast
    PUBLIC IKFAST_NO_MAIN IKFAST_CLIBRARY
  )
  target_compile_options(GeneratedWamIkFast PRIVATE -w)

  if(BUILD_SHARED_LIBS)
    dart_add_test("integration" test_IkFast)
    target_link_libraries(test_IkFast dart-utils-urdf)
    add_dependencies(test_IkFast GeneratedWamIkFast SharedLibraryWamIkFast)
  endif()
endif()

dart_format_add(
  GeneratedWamIkFast.cpp
  SharedLibraryWamIkFast.hpp
  SharedLibraryWamIkFast.cpp
)

dart_add_test("integration" test_Distance)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_Distance dart-collision-bullet)
endif()

dart_add_test("integration" test_Raycast)
if(TARGET dart-collision-bullet)
  target_link_libraries(test_Raycast dart-collision-bullet)
endif()

if(TARGET dart-collision-ode)
  # This test doesn't work with FCL because it converts simple shapes to meshes, which makes
  # it difficult to come up with correct test expectations.
  dart_add_test("integration" test_ForceDependentSlip)
  target_link_libraries(test_ForceDependentSlip dart-collision-ode)
endif()

if(TARGET dart-collision-bullet AND TARGET dart-collision-ode)
  dart_add_test("integration" test_CapsuleGroundContact)
  target_link_libraries(
    test_CapsuleGroundContact
    dart-collision-bullet
    dart-collision-ode
  )
endif()

if(TARGET dart-utils)
  dart_add_test("integration" test_Collision)
  target_link_libraries(test_Collision dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_Collision dart-collision-bullet)
  endif()
  if(TARGET dart-collision-ode)
    target_link_libraries(test_Collision dart-collision-ode)
  endif()

  dart_add_test("integration" test_Dynamics)
  target_link_libraries(test_Dynamics dart-utils)

  dart_add_test("integration" test_Joints)
  target_link_libraries(test_Joints dart-utils)

  dart_add_test("integration" test_JointForceTorque)
  target_link_libraries(test_JointForceTorque dart-utils)

  dart_add_test("integration" test_Skeleton)
  target_link_libraries(test_Skeleton dart-utils)

  dart_add_test("integration" test_MetaSkeleton)
  target_link_libraries(test_MetaSkeleton dart-utils)

  dart_add_test("integration" test_SoftDynamics)
  target_link_libraries(test_SoftDynamics dart-utils)

  dart_add_test("integration" test_World)
  target_link_libraries(test_World dart-utils)
  if(TARGET dart-collision-bullet)
    target_link_libraries(test_World dart-collision-bullet)
  endif()

  if(TARGET dart-utils-urdf)
    dart_add_test("integration" test_ForwardKinematics)
    target_link_libraries(test_ForwardKinematics dart-utils-urdf)
  endif()
endif()
