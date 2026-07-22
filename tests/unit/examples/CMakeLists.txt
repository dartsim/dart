dart_add_test("unit" test_HeadlessExactFbfFailFast)
dart_add_test("unit" test_FbfLiteralMasonryArchSpec)

set(
  fbf_author_incline_spec_source
  "${DART_SOURCE_DIR}/examples/demos/scenes/FbfAuthorInclineSpec.hpp"
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
      "${fbf_author_incline_spec_source}"
      "${fbf_exact_solver_options_source}"
)
file(SHA256 "${fbf_author_incline_spec_source}" fbf_author_incline_spec_sha256)
file(
  SHA256 "${fbf_exact_solver_options_source}"
  fbf_exact_solver_options_sha256
)
dart_add_test("unit" test_FbfAuthorInclineSpec)
target_compile_definitions(
  test_FbfAuthorInclineSpec
  PRIVATE
    DART_FBF_AUTHOR_INCLINE_SPEC_SHA256="${fbf_author_incline_spec_sha256}"
    DART_FBF_EXACT_SOLVER_OPTIONS_SHA256="${fbf_exact_solver_options_sha256}"
)
