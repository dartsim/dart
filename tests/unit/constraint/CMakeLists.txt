dart_add_test("unit" test_ContactSurface test_ContactSurface.cpp)
dart_add_test("unit" test_NonFiniteContact test_NonFiniteContact.cpp)
dart_add_test(
  "unit" test_CylindricalJointConstraint test_CylindricalJointConstraint.cpp
)

dart_add_test("unit" test_FrictionRows test_FrictionRows.cpp)
dart_add_test("unit" test_FrictionCone test_FrictionCone.cpp)
target_sources(
  test_FrictionCone
  PRIVATE ../../integration/AllocationCounting.cpp
)
if(DART_CODECOV)
  target_compile_definitions(test_FrictionCone PRIVATE DART_CODECOV=1)
endif()
dart_format_add(FrictionRegressionCases.hpp)
