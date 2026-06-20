dart_add_test(
  "unit" test_DantzigNative test_DantzigNative.cpp
)

target_link_libraries(test_DantzigNative dart-baseline-ode)

dart_add_test(
  "unit" test_DantzigHeaderCompatibility
  test_DantzigHeaderCompatibility.cpp
)
