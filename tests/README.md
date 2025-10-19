# DART Test Suite

For comprehensive documentation on the DART test suite, including:
- Test organization and structure
- Unit vs Integration test guidelines
- Adding new tests
- Running and debugging tests
- Best practices

**Please see: [`docs/onboarding/testing.md`](../docs/onboarding/testing.md)**

## Quick Reference

### Run all tests:
```bash
cd build
ctest
```

### Run tests by category:
```bash
ctest -L integration
ctest -L unit
ctest -L regression
```

### Run a specific test:
```bash
ctest -R test_Collision
```

### Run tests in parallel:
```bash
ctest -j8  # Run 8 tests in parallel
```
