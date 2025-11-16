#!/usr/bin/env python3

"""
Test script to verify dartpy_nb installation works correctly.

This script performs basic sanity checks on a dartpy_nb installation:
1. Import dartpy_nb module
2. Check basic functionality
3. Verify key components are available
"""

import sys


def test_import():
    """Test that dartpy_nb can be imported."""
    try:
        import dartpy_nb  # type: ignore

        print("✓ dartpy_nb module imported successfully")
        print(f"  Location: {dartpy_nb.__file__}")
        return True
    except ImportError as e:
        print(f"✗ Failed to import dartpy_nb: {e}")
        return False


def test_basic_functionality():
    """Test basic dartpy_nb functionality."""
    try:
        import dartpy_nb as dart

        # Test creating a world
        world = dart.simulation.World()
        print(f"✓ Created World: {world}")

        # Test creating a skeleton
        skel = dart.dynamics.Skeleton()
        skel.setName("test_skeleton")
        print(f"✓ Created Skeleton: {skel.getName()}")

        # Test adding skeleton to world
        world.addSkeleton(skel)
        print(f"✓ Added skeleton to world: {world.getNumSkeletons()} skeleton(s)")

        return True
    except ImportError as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False


def test_available_modules():
    """Check which dartpy_nb modules are available."""
    try:
        import dartpy_nb as dartpy  # type: ignore

        modules = [
            "common",
            "math",
            "dynamics",
            "collision",
            "constraint",
            "simulation",
            "utils",
        ]

        available = []
        missing = []

        for module_name in modules:
            if hasattr(dartpy, module_name):
                available.append(module_name)
            else:
                missing.append(module_name)

        print(f"✓ Available modules ({len(available)}): {', '.join(available)}")
        if missing:
            print(f"  Missing modules ({len(missing)}): {', '.join(missing)}")

        return len(available) > 0
    except ImportError as e:
        print(f"✗ Cannot check modules: {e}")
        return False


def main():
    """Run all tests."""
    print("Testing dartpy_nb installation...\n")

    tests = [
        ("Import", test_import),
        ("Basic Functionality", test_basic_functionality),
        ("Available Modules", test_available_modules),
    ]

    results = []
    for name, test_func in tests:
        print(f"\n{name} Test:")
        print("-" * 40)
        result = test_func()
        results.append((name, result))

    # Summary
    print("\n" + "=" * 40)
    print("Test Summary:")
    print("=" * 40)
    passed = sum(1 for _, result in results if result)
    total = len(results)

    for name, result in results:
        status = "PASS" if result else "FAIL"
        symbol = "✓" if result else "✗"
        print(f"{symbol} {name}: {status}")

    print(f"\nTotal: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All tests passed!")
        return 0
    else:
        print(f"\n❌ {total - passed} test(s) failed")
        return 1


if __name__ == "__main__":
    sys.exit(main())
