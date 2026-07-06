#!/usr/bin/env python3

"""
Test script to verify dartpy installation works correctly.

This script performs basic sanity checks on a dartpy installation:
1. Import dartpy module
2. Check basic functionality
3. Verify key components are available
"""

import sys


def test_import():
    """Test that dartpy can be imported."""
    try:
        import dartpy  # type: ignore

        print("✓ dartpy module imported successfully")
        print(f"  Location: {dartpy.__file__}")
        return True
    except ImportError as e:
        print(f"✗ Failed to import dartpy: {e}")
        return False


def test_basic_functionality():
    """Test basic dartpy functionality."""
    try:
        import dartpy as dart

        # dartpy.World is the DART 7 ECS World, promoted to the flat namespace
        # from the simulation module. Wheels are expected to ship it on every
        # platform, including Windows.
        if hasattr(dart, "World"):
            world = dart.World()
            print(f"✓ Created World: {world}")

            skeleton = dart.Skeleton("box")
            skeleton.create_free_joint_and_body_node_pair()
            print(f"✓ Created Skeleton: {skeleton.get_name()}")

            # The ECS World ingests a dynamics Skeleton via the module-level
            # add_skeleton(world, skeleton) -> Multibody facade function (not a
            # World method); stepping then advances the ECS simulation.
            multibody = dart.add_skeleton(world, skeleton)
            print(f"✓ Added skeleton as multibody with {multibody.num_dofs} DoF(s)")

            world.step()
            print("✓ Stepped world")
            return True

        print(
            "✗ dartpy.World is missing; dartpy wheels must include the "
            "DART 7 simulation module"
        )
        return False
    except ImportError as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        return False


def test_available_modules():
    """Check which dartpy modules are available."""
    try:
        import dartpy  # type: ignore

        modules = [
            "common",
            "math",
            "dynamics",
            "collision",
            "constraint",
            "io",
            "simulation",
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
    print("Testing dartpy installation...\n")

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
