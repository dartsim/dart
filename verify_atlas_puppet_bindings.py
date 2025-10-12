#!/usr/bin/env python3
"""
Comprehensive verification script for atlas_puppet Python bindings.
This script ACTUALLY RUNS the code paths that the example uses.
"""

import sys

import numpy as np

# Add build path
sys.path.insert(0, 'build/default/cpp/Release/python/dartpy')

import dartpy as dart


def test_endeffector_bindings():
    """Test all EndEffector bindings used in the example."""
    print("=" * 70)
    print("Testing EndEffector Bindings")
    print("=" * 70)

    # Load Atlas
    urdf = dart.utils.DartLoader()
    atlas = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    # Test createEndEffector
    print("\n1. Testing createEndEffector()...")
    l_foot = atlas.getBodyNode("l_foot").createEndEffector("test_foot")
    assert l_foot is not None, "createEndEffector failed"
    print("   ✓ createEndEffector() works")

    # Test setRelativeTransform
    print("\n2. Testing setRelativeTransform()...")
    tf = dart.math.Isometry3()
    tf.set_translation([0.186, 0.0, -0.08])
    l_foot.setRelativeTransform(tf)
    print("   ✓ setRelativeTransform() works")

    # Test getRelativeTransform
    print("\n3. Testing getRelativeTransform()...")
    tf_back = l_foot.getRelativeTransform()
    assert np.allclose(tf_back.translation(), [0.186, 0.0, -0.08]), "Transform mismatch"
    print("   ✓ getRelativeTransform() works")

    # Test getWorldTransform
    print("\n4. Testing getWorldTransform()...")
    world_tf = l_foot.getWorldTransform()
    assert world_tf is not None, "getWorldTransform failed"
    print("   ✓ getWorldTransform() works")

    # Test setDefaultRelativeTransform
    print("\n5. Testing setDefaultRelativeTransform()...")
    tf_hand = dart.math.Isometry3()
    tf_hand.set_translation([0.0009, 0.1254, 0.012])
    l_hand = atlas.getBodyNode("l_hand").createEndEffector("test_hand")
    l_hand.setDefaultRelativeTransform(tf_hand, True)
    print("   ✓ setDefaultRelativeTransform() works")

    # Test getIK
    print("\n6. Testing getIK()...")
    ik = l_hand.getIK(True)
    assert ik is not None, "getIK failed"
    print("   ✓ getIK() works")

    # Test createSupport
    print("\n7. Testing createSupport()...")
    support = l_foot.createSupport()
    assert support is not None, "createSupport failed"
    print("   ✓ createSupport() works")

    # Test Support.setGeometry
    print("\n8. Testing Support.setGeometry()...")
    geometry = [
        np.array([0.1, 0.1, 0.0]),
        np.array([0.1, -0.1, 0.0]),
        np.array([-0.1, -0.1, 0.0]),
        np.array([-0.1, 0.1, 0.0])
    ]
    support.setGeometry(geometry)
    print("   ✓ Support.setGeometry() works")

    # Test Support.setActive
    print("\n9. Testing Support.setActive()...")
    support.setActive(True)
    print("   ✓ Support.setActive() works")

    print("\n✅ All EndEffector bindings work!")
    return True


def test_ik_bindings():
    """Test all IK bindings used in the example."""
    print("\n" + "=" * 70)
    print("Testing IK Bindings")
    print("=" * 70)

    # Load Atlas
    urdf = dart.utils.DartLoader()
    atlas = urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    # Create end effector
    l_hand = atlas.getBodyNode("l_hand").createEndEffector("test_hand")

    # Test InteractiveFrame creation
    print("\n1. Testing InteractiveFrame creation...")
    target = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), "test_target")
    assert target is not None, "InteractiveFrame creation failed"
    print("   ✓ InteractiveFrame works")

    # Test IK.setTarget
    print("\n2. Testing IK.setTarget()...")
    ik = l_hand.getIK(True)
    ik.setTarget(target)
    print("   ✓ IK.setTarget() works")

    # Test IK.useWholeBody
    print("\n3. Testing IK.useWholeBody()...")
    ik.useWholeBody()
    print("   ✓ IK.useWholeBody() works")

    # Test IK.getGradientMethod
    print("\n4. Testing IK.getGradientMethod()...")
    grad_method = ik.getGradientMethod()
    assert grad_method is not None, "getGradientMethod failed"
    print("   ✓ IK.getGradientMethod() works")

    # Test GradientMethod.setComponentWeights
    print("\n5. Testing GradientMethod.setComponentWeights()...")
    weights = 0.01 * np.ones(6)
    grad_method.setComponentWeights(weights)
    print("   ✓ GradientMethod.setComponentWeights() works")

    # Test IK.getErrorMethod
    print("\n6. Testing IK.getErrorMethod()...")
    error_method = ik.getErrorMethod()
    assert error_method is not None, "getErrorMethod failed"
    print("   ✓ IK.getErrorMethod() works")

    # Test ErrorMethod.setLinearBounds
    print("\n7. Testing ErrorMethod.setLinearBounds()...")
    bounds = np.full(3, np.inf)
    error_method.setLinearBounds(-bounds, bounds)
    print("   ✓ ErrorMethod.setLinearBounds() works")

    # Test ErrorMethod.setAngularBounds
    print("\n8. Testing ErrorMethod.setAngularBounds()...")
    error_method.setAngularBounds(-bounds, bounds)
    print("   ✓ ErrorMethod.setAngularBounds() works")

    # Test IK.solveAndApply
    print("\n9. Testing IK.solveAndApply()...")
    result = atlas.getOrCreateIK().solveAndApply()
    print(f"   ✓ IK.solveAndApply() works (result={result})")

    print("\n✅ All IK bindings work!")
    return True


def test_worldnode_bindings():
    """Test RealTimeWorldNode can be subclassed."""
    print("\n" + "=" * 70)
    print("Testing WorldNode Bindings")
    print("=" * 70)

    # Test RealTimeWorldNode subclassing
    print("\n1. Testing RealTimeWorldNode subclassing...")

    class TestWorldNode(dart.gui.osg.RealTimeWorldNode):
        def __init__(self, world):
            super().__init__(world)
            self.called = False

        def customPreRefresh(self):
            self.called = True

    world = dart.simulation.World()
    node = TestWorldNode(world)
    assert node is not None, "RealTimeWorldNode subclass failed"
    print("   ✓ RealTimeWorldNode can be subclassed")

    print("\n✅ WorldNode bindings work!")
    return True


def main():
    """Run all verification tests."""
    print()
    print("#" * 70)
    print("#  ATLAS PUPPET PYTHON BINDINGS VERIFICATION")
    print("#" * 70)
    print()
    print("This script verifies that ALL bindings used by atlas_puppet work.")
    print()

    all_passed = True

    try:
        all_passed &= test_endeffector_bindings()
    except Exception as e:
        print(f"\n❌ EndEffector bindings FAILED: {e}")
        import traceback
        traceback.print_exc()
        all_passed = False

    try:
        all_passed &= test_ik_bindings()
    except Exception as e:
        print(f"\n❌ IK bindings FAILED: {e}")
        import traceback
        traceback.print_exc()
        all_passed = False

    try:
        all_passed &= test_worldnode_bindings()
    except Exception as e:
        print(f"\n❌ WorldNode bindings FAILED: {e}")
        import traceback
        traceback.print_exc()
        all_passed = False

    print()
    print("=" * 70)
    if all_passed:
        print("✅ ALL VERIFICATION TESTS PASSED!")
        print("=" * 70)
        print("\nThe atlas_puppet example should now work correctly.")
        return 0
    else:
        print("❌ SOME TESTS FAILED!")
        print("=" * 70)
        print("\nFix the failing bindings before running the example.")
        return 1


if __name__ == "__main__":
    sys.exit(main())
