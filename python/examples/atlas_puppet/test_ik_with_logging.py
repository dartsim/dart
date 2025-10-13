#!/usr/bin/env python3
"""
Test IK solver with comprehensive C++ debug logging enabled.
This helps debug why IK returns success but doesn't move joints.

Run this with proper pixi environment that sets PYTHONPATH
"""

import dartpy as dart
import numpy as np
import os

# Enable INFO level logging to see C++ debug output
print("=" * 80)
print("Setting log level to INFO to see C++ debug output")
print("=" * 80)
dart.common.set_log_level("info")
print(f"Current log level: {dart.common.get_log_level()}")
print()

# Load the Atlas robot
print("Loading Atlas robot...")
os.environ['DART_DATA_PATH'] = os.path.join(os.getcwd(), 'data')
atlas_urdf = "dart://sample/sdf/atlas/atlas_v3_no_head.urdf"
atlas = dart.utils.DartLoader().parseSkeleton(atlas_urdf)
if atlas is None:
    print("ERROR: Failed to load atlas robot!")
    import sys
    sys.exit(1)
print(f"✓ Loaded {atlas.getName()} with {atlas.getNumDofs()} DOFs")
print()

# Get the skeleton's whole-body IK
print("Getting skeleton IK module...")
skel_ik = atlas.getIK(True)  # True = create if doesn't exist
print(f"✓ Skeleton IK type: {type(skel_ik)}")
print()

# Store initial joint positions
positions_before = atlas.getPositions().copy()
print("Initial joint positions (first 10 DOFs):")
print(positions_before[:10])
print()

# Try to solve IK without any targets (should do nothing but won't crash)
print("=" * 80)
print("CALLING solveAndApply(True) - Watch for C++ INFO logging output")
print("=" * 80)
solved = skel_ik.solveAndApply(True)
print("=" * 80)
print(f"solveAndApply(True) returned: {solved}")
print("=" * 80)
print()

# Check joint positions after
positions_after = atlas.getPositions().copy()
print("Joint positions after solving (first 10 DOFs):")
print(positions_after[:10])
print()

# Calculate changes
position_changes = np.abs(positions_after - positions_before)
max_change = np.max(position_changes)
print(f"Max position change: {max_change:.10f}")
print(f"Number of DOFs that changed (> 1e-10): {np.sum(position_changes > 1e-10)}")
print()

# Summary
print("=" * 80)
print("SUMMARY")
print("=" * 80)
print(f"IK solved: {solved}")
print(f"Max joint change: {max_change:.10f}")
if max_change < 1e-10:
    print("\n✓ EXPECTED: No joints moved (no IK targets set)")
else:
    print(f"\n⚠️  UNEXPECTED: Joints moved by {max_change:.6f} without targets!")
print("=" * 80)
print()
print("SUCCESS: C++ logging bindings are working!")
print("Next step: Check main.py to see why IK with actual targets doesn't work")
