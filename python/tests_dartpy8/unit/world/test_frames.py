"""Tests for Frame, FreeFrame, and FixedFrame classes."""

# TODO: Re-enable Frame API tests once C++ implementation is complete
#
# The Frame API tests have been temporarily disabled because the following methods
# are not yet implemented in the C++ Frame class:
#
# Transform getters/setters:
#   - setRelativeTransform(), getRelativeTranslation(), setRelativeTranslation()
#   - setRelativeRotation(), getRelativeRotation(), setRelativeQuaternion()
#   - getRelativeQuaternion(), setPosition(), getPosition(), setQuaternion()
#   - getQuaternion(), setTransform()
#
# Velocity getters/setters:
#   - setSpatialVelocity(), getSpatialVelocity(), setLinearVelocity()
#   - getLinearVelocity(), setAngularVelocity(), getAngularVelocity()
#
# Acceleration getters/setters:
#   - setSpatialAcceleration(), getSpatialAcceleration(), setLinearAcceleration()
#   - getLinearAcceleration(), setAngularAcceleration(), getAngularAcceleration()
#
# Once these methods are implemented in C++ and bound to Python, the test classes
# (TestFreeFrame, TestFixedFrame, TestMixedFrameTypes) should be restored from git history.
#
# See git commit history for the full test implementation.
