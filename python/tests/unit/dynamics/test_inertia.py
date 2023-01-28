import math
import platform

import dartpy as dart
import numpy as np
import pytest


def test_inertia_init():
    """
    Test basic functionality for the `dartpy.dynamics.Inertia` class.
    """
    # test default values
    i1 = dart.dynamics.Inertia()
    assert i1 is not None

    # initialize with parameters
    i2 = dart.dynamics.Inertia(0.1, [0, 0, 0], 1.3 * np.eye(3))
    assert i1 is not None

    newMass = 1.5
    i2.setMass(newMass)
    assert i2.getMass() == newMass

    newCOM = np.array((0.1, 0, 0))
    i2.setLocalCOM(newCOM)
    assert np.allclose(i2.getLocalCOM(), newCOM)

    newMoment = 0.4 * newMass * 0.1**2 * np.eye(3)
    i2.setMoment(newMoment)
    assert np.allclose(i2.getMoment(), newMoment)

    i2.setSpatialTensor(0.3 * i2.getSpatialTensor())

    assert i2.verify()

    for i in range(10):  # based on the C++ tests
        mass = np.random.uniform(0.1, 10.0)
        com = np.random.uniform(-5, 5, 3)
        I = np.random.rand(3, 3) - 0.5 + np.diag(np.random.uniform(0.6, 1, 3), 0)
        I = (I + I.T) / 2

        inertia = dart.dynamics.Inertia(mass, com, I)
        assert inertia.verify()


def test_inertia_static_methods():
    """
    Test the class methods `verifyMoment`and `verifySpatialTensor`.
    """
    assert dart.dynamics.Inertia.verifyMoment(np.eye(3), printWarnings=False)
    for i in range(10):
        I = np.random.rand(3, 3) - 0.5 + np.diag(np.random.uniform(1, 10, 3), 0)
        I = (I + I.T) / 2
        assert dart.dynamics.Inertia.verifyMoment(I)

    assert dart.dynamics.Inertia.verifySpatialTensor(np.eye(6), printWarnings=False)


def test_failing_moment_and_spatial():
    """
    Test some failure cases of the verify methods.
    """

    for i in range(10):
        I = np.random.rand(3, 3) - 0.5 - np.diag(np.random.uniform(1, 10, 3), 0)
        assert not dart.dynamics.Inertia.verifyMoment(I, printWarnings=False)

    # fails e.g. due to off diagonal values in translational part.
    assert not dart.dynamics.Inertia.verifySpatialTensor(
        np.random.rand(6, 6), printWarnings=False
    )


if __name__ == "__main__":
    pytest.main()
