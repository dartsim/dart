import numpy as np

import dartpy_nb


def test_euler_round_trip():
    angles = np.array([0.1, -0.2, 0.3])
    rot = dartpy_nb.math.eulerXYZToMatrix(angles)
    recovered = dartpy_nb.math.matrixToEulerXYZ(rot)
    assert np.allclose(angles, recovered)


def test_exp_map_identity():
    zero_twist = np.zeros(6)
    transform = dartpy_nb.math.expMap(zero_twist)
    assert np.allclose(transform.matrix(), np.eye(4))


def test_verify_rotation():
    rot = dartpy_nb.math.eulerZYXToMatrix(np.zeros(3))
    assert dartpy_nb.math.verifyRotation(rot)
