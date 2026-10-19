"""dartpy.math.Transform and Rotation: immutable scalar or batched values."""

import pickle

import dartpy as dart
import numpy as np
import pytest

RNG = np.random.default_rng(7)


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def random_rotation(n=None):
    q = RNG.normal(size=(4,) if n is None else (n, 4))
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    return dart.Rotation(wxyz=q).matrix


def test_canonical_module_and_reexport():
    for name in ("Transform", "Rotation"):
        cls = getattr(dart.math, name)
        assert getattr(dart, name) is cls
        assert cls.__module__ == "dartpy.math"
        assert cls.__array_ufunc__ is None


def test_identity_and_default_constructor():
    assert dart.Transform() == dart.Transform.identity()
    assert dart.Rotation() == dart.Rotation.identity()
    np.testing.assert_array_equal(dart.Transform().matrix, np.eye(4))
    np.testing.assert_array_equal(dart.Rotation().matrix, np.eye(3))
    with pytest.raises(TypeError):
        dart.Transform(np.eye(3))  # positional arguments are rejected


@pytest.mark.parametrize("value_type", [dart.Transform, dart.Rotation])
def test_every_rotation_form_agrees(value_type):
    roll, pitch, yaw = 0.3, -0.7, 1.9
    R = rz(yaw) @ ry(pitch) @ rx(roll)
    q = dart.Rotation(rotation=R).wxyz
    forms = [
        value_type(rotation=R),
        value_type(rotation=dart.Rotation(rotation=R)),
        value_type(wxyz=q),
        value_type(xyzw=q[[1, 2, 3, 0]]),
        value_type(rpy=(roll, pitch, yaw)),
    ]
    if value_type is dart.Transform:
        M = np.eye(4)
        M[:3, :3] = R
        forms.append(value_type(matrix=M))
    for value in forms:
        np.testing.assert_allclose(value @ np.eye(3), np.eye(3) @ R.T, atol=1e-14)
        np.testing.assert_allclose(value.wxyz, q, atol=1e-14)


def test_rpy_is_extrinsic_xyz():
    roll, pitch, yaw = 0.4, 0.9, -2.1
    expected = rz(yaw) @ ry(pitch) @ rx(roll)
    rotation = dart.Rotation(rpy=(roll, pitch, yaw))
    np.testing.assert_allclose(rotation.matrix, expected, atol=1e-15)
    np.testing.assert_allclose(rotation.as_rpy(), (roll, pitch, yaw), atol=1e-14)
    # Differs from the intrinsic helper's convention, which is why it is not reused.
    assert not np.allclose(rotation.matrix, rx(roll) @ ry(pitch) @ rz(yaw))
    locked = dart.Rotation(rpy=(0.3, np.pi / 2, 0.5))
    recovered = dart.Rotation(rpy=locked.as_rpy())
    np.testing.assert_allclose(recovered.matrix, locked.matrix, atol=1e-12)
    assert recovered.as_rpy()[0] == 0


def test_constructor_conflicts_and_shapes():
    with pytest.raises(ValueError, match="at most one"):
        dart.Transform(rotation=np.eye(3), wxyz=(1, 0, 0, 0))
    with pytest.raises(ValueError, match="at most one"):
        dart.Rotation(rpy=(0, 0, 0), xyzw=(0, 0, 0, 1))
    with pytest.raises(ValueError, match="either matrix= or translation="):
        dart.Transform(matrix=np.eye(4), translation=(1, 2, 3))
    with pytest.raises(TypeError):
        dart.Rotation(matrix=np.eye(4))
    with pytest.raises(TypeError):
        dart.Rotation(translation=(1, 2, 3))
    for options in (
        {"rotation": np.eye(4)},
        {"wxyz": (1, 0, 0)},
        {"rpy": (0, 0)},
        {"translation": (1, 2)},
        {"matrix": np.eye(3)},
    ):
        with pytest.raises(ValueError, match="shape"):
            dart.Transform(**options)
    with pytest.raises(ValueError):
        dart.Transform(rotation="not a matrix")


def test_validation_tolerance():
    R = random_rotation()
    noisy = R + RNG.normal(size=(3, 3)) * 1e-12
    np.testing.assert_array_equal(dart.Rotation(rotation=noisy).matrix, noisy)
    bad = R + RNG.normal(size=(3, 3)) * 1e-4
    with pytest.raises(ValueError, match="normalize=True"):
        dart.Rotation(rotation=bad)
    projected = dart.Rotation(rotation=bad, normalize=True).matrix
    np.testing.assert_allclose(projected.T @ projected, np.eye(3), atol=1e-14)
    assert np.linalg.det(projected) == pytest.approx(1, abs=1e-14)
    assert np.abs(projected - R).max() < 1e-3
    six_digits = np.array([[float(f"{v:.5e}") for v in row] for row in R])
    assert dart.Rotation(rotation=six_digits) is not None
    for value in (np.inf, np.nan):
        with pytest.raises(ValueError, match="finite"):
            dart.Transform(translation=(value, 0, 0))
        with pytest.raises(ValueError, match="finite"):
            dart.Rotation(rpy=(value, 0, 0))
        with pytest.raises(ValueError, match="finite"):
            dart.Rotation(wxyz=(value, 0, 0, 0))
    with pytest.raises(ValueError, match="homogeneous row"):
        dart.Transform(matrix=np.diag([1.0, 1, 1, 2]))


def test_normalize_criterion():
    projected = dart.Rotation(rotation=1.001 * np.eye(3), normalize=True)
    np.testing.assert_allclose(projected.matrix, np.eye(3), atol=1e-12)
    for options in (
        {"rotation": np.diag([-1, 1, 1])},
        {"rotation": np.array([[1, 0.25, 0], [0, 1, 0], [0, 0, 1]])},
        {"rotation": np.diag([2, 1, 1])},
        {"rotation": np.zeros((3, 3))},
    ):
        with pytest.raises(ValueError):
            dart.Rotation(normalize=True, **options)
        with pytest.raises(ValueError):
            dart.Rotation(**options)
    # Reflection errors do not suggest normalize=True.
    with pytest.raises(ValueError) as info:
        dart.Rotation(rotation=np.diag([-1, 1, 1]))
    assert "normalize=True" not in str(info.value)


def test_quaternion_norm_and_scaled_normalization():
    with pytest.raises(ValueError, match="normalize=True"):
        dart.Rotation(wxyz=(1, 0, 0, 1))
    with pytest.raises(ValueError, match="zero quaternion"):
        dart.Rotation(wxyz=(0, 0, 0, 0), normalize=True)
    huge = np.array([1, 1, 0, 0]) * 1e200
    tiny = np.array([1, 1, 0, 0]) * 1e-200
    expected = dart.Rotation(wxyz=np.array([1, 1, 0, 0]) / np.sqrt(2)).wxyz
    for q in (huge, tiny):
        np.testing.assert_allclose(dart.Rotation(wxyz=q, normalize=True).wxyz, expected)
    # Within tolerance of unit, the quaternion is accepted and the matrix built from the unit quaternion.
    close = np.array([1, 0, 0, 0]) * (1 + 1e-7)
    np.testing.assert_array_equal(dart.Rotation(wxyz=close).matrix, np.eye(3))


def test_quaternion_sign_policy_and_ties():
    half = np.pi / 4
    q = np.array([np.cos(half), np.sin(half), 0, 0])
    np.testing.assert_allclose(dart.Rotation(wxyz=-q).wxyz, q, atol=1e-15)
    # w == 0: the first nonzero of (x, y, z) is positive.
    np.testing.assert_array_equal(dart.Rotation(wxyz=(0, -1, 0, 0)).wxyz, (0, 1, 0, 0))
    np.testing.assert_array_equal(dart.Rotation(wxyz=(0, 0, 0, -1)).wxyz, (0, 0, 0, 1))
    tie = np.array([0, 0, 1, -1]) / np.sqrt(2)
    np.testing.assert_allclose(dart.Rotation(wxyz=-tie).wxyz, tie, atol=1e-15)
    np.testing.assert_allclose(dart.Rotation(wxyz=tie).wxyz, tie, atol=1e-15)
    # Largest-diagonal branch: a near-180-degree rotation stays accurate.
    R = rx(np.pi - 1e-9)
    back = dart.Rotation(wxyz=dart.Rotation(rotation=R).wxyz).matrix
    np.testing.assert_allclose(back, R, atol=1e-15)


def test_wxyz_xyzw_round_trips():
    for _ in range(20):
        q = RNG.normal(size=4)
        q /= np.linalg.norm(q)
        if q[0] < 0:
            q = -q
        rotation = dart.Rotation(wxyz=q)
        np.testing.assert_allclose(rotation.wxyz, q, atol=1e-15)
        np.testing.assert_allclose(rotation.xyzw, q[[1, 2, 3, 0]], atol=1e-15)
        np.testing.assert_allclose(dart.Rotation(xyzw=rotation.xyzw).wxyz, q, atol=1e-15)
        transform = dart.Transform(xyzw=rotation.xyzw, translation=(1, 2, 3))
        np.testing.assert_allclose(transform.wxyz, q, atol=1e-15)
        # Quaternion round trips are not bit-exact; copying a Rotation is.
        np.testing.assert_allclose(transform.rotation.matrix, rotation.matrix, atol=1e-15)
        assert dart.Transform(rotation=rotation, translation=(1, 2, 3)).rotation == rotation


def test_composition_and_inverse():
    a = dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1, 2, 3))
    b = dart.Transform(wxyz=(0, 0, 1, 0), translation=(-1, 0.5, 0))
    np.testing.assert_allclose((a @ b).matrix, a.matrix @ b.matrix, atol=1e-15)
    np.testing.assert_allclose((a @ a.inverse()).matrix, np.eye(4), atol=1e-14)
    np.testing.assert_allclose(a.inverse().matrix, np.linalg.inv(a.matrix), atol=1e-14)
    r = dart.Rotation(rpy=(0.5, 0, 0))
    assert isinstance(a @ r, dart.Transform)
    assert isinstance(r @ a, dart.Transform)
    assert isinstance(r @ r, dart.Rotation)
    np.testing.assert_allclose((a @ r).matrix, a.matrix @ dart.Transform(rotation=r).matrix, atol=1e-15)
    np.testing.assert_allclose((r @ a).matrix, dart.Transform(rotation=r).matrix @ a.matrix, atol=1e-15)
    np.testing.assert_allclose((r @ r.inverse()).matrix, np.eye(3), atol=1e-15)
    with pytest.raises(TypeError):
        a * b
    with pytest.raises(TypeError):
        a * 2.0


def test_points_shapes():
    T = dart.Transform(rpy=(0.3, -0.2, 0.9), translation=(0.5, -1, 2))
    R = T.matrix[:3, :3]
    t = T.translation
    point = np.array([1.0, 2.0, 3.0])
    np.testing.assert_allclose(T @ point, R @ point + t, atol=1e-14)
    np.testing.assert_allclose(T @ (1, 2, 3), R @ point + t, atol=1e-14)
    assert (T @ point).shape == (3,)
    points = RNG.normal(size=(5, 3))
    np.testing.assert_allclose(T @ points, points @ R.T + t, atol=1e-14)
    grid = RNG.normal(size=(2, 4, 3))
    np.testing.assert_allclose(T @ grid, grid @ R.T + t, atol=1e-14)
    assert (T @ grid).shape == (2, 4, 3)
    np.testing.assert_allclose(T.rotation @ grid, grid @ R.T, atol=1e-14)
    strided = np.asfortranarray(points)
    np.testing.assert_allclose(T @ strided, T @ points)
    for bad in (np.zeros(4), np.zeros((3, 2)), 1.0):
        with pytest.raises(ValueError, match=r"\(\.\.\., 3\)"):
            T @ bad


def test_batches():
    n = 4
    rotations = random_rotation(n)
    translations = RNG.normal(size=(n, 3))
    batch = dart.Transform(rotation=rotations, translation=translations)
    assert len(batch) == n
    assert batch.matrix.shape == (n, 4, 4)
    assert batch.translation.shape == (n, 3)
    assert batch.wxyz.shape == (n, 4)
    assert batch.rotation.matrix.shape == (n, 3, 3)
    assert batch.rotation.as_rpy().shape == (n, 3)
    assert len(batch.rotation) == n
    for i in range(n):
        item = batch[i]
        np.testing.assert_array_equal(item.matrix[:3, :3], rotations[i])
        np.testing.assert_array_equal(item.translation, translations[i])
        assert item == batch[i - n]
    with pytest.raises(IndexError):
        batch[n]
    scalar = dart.Transform(translation=(1, 0, 0))
    with pytest.raises(TypeError):
        len(scalar)
    with pytest.raises(TypeError):
        scalar[0]
    # One-item arrays with a leading axis are batches of one.
    one = dart.Transform(wxyz=[[1, 0, 0, 0]])
    assert len(one) == 1
    assert one != dart.Transform()
    assert one[0] == dart.Transform()
    # Broadcasting: scalar (x) batch and equal-length batches only.
    left = scalar @ batch
    right = batch @ scalar
    pair = batch @ batch
    for i in range(n):
        np.testing.assert_allclose(left[i].matrix, scalar.matrix @ batch[i].matrix, atol=1e-15)
        np.testing.assert_allclose(right[i].matrix, batch[i].matrix @ scalar.matrix, atol=1e-15)
        np.testing.assert_allclose(pair[i].matrix, batch[i].matrix @ batch[i].matrix, atol=1e-15)
    with pytest.raises(ValueError, match="batch sizes"):
        batch @ dart.Transform(translation=np.zeros((n + 1, 3)))
    with pytest.raises(ValueError, match="does not match"):
        dart.Transform(rotation=rotations, translation=np.zeros((n + 1, 3)))
    mixed = dart.Transform(rotation=rotations[0], translation=translations)
    assert len(mixed) == n
    np.testing.assert_array_equal(mixed[2].matrix[:3, :3], rotations[0])
    point = np.array([1.0, 2.0, 3.0])
    expected = np.stack([batch[i] @ point for i in range(n)])
    np.testing.assert_allclose(batch @ point, expected, atol=1e-15)
    points = RNG.normal(size=(n, 3))
    expected = np.stack([batch[i] @ points[i] for i in range(n)])
    np.testing.assert_allclose(batch @ points, expected, atol=1e-15)
    with pytest.raises(ValueError, match="accepts points of shape"):
        batch @ np.zeros((n + 1, 3))
    with pytest.raises(ValueError, match="accepts points of shape"):
        batch @ np.zeros((2, n, 3))
    with pytest.raises(TypeError, match="unhashable"):
        hash(batch)
    assert repr(batch) == f"Transform(<batch of {n}>)"


def test_batched_rotation_forms():
    n = 3
    q = RNG.normal(size=(n, 4))
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    rpy = RNG.uniform(-1, 1, size=(n, 3))
    from_q = dart.Rotation(wxyz=q)
    from_xyzw = dart.Rotation(xyzw=q[:, [1, 2, 3, 0]])
    from_rpy = dart.Rotation(rpy=rpy)
    matrices = np.stack([(rz(y) @ ry(p) @ rx(r)) for r, p, y in rpy])
    np.testing.assert_allclose(from_rpy.matrix, matrices, atol=1e-15)
    np.testing.assert_allclose(from_q.matrix, from_xyzw.matrix, atol=1e-15)
    stacked = dart.Transform(matrix=dart.Transform(rotation=from_rpy).matrix)
    assert stacked == dart.Transform(rotation=from_rpy)
    assert dart.Rotation(rotation=from_rpy) == from_rpy


def test_equality_and_hash_are_bit_exact():
    a = dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1, 2, 3))
    b = dart.Transform(matrix=a.matrix)
    assert a == b
    assert hash(a) == hash(b)
    assert not (a != b)
    assert a != dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1, 2, np.nextafter(3, 4)))
    # -0.0 and 0.0 differ in bits, so they are different values.
    assert dart.Transform(translation=(0, 0, 0.0)) != dart.Transform(translation=(0, 0, -0.0))
    assert a.__eq__(a.rotation) is NotImplemented
    assert a != a.rotation
    assert a != "pose"
    assert {a: 1}[b] == 1
    assert hash(dart.Rotation()) == hash(dart.Rotation.identity())


def test_pickle_keeps_bits():
    values = [
        dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1 / 3, -2 / 7, 1e-300)),
        dart.Rotation(wxyz=np.array([1, 2, 3, 4]) / np.sqrt(30)),
        dart.Transform(rotation=random_rotation(3), translation=RNG.normal(size=(3, 3))),
        dart.Rotation(rpy=RNG.normal(size=(2, 3))),
    ]
    for value in values:
        for protocol in range(2, pickle.HIGHEST_PROTOCOL + 1):
            restored = pickle.loads(pickle.dumps(value, protocol=protocol))
            assert type(restored) is type(value)
            assert restored == value
            np.testing.assert_array_equal(restored.matrix, value.matrix)
    assert len(pickle.loads(pickle.dumps(values[2]))) == 3
    with pytest.raises(ValueError):
        dart.Transform.__new__(dart.Transform).__setstate__((False, b"short"))


def test_immutability():
    T = dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1, 2, 3))
    for array in (T.matrix, T.translation, T.wxyz, T.xyzw, T.rotation.matrix, T.rotation.as_rpy(), T @ np.eye(3)):
        assert not array.flags.writeable
        with pytest.raises(ValueError):
            array[...] = 0
    before = T.matrix.copy()
    copy = T.translation.copy()
    copy[0] = 99
    np.testing.assert_array_equal(T.matrix, before)
    for name in ("translation", "rotation", "matrix", "wxyz", "xyzw", "extra"):
        with pytest.raises(AttributeError):
            setattr(T, name, 0)
    with pytest.raises(TypeError):

        class Sub(dart.Transform):
            pass


def test_numpy_defers_to_matmul():
    T = dart.Transform(translation=(1, 2, 3))
    points = np.zeros((2, 3))
    with pytest.raises(TypeError, match="value @ points"):
        points @ T
    with pytest.raises(TypeError):
        np.matmul(points, T)
    with pytest.raises(TypeError):
        points * T
    np.testing.assert_array_equal(T @ points, np.tile((1, 2, 3), (2, 1)))


def test_from_axis_angle():
    rotation = dart.Rotation.from_axis_angle((0, 0, 2), np.pi / 2)
    np.testing.assert_allclose(rotation.matrix, rz(np.pi / 2), atol=1e-15)
    np.testing.assert_allclose(rotation @ (1, 0, 0), (0, 1, 0), atol=1e-15)
    with pytest.raises(ValueError, match="nonzero"):
        dart.Rotation.from_axis_angle((0, 0, 0), 1.0)
    with pytest.raises(ValueError, match="finite"):
        dart.Rotation.from_axis_angle((0, 0, 1), np.nan)
    with pytest.raises(ValueError, match="shape"):
        dart.Rotation.from_axis_angle((0, 0), 1.0)


def test_isometry_interop():
    T = dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1, 2, 3))
    iso = T.to_isometry()
    assert isinstance(iso, dart.math.Isometry3)
    np.testing.assert_array_equal(iso.matrix(), T.matrix)
    iso.set_translation((4, 5, 6))
    np.testing.assert_array_equal(T.translation, (1, 2, 3))
    back = dart.Transform.from_isometry(iso)
    np.testing.assert_array_equal(back.translation, (4, 5, 6))
    np.testing.assert_array_equal(back.rotation.matrix, T.rotation.matrix)
    with pytest.raises(TypeError):
        dart.Transform.from_isometry(np.eye(4))
    with pytest.raises(TypeError):
        dart.Transform(translation=np.zeros((2, 3))).to_isometry()
    skewed = dart.math.Isometry3(np.diag([2.0, 1, 1, 1]))
    with pytest.raises(ValueError):
        dart.Transform.from_isometry(skewed)


def test_repr_round_trips_floats():
    T = dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1 / 3, 2, 3))
    text = repr(T)
    assert text.startswith("Transform(wxyz=[") and "translation=[" in text
    rebuilt = eval(text, {"Transform": dart.Transform})
    np.testing.assert_allclose(rebuilt.matrix, T.matrix, atol=1e-15)
    np.testing.assert_array_equal(rebuilt.translation, T.translation)
    rotation = eval(repr(T.rotation), {"Rotation": dart.Rotation})
    np.testing.assert_allclose(rotation.matrix, T.rotation.matrix, atol=1e-15)


def test_composition_drift_is_not_normalized():
    # Composition is plain matrix multiplication; validation happens only at
    # construction, so drift accumulates slowly and never changes silently.
    step = dart.Rotation(rpy=(0.01, 0.02, 0.03))
    acc = dart.Rotation()
    for _ in range(2000):
        acc = acc @ step
    error = np.linalg.norm(acc.matrix.T @ acc.matrix - np.eye(3))
    assert 0 < error < 1e-12
    assert dart.Rotation(rotation=acc.matrix) == acc


def test_points_of_any_rank_and_empty():
    T = dart.Transform(rpy=(0.3, -0.2, 0.9), translation=(0.5, -1, 2))
    R = T.matrix[:3, :3]
    t = T.translation
    for shape in ((2, 3, 2, 3), (2, 1, 3, 2, 3)):
        points = RNG.normal(size=shape)
        out = T @ points
        assert out.shape == shape
        assert not out.flags.writeable
        np.testing.assert_allclose(out, points @ R.T + t, atol=1e-14)
        np.testing.assert_allclose(T.rotation @ points, points @ R.T, atol=1e-14)
    for shape in ((0, 3), (0, 4, 3), (2, 0, 3)):
        assert (T @ np.empty(shape)).shape == shape
    for scalar in (np.float64(1.0), np.array(2.0)):
        with pytest.raises(ValueError, match=r"\(\.\.\., 3\)"):
            T @ scalar


def test_quaternion_normalization_extremes():
    expected = dart.Rotation(wxyz=(0.5, 0.5, 0.5, 0.5))
    huge = dart.Rotation(wxyz=(1e308,) * 4, normalize=True)
    assert huge != dart.Rotation()
    np.testing.assert_allclose(huge.wxyz, expected.wxyz, atol=1e-15)
    tiny = dart.Rotation(wxyz=(5e-324, 5e-324, 0, 0), normalize=True)
    M = tiny.matrix
    np.testing.assert_allclose(M.T @ M, np.eye(3), atol=1e-15)
    np.testing.assert_allclose(tiny.wxyz, np.array([1, 1, 0, 0]) / np.sqrt(2), atol=1e-15)
    # The small coefficient underflows relative to the large one and vanishes.
    mixed = dart.Rotation(wxyz=(1e200, 1e-200, 0, 0), normalize=True)
    np.testing.assert_array_equal(mixed.wxyz, (1, 0, 0, 0))
    for q in ((1e308,) * 4, (5e-324, 5e-324, 0, 0), (1e200, 1e-200, 0, 0)):
        with pytest.raises(ValueError, match="normalize=True"):
            dart.Rotation(wxyz=q)


def test_from_axis_angle_extremes():
    expected = dart.Rotation.from_axis_angle((1, 1, 1), 1.0)
    for scale in (1.7e308, 1e200, 1e-200, 5e-324):
        axis = np.array([1, 1, 1]) * scale
        rotation = dart.Rotation.from_axis_angle(axis, 1.0)
        np.testing.assert_allclose(rotation.matrix, expected.matrix, atol=1e-15)
    mixed = dart.Rotation.from_axis_angle((1e200, 1e-200, 0), 0.7)
    np.testing.assert_allclose(mixed.matrix, rx(0.7), atol=1e-15)
    skewed = dart.Rotation.from_axis_angle((5e-324, 0, 1e-320), 0.5).matrix
    np.testing.assert_allclose(skewed.T @ skewed, np.eye(3), atol=1e-15)
    assert np.linalg.det(skewed) == pytest.approx(1, abs=1e-14)
    for axis in ((0, 0, 0), (0.0, -0.0, 0.0)):
        with pytest.raises(ValueError, match="nonzero"):
            dart.Rotation.from_axis_angle(axis, 1.0)


def test_normalize_rejects_small_shear():
    for shear in (0.005, 0.002):
        S = np.array([[1, shear, 0], [0, 1, 0], [0, 0, 1]])
        with pytest.raises(ValueError, match="spread"):
            dart.Rotation(rotation=S, normalize=True)
        with pytest.raises(ValueError) as info:
            dart.Rotation(rotation=S)
        assert "normalize=True" not in str(info.value)
    uniform = dart.Rotation(rotation=1.001 * np.eye(3), normalize=True)
    np.testing.assert_allclose(uniform.matrix, np.eye(3), atol=1e-12)
    R = random_rotation()
    rounded = R.astype(np.float32).astype(np.float64)
    with pytest.raises(ValueError, match="normalize=True"):
        dart.Rotation(rotation=rounded * (1 + 1e-4))
    projected = dart.Rotation(rotation=rounded * (1 + 1e-4), normalize=True).matrix
    np.testing.assert_allclose(projected, R, atol=1e-6)


def test_empty_batches_are_rejected():
    for options in (
        {"translation": np.empty((0, 3))},
        {"rotation": np.zeros((0, 3, 3))},
        {"wxyz": np.zeros((0, 4))},
        {"xyzw": np.zeros((0, 4))},
        {"rpy": np.zeros((0, 3))},
        {"matrix": np.zeros((0, 4, 4))},
    ):
        with pytest.raises(ValueError, match="empty"):
            dart.Transform(**options)
    with pytest.raises(ValueError, match="empty"):
        dart.Rotation(rotation=np.zeros((0, 3, 3)))
    with pytest.raises(ValueError, match="empty"):
        dart.Rotation(rpy=np.zeros((0, 3)))
    # A flat empty list is shape (0,), a shape error rather than an empty batch.
    with pytest.raises(ValueError, match="shape"):
        dart.Rotation(rpy=[])


def test_setstate_checks_structure_and_keeps_exact_bits():
    def restore(cls, batched, raw):
        value = cls.__new__(cls)
        value.__setstate__((batched, raw))
        return value

    good = dart.Transform(rpy=(0.1, 0.2, 0.3), translation=(1, 2, 3))
    batched, raw = good.__getstate__()
    assert restore(dart.Transform, batched, raw) == good
    nan = np.eye(4)
    nan[0, 3] = np.nan
    reflection = np.diag([-1.0, 1.0, 1.0, 1.0])
    row = np.eye(4)
    row[3, 3] = 2
    moved = np.eye(4)
    moved[0, 3] = 1
    # Storage is column-major, so transpose before serializing.
    cases = [(nan, "finite"), (reflection, "determinant"), (row, "homogeneous")]
    for bad, pattern in cases:
        with pytest.raises(ValueError, match=pattern):
            restore(dart.Transform, False, bad.T.tobytes())
        with pytest.raises(ValueError, match=pattern):
            restore(dart.Rotation, True, np.eye(4).tobytes() + bad.T.tobytes())
    with pytest.raises(ValueError, match="translation"):
        restore(dart.Rotation, False, moved.T.tobytes())
    assert restore(dart.Transform, False, moved.T.tobytes()).translation[0] == 1
    for bad_raw in (b"", b"short", raw + raw, raw[:-1]):
        with pytest.raises(ValueError):
            restore(dart.Transform, False, bad_raw)
    with pytest.raises(ValueError):
        restore(dart.Transform, True, b"")
    with pytest.raises(ValueError):
        restore(dart.Transform, True, raw[:-8])


def test_quaternion_sign_rule_is_exact():
    # The sign rule looks at the exact bits of w: a rounded w of -1e-17 still
    # negates the whole quaternion rather than being treated as zero.
    q = dart.Rotation(wxyz=(-1e-17, 1, 0, 0), normalize=True).wxyz
    assert q[0] > 0
    assert q[1] == pytest.approx(-1, abs=1e-15)
    assert hash(dart.Rotation()) != -1


@pytest.mark.parametrize("cls", [dart.Transform, dart.Rotation])
def test_pickle_round_trips_drifted_composition(cls):
    import pickle

    a = cls(rotation=(1 + 2e-6) * np.eye(3))
    drifted = a @ a
    restored = pickle.loads(pickle.dumps(drifted))
    assert restored == drifted


def test_normalize_spread_threshold_boundary():
    inside = np.diag([1.0005, 1.0, 1.0])
    outside = np.diag([1.0015, 1.0, 1.0])
    dart.Rotation(rotation=inside, normalize=True)
    with pytest.raises(ValueError):
        dart.Rotation(rotation=outside, normalize=True)
