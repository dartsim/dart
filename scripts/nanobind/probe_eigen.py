"""Build and check the production Eigen caster against the active NumPy version.

For the NumPy 1.x fallback, run from the repository root:
  pixi exec --spec python=3.12 --spec numpy=1.26 --spec nanobind=3.1 \
    --spec eigen --spec cmake --spec ninja -- python scripts/nanobind/probe_eigen.py \
    --build-dir build/nanobind-eigen-numpy1
Repeat with --spec numpy=2 and a separate build directory for the fast path.
"""

import argparse
import gc
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def check_exports(module_dir):
    sys.path.insert(0, str(module_dir))
    import numpy as np
    import numpy_export_probe as probe

    major = int(np.__version__.split(".", 1)[0])
    assert probe.fast_export() is (major == 2)
    for size in (0, 1, 7, 1000):
        actual = probe.vector(size)
        gc.collect()
        np.testing.assert_array_equal(actual, np.arange(1, size + 1, dtype=float))
        assert actual.dtype == np.float64 and actual.shape == (size,)
        assert actual.flags.writeable
        if size:
            actual[0] = -1
    for shape in ((0, 0), (0, 3), (2, 0), (2, 3), (10, 7)):
        actual = probe.matrix(*shape)
        gc.collect()
        expected = np.arange(1, shape[0] * shape[1] + 1, dtype=float).reshape(shape)
        np.testing.assert_array_equal(actual, expected)
        assert actual.dtype == np.float64 and actual.shape == shape
        assert actual.flags.writeable
    for shape in ((7,), (7, 1)):
        storage = np.arange(7.0, dtype=np.float64)
        output = storage.reshape(shape)
        probe.mutate(output)
        np.testing.assert_array_equal(storage, np.arange(10.0, 17.0))
        np.testing.assert_array_equal(output, storage.reshape(shape))
    readonly = np.zeros((7, 1))
    readonly.flags.writeable = False
    for invalid in (
        np.zeros((7, 2)),
        np.zeros(7, dtype=np.float32),
        np.zeros(14)[::2],
        readonly,
    ):
        before = invalid.copy()
        try:
            probe.mutate(invalid)
        except TypeError:
            pass
        else:
            raise AssertionError("invalid mutable Ref accepted")
        np.testing.assert_array_equal(invalid, before)
    for call in (lambda: probe.vector(-1), lambda: probe.matrix(-1, 2)):
        try:
            call()
        except ValueError:
            pass
        else:
            raise AssertionError("negative dimensions accepted")
    print(
        f"NumPy {np.__version__}: runtime selection, Eigen export lifetime, and mutable Ref passed"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path)
    parser.add_argument("--check-dir", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.check_dir is not None:
        check_exports(args.check_dir)
        return
    if args.build_dir is None:
        parser.error("--build-dir is required")
    subprocess.run(
        [
            "cmake",
            "-G",
            "Ninja",
            "-S",
            str(ROOT / "scripts/nanobind/eigen_probe"),
            "-B",
            str(args.build_dir),
            f"-DPython_EXECUTABLE={sys.executable}",
            f"-DCMAKE_PREFIX_PATH={sys.prefix}",
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        check=True,
    )
    subprocess.run(
        [
            "cmake",
            "--build",
            str(args.build_dir),
            "--target",
            "check",
            "--parallel",
            "2",
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
