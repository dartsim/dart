import dartpy_nb as dart
import pytest


def test_basics():
    sw = dart.common.Stopwatch()

    assert sw.isStarted()
    assert dart.common.Stopwatch(True).isStarted()
    assert not dart.common.Stopwatch(False).isStarted()

    sw.stop()
    assert not sw.isStarted()

    elapsed = sw.elapsedS()
    assert elapsed == sw.elapsedS()

    sw.start()
    assert sw.elapsedS() >= elapsed

    sw.start()
    assert sw.isStarted()

    sw.start()
    sw.reset()
    assert sw.isStarted()
    assert sw.elapsedS() >= 0.0

    sw.stop()
    sw.reset()
    assert not sw.isStarted()
    assert sw.elapsedS() == pytest.approx(0.0)

    sw.print()


if __name__ == "__main__":
    pytest.main()
