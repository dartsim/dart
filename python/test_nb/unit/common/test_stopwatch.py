import dartpy as dart
import pytest


def test_basics():
    sw = dart.common.Stopwatch()

    # Stopwatch is started by default
    assert sw.isStarted()
    assert dart.common.Stopwatch(True).isStarted()
    assert not dart.common.Stopwatch(False).isStarted()

    # Stop the stopwatch
    sw.stop()
    assert not sw.isStarted()

    # Elapsed time should be the same
    elapsed1 = sw.elapsedS()
    assert elapsed1 == sw.elapsedS()
    assert elapsed1 == sw.elapsedS()

    # Elapsed time monotonically increase while the stopwatch is running
    sw.start()
    assert sw.elapsedS() >= elapsed1
    assert sw.elapsedS() >= elapsed1

    # Starting a stopwatch already started doesn't have any effect
    sw.start()
    assert sw.isStarted()

    # Restting a started stopwatch resets the elapsed time but doesn't stop the
    # stopwatch
    sw.start()
    sw.reset()
    assert sw.isStarted()
    assert sw.elapsedS() >= 0.0
    assert sw.elapsedS() >= 0.0

    # Restting a stopped stopwatch resets the elapsed time but doesn't start the
    # stopwatch
    sw.stop()
    sw.reset()
    assert not sw.isStarted()
    assert sw.elapsedS() == pytest.approx(0.0)

    sw.print()


if __name__ == "__main__":
    pytest.main()
