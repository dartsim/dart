# Copyright (c) 2011-2021, The DART development contributors

import pytest
import dartpy8 as dart


def test_basics():
    sw = dart.common.Stopwatch()

    # Stopwatch is started by default
    assert sw.is_started()
    assert dart.common.Stopwatch(True).is_started()
    assert not dart.common.Stopwatch(False).is_started()

    # Stop the stopwatch
    sw.stop()
    assert not sw.is_started()

    # Elapsed time should be the same
    elapsed1 = sw.elapsed_s()
    assert elapsed1 == sw.elapsed_s()
    assert elapsed1 == sw.elapsed_s()

    # Elapsed time monotonically increase while the stopwatch is running
    sw.start()
    assert sw.elapsed_s() >= elapsed1
    assert sw.elapsed_s() >= elapsed1

    # Starting a stopwatch already started doesn't have any effect
    sw.start()
    assert sw.is_started()

    # Restting a started stopwatch resets the elapsed time but doesn't stop the
    # stopwatch
    sw.start()
    sw.reset()
    assert sw.is_started()
    assert sw.elapsed_s() >= 0.0
    assert sw.elapsed_s() >= 0.0

    # Restting a stopped stopwatch resets the elapsed time but doesn't start the
    # stopwatch
    sw.stop()
    sw.reset()
    assert not sw.is_started()
    assert sw.elapsed_s() == pytest.approx(0.0)

    sw.print()


if __name__ == "__main__":
    pytest.main()
