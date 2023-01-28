import dartpy as dart
import pytest


def test_basics():
    dart.common.trace("trace log")
    dart.common.debug("debug log")
    dart.common.info("info log")
    dart.common.warn("warn log")
    dart.common.error("error log")
    dart.common.fatal("fatal log")


def test_arguments():
    val = 10
    dart.common.info("Log with param '{}' and '{}'".format(1, val))


if __name__ == "__main__":
    pytest.main()
