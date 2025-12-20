import dartpy as dart
import pytest


def test_basics():
    dart.trace("trace log")
    dart.debug("debug log")
    dart.info("info log")
    dart.warn("warn log")
    dart.error("error log")
    dart.fatal("fatal log")


def test_arguments():
    val = 10
    dart.info("Log with param '{}' and '{}'".format(1, val))


if __name__ == "__main__":
    pytest.main()
