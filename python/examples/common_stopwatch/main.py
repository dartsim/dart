#!/usr/bin/env python3
"""Showcase dartpy_nb.common.Stopwatch."""

from __future__ import annotations

import time

import dartpy_nb as dart


def main() -> None:
    sw = dart.common.Stopwatch()
    time.sleep(0.01)
    sw.stop()
    print("Elapsed (s):", sw.elapsedS())
    sw.reset()
    sw.start()
    dart.common.info("Stopwatch restarted")


if __name__ == "__main__":
    main()
