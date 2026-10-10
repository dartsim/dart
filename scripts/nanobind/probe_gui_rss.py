"""Linux create/destroy RSS oracle; run in a fresh process for each binder."""

import gc
import json
import os
from pathlib import Path

import dartpy as dart


def rss():
    return int(Path("/proc/self/statm").read_text().split()[1]) * os.sysconf(
        "SC_PAGE_SIZE"
    )


def cycle():
    viewer = dart.gui.osg.Viewer()
    node = dart.gui.osg.WorldNode(dart.simulation.World())
    handler = dart.gui.osg.GUIEventHandler()
    viewer.addWorldNode(node)
    viewer.addEventHandler(handler)


def main():
    for _ in range(100):
        cycle()
    gc.collect()
    before = rss()
    for _ in range(1000):
        cycle()
    gc.collect()
    growth = rss() - before
    assert growth < 8 * 1024 * 1024, growth
    print(json.dumps({"iterations": 1000, "rss_growth_bytes": growth}))


if __name__ == "__main__":
    main()
