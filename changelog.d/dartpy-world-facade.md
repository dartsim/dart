Section: Python

  * Add keyword options and properties to `dartpy.World`, with collision and
    solver handles and compile/edit contexts. Invalid `time_step` values raise
    `ValueError`. If an edit body raises, its changes remain and the World
    stays uncompiled.
    [#3968](https://github.com/dartsim/dart/pull/3968)
