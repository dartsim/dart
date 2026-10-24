Section: Collision

  * Refresh soft-body collision geometry after face edits, preventing stale
    contacts and invalid mesh access when the point-mass count stays unchanged.
    Soft meshes with no faces produce no contacts, including with FCL.
    [#3926](https://github.com/dartsim/dart/pull/3926)
