Section: Build

  * Fetch Tracy 0.14.1 from a SHA-256-verified tag archive instead of Git.
    A system Tracy for the Tracy profiling backend must be 0.14.1 or a later
    0.14 release built with `TRACY_ENABLE=ON`. The Pixi profile environment
    fetches Tracy and no longer provides a viewer or the `tracy` task: install
    a Tracy 0.14 viewer separately.
