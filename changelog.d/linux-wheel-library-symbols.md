Section: Python

  * Linux pip wheels ship their bundled shared libraries without symbol
    tables, which makes the x86_64 wheel about 1 MB smaller to download and
    about 6 MB smaller once installed.
    [#3955](https://github.com/dartsim/dart/pull/3955)
