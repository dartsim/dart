Section: Python

  * Linux wheels bundle sdformat 16.1, as the macOS and Windows wheels do. Its
    shared schema descriptions cut the memory and time needed to load large SDF
    worlds: a world of 1,000 small models now loads in about 0.5 s and 0.2 GB
    instead of 6.5 s and 4.8 GB. The accepted SDF versions are unchanged (up
    to 1.12).
    [#3958](https://github.com/dartsim/dart/pull/3958)
