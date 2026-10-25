Section: Build

  * A GUI configure with `FETCHCONTENT_FULLY_DISCONNECTED=ON` no longer
    downloads Filament's license notices. If the installed Filament SDK has no
    `notices/` tree, as in conda-forge's package, seed the Filament 1.77.3
    source tree as `FETCHCONTENT_SOURCE_DIR_DART_FILAMENT_SDK`; otherwise
    configure stops and names the missing file.
    [#3965](https://github.com/dartsim/dart/pull/3965)
