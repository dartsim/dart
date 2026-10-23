Section: API

  * The GUI log panel captures native diagnostics alongside dartpy's automatic
    Python logging bridge, delivering each record once to each destination.
    Configure Python's `dartpy` logger directly; the duplicate manual
    `enable_logging` and `disable_logging` controls are removed.
    Legacy C++ console diagnostics reach custom sinks without native threshold
    filtering; default console output is unchanged. Panel updates run on the
    GUI thread and retain the newest 2000 pending records.
    With a custom sink or inside the GUI App, unterminated `dt*` lines wait for
    an explicit flush, the next `dt*` statement on that thread, or thread exit;
    their terminal ordering relative to direct stdout/stderr writes can differ.
    [#3899](https://github.com/dartsim/dart/pull/3899),
    [#3854](https://github.com/dartsim/dart/pull/3854)
