Section: API

  * Add C++ `World::stepMultiple(count, resetCommand)` to run counted ordinary
    steps with per-step force and command clearing. Concurrent or re-entrant
    stepping, compilation, scoped editing, and WorldState capture or restore
    on one World now throw `std::runtime_error` with the `WorldBusyError`
    prefix. Nested edits on their owning thread remain supported.
