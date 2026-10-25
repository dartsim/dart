Section: Python

  * dartpy raises the builtin exception that names each failure, such as
    `ValueError`, `KeyError`, `FileNotFoundError` or `RuntimeError`. A message
    from a DART rule starts with a stable prefix, such as `ModelFrozenError: `.
    Match the builtin and the prefix. Only three classes remain:
    `dartpy.errors.WorldBusyError`, `dartpy.gui.RenderingUnavailableError` and
    `dartpy.viewer.AssetsNotPrefetchedError`. Each derives directly from
    `RuntimeError`. This replaces the classes that #3844 added to
    `dartpy.errors`: `DartError`, `OperationError`, `ValidationError`,
    `NotFoundError`, `ModelFrozenError`, `CheckpointMismatchError` and
    `NameConflictError` are removed.
