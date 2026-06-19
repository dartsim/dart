# Testing

Use Pixi tasks from the repository root.

```bash
pixi run test
pixi run test-py
pixi run test-all
```

`pixi run test` runs the C++ test suite after building tests. `pixi run test-py`
runs the Python binding tests. `pixi run test-all` is the broad default gate for
the release branch.

For dependency minimization, run focused tests for the touched component and
then broaden when the dependency affects shared package, collision, constraint,
or GUI behavior.
