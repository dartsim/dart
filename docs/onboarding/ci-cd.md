# CI And Release-Branch Checks

Use GitHub Actions as the hosted source of truth after a PR is opened. Locally,
run the smallest gate that proves the touched surface, then broaden when shared
runtime, package, or downstream behavior changes.

Useful commands:

```bash
pixi run lint
pixi run build
pixi run test
pixi run test-py
pixi run -e gazebo test-gz
```

For failing CI, inspect the exact run and job logs before changing code. Prefer
reproducing locally, but document when a hosted-platform failure cannot be
reproduced on the current machine.
