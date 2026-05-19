# Collision Reference Engines

This directory contains the FCL, Bullet, and ODE reference collision engines
used by DART's tests and benchmarks. They are not part of the installed DART
runtime surface.

The public legacy headers under `dart/collision/{fcl,bullet,ode}/` remain
native-backed compatibility facades for downstream source compatibility. Tests
and benchmarks that intentionally compare against external engines should
include headers from `dart/test/reference_collision/...` and link the
`dart-test-reference-{fcl,bullet,ode}` targets.
