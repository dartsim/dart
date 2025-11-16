# Multi Pendulum

This tutorial demonstrates key interactions with DART's dynamics API during
simulation: building a skeletal model, applying forces, tuning implicit springs
and dampers, and adding simple constraints. Each lesson includes both the C++
and dartpy snippets so you can follow along in your preferred language.

Please reference the source code in
[**tutorials/tutorial_multi_pendulum/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_multi_pendulum/main.cpp),
[**tutorials/tutorial_multi_pendulum_finished/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_multi_pendulum_finished/main.cpp),
[**python/tutorials/01_multi_pendulum/main.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/01_multi_pendulum/main.py),
and
[**python/tutorials/01_multi_pendulum/main_finished.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/01_multi_pendulum/main_finished.py).

## Lesson 0: Simulate a passive multi-pendulum

We create a five-link pendulum swinging under gravity, attach it to a
`World`/viewer, and let DART drive the simulation.

```{code-tabs}
---
label: create-skeleton
---
lang: cpp
source: tutorials/tutorial_multi_pendulum/main.cpp
start-after: // Lesson 0: Skeleton creation (snippet)
end-before: // End snippet
---
lang: python
source: python/tutorials/01_multi_pendulum/main.py
start-after: # Lesson 0: Skeleton creation (snippet)
end-before: # End snippet
```
