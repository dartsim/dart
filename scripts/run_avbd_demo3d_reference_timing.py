#!/usr/bin/env python3
"""Time a headless native avbd-demo3d source scene checkout."""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import sys
import tempfile
from pathlib import Path
from textwrap import dedent
from typing import Any

SOURCE_REVISION = "7701bd427d55"
SOURCE_REPOSITORY = "https://github.com/savant117/avbd-demo3d"

_GROUND_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneGround(&solver);

  int rigidBodies = 0;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
  }

  int joints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 1,\n");
  std::printf("  \"scene_name\": \"Ground\",\n");
  std::printf("  \"scene_builder\": \"sceneGround\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_DYNAMIC_FRICTION_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneDynamicFriction(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 2,\n");
  std::printf("  \"scene_name\": \"Dynamic Friction\",\n");
  std::printf("  \"scene_builder\": \"sceneDynamicFriction\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_STATIC_FRICTION_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneStaticFriction(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  float dynamicMinFriction = 1.0e30f;
  float dynamicMaxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
    if (body->mass > 0.0f) {
      ++dynamicBodies;
      if (body->friction < dynamicMinFriction) {
        dynamicMinFriction = body->friction;
      }
      if (body->friction > dynamicMaxFriction) {
        dynamicMaxFriction = body->friction;
      }
    } else {
      ++staticBodies;
    }
  }

  int joints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 3,\n");
  std::printf("  \"scene_name\": \"Static Friction\",\n");
  std::printf("  \"scene_builder\": \"sceneStaticFriction\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"dynamic_min_friction\": %.6f,\n", dynamicMinFriction);
  std::printf("  \"dynamic_max_friction\": %.6f,\n", dynamicMaxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_PYRAMID_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  scenePyramid(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 4,\n");
  std::printf("  \"scene_name\": \"Pyramid\",\n");
  std::printf("  \"scene_builder\": \"scenePyramid\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"pyramid_size\": 16,\n");
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_ROPE_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneRope(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  int linearPointJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr) {
      ++joints;
      if (std::isinf(joint->stiffnessLin) && joint->stiffnessAng == 0.0f) {
        ++linearPointJoints;
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 5,\n");
  std::printf("  \"scene_name\": \"Rope\",\n");
  std::printf("  \"scene_builder\": \"sceneRope\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"linear_point_joints\": %d,\n", linearPointJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"rope_links\": 20,\n");
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_HEAVY_ROPE_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneHeavyRope(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  int linearPointJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr) {
      ++joints;
      if (std::isinf(joint->stiffnessLin) && joint->stiffnessAng == 0.0f) {
        ++linearPointJoints;
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 6,\n");
  std::printf("  \"scene_name\": \"Heavy Rope\",\n");
  std::printf("  \"scene_builder\": \"sceneHeavyRope\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"linear_point_joints\": %d,\n", linearPointJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"rope_links\": 20,\n");
  std::printf("  \"heavy_block_size\": 5.0,\n");
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_SPRING_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneSpring(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  int distanceSprings = 0;
  float minSpringStiffness = 1.0e30f;
  float maxSpringStiffness = -1.0e30f;
  float springRestLength = 0.0f;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
    Spring* spring = dynamic_cast<Spring*>(force);
    if (spring != nullptr) {
      ++distanceSprings;
      springRestLength = spring->rest;
      if (spring->stiffness < minSpringStiffness) {
        minSpringStiffness = spring->stiffness;
      }
      if (spring->stiffness > maxSpringStiffness) {
        maxSpringStiffness = spring->stiffness;
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 7,\n");
  std::printf("  \"scene_name\": \"Spring\",\n");
  std::printf("  \"scene_builder\": \"sceneSpring\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"distance_springs\": %d,\n", distanceSprings);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"min_spring_stiffness\": %.6f,\n", minSpringStiffness);
  std::printf("  \"max_spring_stiffness\": %.6f,\n", maxSpringStiffness);
  std::printf("  \"spring_rest_length\": %.6f,\n", springRestLength);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_SPRING_RATIO_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneSpringsRatio(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  int distanceSprings = 0;
  int highStiffnessSprings = 0;
  int lowStiffnessSprings = 0;
  float minSpringStiffness = 1.0e30f;
  float maxSpringStiffness = -1.0e30f;
  float springRestLength = 0.0f;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
    Spring* spring = dynamic_cast<Spring*>(force);
    if (spring != nullptr) {
      ++distanceSprings;
      springRestLength = spring->rest;
      if (spring->stiffness < minSpringStiffness) {
        minSpringStiffness = spring->stiffness;
      }
      if (spring->stiffness > maxSpringStiffness) {
        maxSpringStiffness = spring->stiffness;
      }
      if (spring->stiffness >= 1000.0f) {
        ++highStiffnessSprings;
      } else {
        ++lowStiffnessSprings;
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 8,\n");
  std::printf("  \"scene_name\": \"Spring Ratio\",\n");
  std::printf("  \"scene_builder\": \"sceneSpringsRatio\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"distance_springs\": %d,\n", distanceSprings);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"spring_link_count\": 8,\n");
  std::printf("  \"high_stiffness_springs\": %d,\n", highStiffnessSprings);
  std::printf("  \"low_stiffness_springs\": %d,\n", lowStiffnessSprings);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"min_spring_stiffness\": %.6f,\n", minSpringStiffness);
  std::printf("  \"max_spring_stiffness\": %.6f,\n", maxSpringStiffness);
  std::printf("  \"spring_rest_length\": %.6f,\n", springRestLength);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_STACK_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneStack(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 9,\n");
  std::printf("  \"scene_name\": \"Stack\",\n");
  std::printf("  \"scene_builder\": \"sceneStack\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"stack_box_count\": 10,\n");
  std::printf("  \"box_z_spacing\": %.6f,\n", 1.5);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_STACK_RATIO_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneStackRatio(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 10,\n");
  std::printf("  \"scene_name\": \"Stack Ratio\",\n");
  std::printf("  \"scene_builder\": \"sceneStackRatio\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"stack_ratio_box_count\": 4,\n");
  std::printf("  \"initial_size\": %.6f,\n", 1.0);
  std::printf("  \"size_multiplier\": %.6f,\n", 2.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_SOFT_BODY_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneSoftBody(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  int finiteStiffnessFixedJoints = 0;
  float minLinearStiffness = 1.0e30f;
  float maxLinearStiffness = -1.0e30f;
  float minAngularStiffness = 1.0e30f;
  float maxAngularStiffness = -1.0e30f;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr) {
      ++joints;
      if (std::isfinite(joint->stiffnessLin)
          && std::isfinite(joint->stiffnessAng)) {
        ++finiteStiffnessFixedJoints;
        if (joint->stiffnessLin < minLinearStiffness) {
          minLinearStiffness = joint->stiffnessLin;
        }
        if (joint->stiffnessLin > maxLinearStiffness) {
          maxLinearStiffness = joint->stiffnessLin;
        }
        if (joint->stiffnessAng < minAngularStiffness) {
          minAngularStiffness = joint->stiffnessAng;
        }
        if (joint->stiffnessAng > maxAngularStiffness) {
          maxAngularStiffness = joint->stiffnessAng;
        }
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 11,\n");
  std::printf("  \"scene_name\": \"Soft Body\",\n");
  std::printf("  \"scene_builder\": \"sceneSoftBody\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"fixed_joints\": %d,\n", finiteStiffnessFixedJoints);
  std::printf("  \"finite_stiffness_fixed_joints\": %d,\n", finiteStiffnessFixedJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"soft_body_width\": 4,\n");
  std::printf("  \"soft_body_depth\": 4,\n");
  std::printf("  \"soft_body_height\": 4,\n");
  std::printf("  \"soft_body_stacks\": 3,\n");
  std::printf("  \"soft_body_cells\": 192,\n");
  std::printf("  \"diagonal_ignore_collision_pairs\": 648,\n");
  std::printf("  \"min_linear_stiffness\": %.6f,\n", minLinearStiffness);
  std::printf("  \"max_linear_stiffness\": %.6f,\n", maxLinearStiffness);
  std::printf("  \"min_angular_stiffness\": %.6f,\n", minAngularStiffness);
  std::printf("  \"max_angular_stiffness\": %.6f,\n", maxAngularStiffness);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_BRIDGE_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneBridge(&solver);

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
    } else {
      ++staticBodies;
    }
    if (body->friction < minFriction) {
      minFriction = body->friction;
    }
    if (body->friction > maxFriction) {
      maxFriction = body->friction;
    }
  }

  int joints = 0;
  int linearPointJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr) {
      ++joints;
      if (std::isinf(joint->stiffnessLin) && joint->stiffnessAng == 0.0f) {
        ++linearPointJoints;
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 12,\n");
  std::printf("  \"scene_name\": \"Bridge\",\n");
  std::printf("  \"scene_builder\": \"sceneBridge\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"linear_point_joints\": %d,\n", linearPointJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"bridge_planks\": 40,\n");
  std::printf("  \"load_boxes\": 50,\n");
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_BREAKABLE_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneBreakable(&solver);

  int rigidBodies = 0;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
  }

  int joints = 0;
  int breakableJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr) {
      ++joints;
      if (std::isfinite(joint->fracture)) {
        ++breakableJoints;
      }
    }
  }

  const int warmupSteps = WARMUP_STEPS;
  const int steps = TIMED_STEPS;
  for (int i = 0; i < warmupSteps; ++i) {
    solver.step();
  }

  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < steps; ++i) {
    solver.step();
  }
  const auto end = std::chrono::steady_clock::now();
  const double elapsedNs =
      std::chrono::duration<double, std::nano>(end - start).count();

  int brokenJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr && joint->broken) {
      ++brokenJoints;
    }
  }

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo3d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo3d\",\n");
  std::printf("  \"source_revision\": \"7701bd427d55\",\n");
  std::printf("  \"scene_index\": 13,\n");
  std::printf("  \"scene_name\": \"Breakable\",\n");
  std::printf("  \"scene_builder\": \"sceneBreakable\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"breakable_joints\": %d,\n", breakableJoints);
  std::printf("  \"broken_joints_after_timing\": %d,\n", brokenJoints);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""


class ReferenceTimingError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-dir",
        type=Path,
        required=True,
        help="Checkout of savant117/avbd-demo3d at revision 7701bd427d55.",
    )
    parser.add_argument(
        "--scene",
        choices=[
            "bridge",
            "breakable",
            "dynamic_friction",
            "ground",
            "heavy_rope",
            "pyramid",
            "rope",
            "soft_body",
            "spring",
            "spring_ratio",
            "stack",
            "stack_ratio",
            "static_friction",
        ],
        default="breakable",
        help="Source scene to time.",
    )
    parser.add_argument("--warmup-steps", type=int, default=128)
    parser.add_argument("--steps", type=int, default=20_000)
    parser.add_argument(
        "--cxx",
        default=os.environ.get("CXX", "g++"),
        help="C++ compiler used for the temporary runner.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output timing JSON path.",
    )
    return parser.parse_args(argv)


def _source_file(source_dir: Path, name: str) -> Path:
    path = source_dir / "source" / name
    if not path.is_file():
        raise ReferenceTimingError(f"{path}: required source file not found")
    return path


def _runner_source(args: argparse.Namespace) -> str:
    runners = {
        "bridge": _BRIDGE_RUNNER,
        "breakable": _BREAKABLE_RUNNER,
        "dynamic_friction": _DYNAMIC_FRICTION_RUNNER,
        "ground": _GROUND_RUNNER,
        "heavy_rope": _HEAVY_ROPE_RUNNER,
        "pyramid": _PYRAMID_RUNNER,
        "rope": _ROPE_RUNNER,
        "soft_body": _SOFT_BODY_RUNNER,
        "spring": _SPRING_RUNNER,
        "spring_ratio": _SPRING_RATIO_RUNNER,
        "stack": _STACK_RUNNER,
        "stack_ratio": _STACK_RATIO_RUNNER,
        "static_friction": _STATIC_FRICTION_RUNNER,
    }
    runner = runners[args.scene]
    return (
        dedent(runner)
        .replace("WARMUP_STEPS", str(args.warmup_steps))
        .replace("TIMED_STEPS", str(args.steps))
        .replace("COMPILER_NAME", args.cxx)
    )


def _compile_command(
    args: argparse.Namespace,
    runner: Path,
    executable: Path,
) -> list[str]:
    source_dir = args.source_dir.resolve()
    source_files = [
        _source_file(source_dir, name)
        for name in (
            "solver.cpp",
            "rigid.cpp",
            "force.cpp",
            "joint.cpp",
            "spring.cpp",
            "manifold.cpp",
            "collide.cpp",
        )
    ]
    command = [
        args.cxx,
        "-std=c++17",
        "-O3",
        "-DNDEBUG",
        f"-I{source_dir / 'source'}",
        str(runner),
        *(str(path) for path in source_files),
    ]
    if platform.system() == "Darwin":
        command.extend(["-framework", "OpenGL"])
    else:
        command.append("-lGL")
    command.extend(["-o", str(executable)])
    return command


def _load_runner_json(stdout: str) -> dict[str, Any]:
    data = json.loads(stdout)
    if not isinstance(data, dict):
        raise ReferenceTimingError("reference timing runner returned non-object JSON")
    return data


def run_reference_timing(args: argparse.Namespace) -> dict[str, Any]:
    if args.warmup_steps < 0:
        raise ReferenceTimingError("--warmup-steps must be non-negative")
    if args.steps < 1:
        raise ReferenceTimingError("--steps must be positive")

    with tempfile.TemporaryDirectory(prefix="avbd_demo3d_ref_") as temp_dir_text:
        temp_dir = Path(temp_dir_text)
        runner = temp_dir / "reference_runner.cpp"
        executable = temp_dir / "reference_runner"
        runner.write_text(_runner_source(args), encoding="utf-8")

        compile_command = _compile_command(args, runner, executable)
        subprocess.run(compile_command, check=True)
        completed = subprocess.run(
            [str(executable)],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )

    timing = _load_runner_json(completed.stdout)
    source_root = str(args.source_dir.resolve())
    temp_root = str(temp_dir)
    timing["compile_command"] = [
        part.replace(source_root, "<source-dir>").replace(temp_root, "<temp-dir>")
        for part in compile_command
    ]
    return timing


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        timing = run_reference_timing(args)
    except (OSError, subprocess.CalledProcessError, ReferenceTimingError) as exc:
        raise SystemExit(str(exc)) from exc
    write_json(args.output, timing)
    print(f"Wrote avbd-demo3d reference timing: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
