#!/usr/bin/env python3
"""Time a headless native avbd-demo2d source scene checkout."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import subprocess
import sys
import tempfile
from pathlib import Path
from textwrap import dedent
from typing import Any

SOURCE_REVISION = "74699a11f858"
SOURCE_REPOSITORY = "https://github.com/savant117/avbd-demo2d"

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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
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
  std::printf("  \"dynamic_bodies\": %d,\n", dynamicBodies);
  std::printf("  \"static_bodies\": %d,\n", staticBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"ground_width\": %.6f,\n", 100.0);
  std::printf("  \"ground_height\": %.6f,\n", 1.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_DYNAMIC_FRICTION_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneDynamicFriction(&solver);
  const float requestedMaxFriction = DYNAMIC_FRICTION_MAX_FRICTION;
  const float frictionScale = requestedMaxFriction / 5.0f;

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  int boxCount = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  float dynamicMinFriction = 1.0e30f;
  float dynamicMaxFriction = -1.0e30f;
  float maxInitialSpeed = 0.0f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
      ++boxCount;
      body->friction *= frictionScale;
      const float speed = std::sqrt(
          body->velocity.x * body->velocity.x +
          body->velocity.y * body->velocity.y);
      if (speed > maxInitialSpeed) {
        maxInitialSpeed = speed;
      }
      if (body->friction < dynamicMinFriction) {
        dynamicMinFriction = body->friction;
      }
      if (body->friction > dynamicMaxFriction) {
        dynamicMaxFriction = body->friction;
      }
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
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
  std::printf("  \"box_count\": %d,\n", boxCount);
  std::printf("  \"friction_samples\": %d,\n", boxCount);
  std::printf("  \"initial_speed\": %.6f,\n", maxInitialSpeed);
  std::printf("  \"requested_max_friction\": %.6f,\n", requestedMaxFriction);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"dynamic_min_friction\": %.6f,\n", dynamicMinFriction);
  std::printf("  \"dynamic_max_friction\": %.6f,\n", dynamicMaxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_STATIC_FRICTION_RUNNER = r"""
#include <chrono>
#include <cmath>
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
  int boxCount = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  float dynamicMinFriction = 1.0e30f;
  float dynamicMaxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
      ++boxCount;
      if (body->friction < dynamicMinFriction) {
        dynamicMinFriction = body->friction;
      }
      if (body->friction > dynamicMaxFriction) {
        dynamicMaxFriction = body->friction;
      }
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
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
  std::printf("  \"box_count\": %d,\n", boxCount);
  std::printf("  \"ramp_angle\": %.9g,\n", 3.14159f / 6.0f);
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
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
  std::printf("  \"pyramid_size\": %d,\n", 20);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_CARDS_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneCards(&solver);

  const float cardHeight = 0.2f * 2.0f;
  const float cardThickness = 0.001f * 2.0f;
  const float anglePositive = 25.0f * 3.14159f / 180.0f;
  const float angleNegative = -25.0f * 3.14159f / 180.0f;
  const float angleHorizontal = 0.5f * 3.14159f;

  int rigidBodies = 0;
  int dynamicBodies = 0;
  int staticBodies = 0;
  int cardCount = 0;
  int horizontalCardCount = 0;
  int negativeCardCount = 0;
  int positiveCardCount = 0;
  float minFriction = 1.0e30f;
  float maxFriction = -1.0e30f;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
    if (body->mass > 0.0f) {
      ++dynamicBodies;
      ++cardCount;
      if (std::fabs(body->position.z - angleHorizontal) < 1.0e-4f) {
        ++horizontalCardCount;
      } else if (std::fabs(body->position.z - angleNegative) < 1.0e-4f) {
        ++negativeCardCount;
      } else if (std::fabs(body->position.z - anglePositive) < 1.0e-4f) {
        ++positiveCardCount;
      }
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 5,\n");
  std::printf("  \"scene_name\": \"Cards\",\n");
  std::printf("  \"scene_builder\": \"sceneCards\",\n");
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
  std::printf("  \"card_count\": %d,\n", cardCount);
  std::printf("  \"card_levels\": %d,\n", 5);
  std::printf("  \"horizontal_card_count\": %d,\n", horizontalCardCount);
  std::printf("  \"leaning_card_count\": %d,\n", negativeCardCount + positiveCardCount);
  std::printf("  \"negative_card_count\": %d,\n", negativeCardCount);
  std::printf("  \"positive_card_count\": %d,\n", positiveCardCount);
  std::printf("  \"card_height\": %.6f,\n", cardHeight);
  std::printf("  \"card_thickness\": %.6f,\n", cardThickness);
  std::printf("  \"angle_positive\": %.9g,\n", anglePositive);
  std::printf("  \"angle_negative\": %.9g,\n", angleNegative);
  std::printf("  \"angle_horizontal\": %.9g,\n", angleHorizontal);
  std::printf("  \"ground_width\": %.6f,\n", 80.0);
  std::printf("  \"ground_height\": %.6f,\n", 4.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 11,\n");
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
  std::printf("  \"stack_box_count\": 20,\n");
  std::printf("  \"box_y_spacing\": %.6f,\n", 2.0);
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 12,\n");
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
  std::printf("  \"stack_ratio_box_count\": 6,\n");
  std::printf("  \"initial_size\": %.6f,\n", 1.0);
  std::printf("  \"size_multiplier\": %.6f,\n", 2.0);
  std::printf("  \"max_box_size\": %.6f,\n", 32.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_ROPE_RUNNER = r"""
#include <chrono>
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 6,\n");
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
  std::printf("  \"linear_point_joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"rope_links\": %d,\n", rigidBodies);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_NET_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneNet(&solver);

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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 16,\n");
  std::printf("  \"scene_name\": \"Net\",\n");
  std::printf("  \"scene_builder\": \"sceneNet\",\n");
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
  std::printf("  \"linear_point_joints\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"ground_bodies\": 1,\n");
  std::printf("  \"net_links\": 40,\n");
  std::printf("  \"falling_blocks\": 50,\n");
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_ROD_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneRod(&solver);

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
  int fixedJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (Joint* joint = dynamic_cast<Joint*>(force)) {
      ++joints;
      if (std::isinf(joint->stiffness[0])
          && std::isinf(joint->stiffness[1])
          && std::isinf(joint->stiffness[2])) {
        ++fixedJoints;
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 13,\n");
  std::printf("  \"scene_name\": \"Rod\",\n");
  std::printf("  \"scene_builder\": \"sceneRod\",\n");
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
  std::printf("  \"fixed_joints\": %d,\n", fixedJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"rod_links\": %d,\n", rigidBodies);
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
  int fixedJoints = 0;
  int finiteFixedJoints = 0;
  int ignoreCollisionPairs = 0;
  float minLinearStiffness = 1.0e30f;
  float maxLinearStiffness = -1.0e30f;
  float minAngularStiffness = 1.0e30f;
  float maxAngularStiffness = -1.0e30f;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (Joint* joint = dynamic_cast<Joint*>(force)) {
      ++joints;
      if (joint->stiffness[0] != 0.0f
          && joint->stiffness[1] != 0.0f
          && joint->stiffness[2] != 0.0f) {
        ++fixedJoints;
      }
      if (std::isfinite(joint->stiffness[0])
          && std::isfinite(joint->stiffness[1])
          && std::isfinite(joint->stiffness[2])) {
        ++finiteFixedJoints;
      }
      if (joint->stiffness[0] < minLinearStiffness) {
        minLinearStiffness = joint->stiffness[0];
      }
      if (joint->stiffness[0] > maxLinearStiffness) {
        maxLinearStiffness = joint->stiffness[0];
      }
      if (joint->stiffness[2] < minAngularStiffness) {
        minAngularStiffness = joint->stiffness[2];
      }
      if (joint->stiffness[2] > maxAngularStiffness) {
        maxAngularStiffness = joint->stiffness[2];
      }
    }
    if (dynamic_cast<IgnoreCollision*>(force) != nullptr) {
      ++ignoreCollisionPairs;
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 14,\n");
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
  std::printf("  \"fixed_joints\": %d,\n", fixedJoints);
  std::printf("  \"finite_stiffness_fixed_joints\": %d,\n", finiteFixedJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"soft_body_width\": %d,\n", 15);
  std::printf("  \"soft_body_height\": %d,\n", 5);
  std::printf("  \"soft_body_stacks\": %d,\n", 2);
  std::printf("  \"soft_body_cells\": %d,\n", dynamicBodies);
  std::printf("  \"diagonal_ignore_collision_pairs\": %d,\n", ignoreCollisionPairs);
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

_JOINT_GRID_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneJointGrid(&solver);

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
  int fixedJoints = 0;
  int ignoreCollisionPairs = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (Joint* joint = dynamic_cast<Joint*>(force)) {
      ++joints;
      if (std::isinf(joint->stiffness[0])
          && std::isinf(joint->stiffness[1])
          && std::isinf(joint->stiffness[2])) {
        ++fixedJoints;
      }
    }
    if (dynamic_cast<IgnoreCollision*>(force) != nullptr) {
      ++ignoreCollisionPairs;
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 15,\n");
  std::printf("  \"scene_name\": \"Joint Grid\",\n");
  std::printf("  \"scene_builder\": \"sceneJointGrid\",\n");
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
  std::printf("  \"fixed_joints\": %d,\n", fixedJoints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"joint_grid_width\": %d,\n", 25);
  std::printf("  \"joint_grid_height\": %d,\n", 25);
  std::printf("  \"joint_grid_cells\": %d,\n", rigidBodies);
  std::printf("  \"diagonal_ignore_collision_pairs\": %d,\n", ignoreCollisionPairs);
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
      if (std::isinf(joint->stiffness[0]) &&
          std::isinf(joint->stiffness[1]) &&
          joint->stiffness[2] == 0.0f) {
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 7,\n");
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
  std::printf("  \"rope_links\": %d,\n", rigidBodies);
  std::printf("  \"regular_link_count\": 19,\n");
  std::printf("  \"heavy_block_size\": %.6f,\n", 30.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_HANGING_ROPE_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneHangingRope(&solver);

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
      if (std::isinf(joint->stiffness[0]) &&
          std::isinf(joint->stiffness[1]) &&
          joint->stiffness[2] == 0.0f) {
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 8,\n");
  std::printf("  \"scene_name\": \"Hanging Rope\",\n");
  std::printf("  \"scene_builder\": \"sceneHangingRope\",\n");
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
  std::printf("  \"rope_links\": 50,\n");
  std::printf("  \"regular_link_count\": 49,\n");
  std::printf("  \"heavy_block_size\": %.6f,\n", 10.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_SPRING_RUNNER = r"""
#include <chrono>
#include <cmath>
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

  int springs = 0;
  float minSpringStiffness = 1.0e30f;
  float maxSpringStiffness = -1.0e30f;
  float restLength = -1.0f;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Spring* spring = dynamic_cast<Spring*>(force);
    if (spring != nullptr) {
      ++springs;
      if (spring->stiffness[0] < minSpringStiffness) {
        minSpringStiffness = spring->stiffness[0];
      }
      if (spring->stiffness[0] > maxSpringStiffness) {
        maxSpringStiffness = spring->stiffness[0];
      }
      restLength = spring->rest;
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 9,\n");
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
  std::printf("  \"joints\": 0,\n");
  std::printf("  \"distance_springs\": %d,\n", springs);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"spring_rest_length\": %.6f,\n", restLength);
  std::printf("  \"min_spring_stiffness\": %.6f,\n", minSpringStiffness);
  std::printf("  \"max_spring_stiffness\": %.6f,\n", maxSpringStiffness);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_SPRING_RATIO_RUNNER = r"""
#include <chrono>
#include <cmath>
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

  int springs = 0;
  int lowStiffnessSprings = 0;
  int highStiffnessSprings = 0;
  float minSpringStiffness = 1.0e30f;
  float maxSpringStiffness = -1.0e30f;
  float restLength = -1.0f;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Spring* spring = dynamic_cast<Spring*>(force);
    if (spring != nullptr) {
      ++springs;
      if (spring->stiffness[0] < minSpringStiffness) {
        minSpringStiffness = spring->stiffness[0];
      }
      if (spring->stiffness[0] > maxSpringStiffness) {
        maxSpringStiffness = spring->stiffness[0];
      }
      if (spring->stiffness[0] == 1000.0f) {
        ++lowStiffnessSprings;
      }
      if (spring->stiffness[0] == 1000000.0f) {
        ++highStiffnessSprings;
      }
      restLength = spring->rest;
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 10,\n");
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
  std::printf("  \"joints\": 0,\n");
  std::printf("  \"distance_springs\": %d,\n", springs);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"spring_link_count\": %d,\n", rigidBodies);
  std::printf("  \"spring_rest_length\": %.6f,\n", restLength);
  std::printf("  \"low_stiffness_springs\": %d,\n", lowStiffnessSprings);
  std::printf("  \"high_stiffness_springs\": %d,\n", highStiffnessSprings);
  std::printf("  \"min_spring_stiffness\": %.6f,\n", minSpringStiffness);
  std::printf("  \"max_spring_stiffness\": %.6f,\n", maxSpringStiffness);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_FRACTURE_RUNNER = r"""
#include <chrono>
#include <cmath>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneFracture(&solver);

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
  int breakableJoints = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    Joint* joint = dynamic_cast<Joint*>(force);
    if (joint != nullptr) {
      ++joints;
      if (std::isfinite(joint->fracture[2])) {
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
    if (joint != nullptr && std::isfinite(joint->fracture[2]) &&
        joint->stiffness[0] == 0.0f &&
        joint->stiffness[1] == 0.0f &&
        joint->stiffness[2] == 0.0f) {
      ++brokenJoints;
    }
  }

  std::printf("{\n");
  std::printf("  \"schema_version\": 1,\n");
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 18,\n");
  std::printf("  \"scene_name\": \"Fracture\",\n");
  std::printf("  \"scene_builder\": \"sceneFracture\",\n");
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
  std::printf("  \"breakable_joints\": %d,\n", breakableJoints);
  std::printf("  \"joint_connected_collision_pairs\": %d,\n", joints);
  std::printf("  \"collision_shapes\": %d,\n", rigidBodies);
  std::printf("  \"chain_links\": 11,\n");
  std::printf("  \"falling_blocks\": 15,\n");
  std::printf("  \"support_blocks\": 2,\n");
  std::printf("  \"break_force\": %.6f,\n", 500.0);
  std::printf("  \"min_friction\": %.6f,\n", minFriction);
  std::printf("  \"max_friction\": %.6f,\n", maxFriction);
  std::printf("  \"broken_joints_after_timing\": %d,\n", brokenJoints);
  std::printf("  \"final_time\": %.9g\n", solver.dt * (warmupSteps + steps));
  std::printf("}\n");
}
"""

_MOTOR_RUNNER = r"""
#include <chrono>
#include <cstdio>

#include "solver.h"
#include "scenes.h"

int main()
{
  Solver solver;
  sceneMotor(&solver);

  int rigidBodies = 0;
  for (Rigid* body = solver.bodies; body != 0; body = body->next) {
    ++rigidBodies;
  }

  int joints = 0;
  int motors = 0;
  for (Force* force = solver.forces; force != 0; force = force->next) {
    if (dynamic_cast<Joint*>(force) != nullptr) {
      ++joints;
    }
    if (dynamic_cast<Motor*>(force) != nullptr) {
      ++motors;
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
  std::printf("  \"source_demo\": \"avbd-demo2d\",\n");
  std::printf("  \"repository\": \"https://github.com/savant117/avbd-demo2d\",\n");
  std::printf("  \"source_revision\": \"74699a11f858\",\n");
  std::printf("  \"scene_index\": 17,\n");
  std::printf("  \"scene_name\": \"Motor\",\n");
  std::printf("  \"scene_builder\": \"sceneMotor\",\n");
  std::printf("  \"compiler\": \"COMPILER_NAME\",\n");
  std::printf("  \"compile_flags\": \"-std=c++17 -O3 -DNDEBUG\",\n");
  std::printf("  \"warmup_steps\": %d,\n", warmupSteps);
  std::printf("  \"steps\": %d,\n", steps);
  std::printf("  \"elapsed_ns\": %.0f,\n", elapsedNs);
  std::printf("  \"cpu_time_per_step_ns\": %.6f,\n", elapsedNs / steps);
  std::printf("  \"rigid_bodies\": %d,\n", rigidBodies);
  std::printf("  \"joints\": %d,\n", joints);
  std::printf("  \"motors\": %d,\n", motors);
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
        help="Checkout of savant117/avbd-demo2d at revision 74699a11f858.",
    )
    parser.add_argument(
        "--scene",
        choices=[
            "cards",
            "dynamic_friction",
            "fracture",
            "ground",
            "heavy_rope",
            "hanging_rope",
            "joint_grid",
            "motor",
            "net",
            "pyramid",
            "rod",
            "rope",
            "soft_body",
            "spring",
            "spring_ratio",
            "stack",
            "stack_ratio",
            "static_friction",
        ],
        default="motor",
        help="Source scene to time.",
    )
    parser.add_argument("--warmup-steps", type=int, default=128)
    parser.add_argument("--steps", type=int, default=200_000)
    parser.add_argument(
        "--dynamic-friction-max-friction",
        type=float,
        default=5.0,
        help=(
            "Maximum dynamic-box friction coefficient for the Dynamic Friction "
            "scene. Only valid with --scene dynamic_friction."
        ),
    )
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


def _cpp_float_literal(value: float) -> str:
    text = f"{value:.9g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return f"{text}f"


def _runner_source(args: argparse.Namespace) -> str:
    runners = {
        "cards": _CARDS_RUNNER,
        "dynamic_friction": _DYNAMIC_FRICTION_RUNNER,
        "fracture": _FRACTURE_RUNNER,
        "ground": _GROUND_RUNNER,
        "heavy_rope": _HEAVY_ROPE_RUNNER,
        "hanging_rope": _HANGING_ROPE_RUNNER,
        "joint_grid": _JOINT_GRID_RUNNER,
        "motor": _MOTOR_RUNNER,
        "net": _NET_RUNNER,
        "pyramid": _PYRAMID_RUNNER,
        "rod": _ROD_RUNNER,
        "rope": _ROPE_RUNNER,
        "soft_body": _SOFT_BODY_RUNNER,
        "spring": _SPRING_RUNNER,
        "spring_ratio": _SPRING_RATIO_RUNNER,
        "stack": _STACK_RUNNER,
        "stack_ratio": _STACK_RATIO_RUNNER,
        "static_friction": _STATIC_FRICTION_RUNNER,
    }
    return (
        dedent(runners[args.scene])
        .replace("WARMUP_STEPS", str(args.warmup_steps))
        .replace("TIMED_STEPS", str(args.steps))
        .replace(
            "DYNAMIC_FRICTION_MAX_FRICTION",
            _cpp_float_literal(args.dynamic_friction_max_friction),
        )
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
            "motor.cpp",
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
    if not math.isfinite(args.dynamic_friction_max_friction):
        raise ReferenceTimingError("--dynamic-friction-max-friction must be finite")
    if args.dynamic_friction_max_friction < 0.0:
        raise ReferenceTimingError(
            "--dynamic-friction-max-friction must be non-negative"
        )
    if args.scene != "dynamic_friction" and args.dynamic_friction_max_friction != 5.0:
        raise ReferenceTimingError(
            "--dynamic-friction-max-friction is only valid with "
            "--scene dynamic_friction"
        )

    with tempfile.TemporaryDirectory(prefix="avbd_demo2d_ref_") as temp_dir_text:
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
    print(f"Wrote avbd-demo2d reference timing: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
