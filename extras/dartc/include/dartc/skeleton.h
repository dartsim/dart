#pragma once

#include "dartc/common.h"

#ifdef __cplusplus
extern "C" {
#endif

DARTC_DECLARE_HANDLE(SkeletonId);

SkeletonId dart_skeleton_create(const char* name = "");

void dart_skeleton_destroy(SkeletonId skel);

const char* dart_skeleton_get_name(SkeletonId skel);

int dart_skeleton_get_num_dofs(SkeletonId skel);

void dart_skeleton_set_positions(
    SkeletonId skel, double* positions, int num_poisitions);

double* dart_skeleton_get_positions(SkeletonId skel);

void dart_skeleton_set_velocities(
    SkeletonId skel, double* velocities, int num_velocities);

double* dart_skeleton_get_velocities(SkeletonId skel);

void dart_skeleton_set_accelerations(
    SkeletonId skel, double* accelerations, int num_accelerations);

double* dart_skeleton_get_accelerations(SkeletonId skel);

#ifdef __cplusplus
}
#endif
