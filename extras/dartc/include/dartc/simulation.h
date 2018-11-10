#pragma once

#include "dartc/common.h"
#include "dartc/skeleton.h"

#ifdef __cplusplus
extern "C" {
#endif

DARTC_DECLARE_HANDLE(WorldId);

WorldId dart_world_create(const char* name = "");

void dart_world_destroy(WorldId world);

const char* dart_world_get_name(WorldId world);

void dart_world_set_gravity(WorldId world, double x, double y, double z);

const double* dart_world_get_gravity(WorldId world);

double dart_world_get_gravity_x(WorldId world);

double dart_world_get_gravity_z(WorldId world);

double dart_world_get_gravity_y(WorldId world);

void dart_world_add_skeleton(WorldId world, SkeletonId skeleton);

int dart_world_get_num_skeletons(WorldId world);

SkeletonId dart_world_get_skeleton(WorldId world, int skeletonIndex);

void dart_world_remove_all_skeletons(WorldId world);

#ifdef __cplusplus
}
#endif
