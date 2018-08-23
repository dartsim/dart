#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef void* SkeletonId;

SkeletonId dart_create_skeleton(const char* name = "");

void dart_destroy_skeleton(SkeletonId skel);

const char* dart_skeleton_get_name(SkeletonId skel);

int dart_skeleton_get_num_dofs(SkeletonId skel);

#ifdef __cplusplus
}
#endif
