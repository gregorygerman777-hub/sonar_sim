#pragma once
#include "simulator.h"
struct ProjectedBin { bool observed=false; int bearing=0, range=0; double elevation=0; };
ProjectedBin project_vertex_to_bin(const Vec3&, const Pose&, const SonarConfig&);
struct Voxel { Vec3 centre; };
struct SonarPose { Pose pose; SonarConfig config; int bearing_tolerance_bins=0, range_tolerance_bins=0; };
using HighlightMask = std::vector<unsigned char>;
using ShadowMask = std::vector<unsigned char>;
bool voxel_is_consistent(const Voxel&, const SonarPose&, const HighlightMask&, const ShadowMask&);
void carve_voxels(const std::vector<Vec3>&, const SonarPose&, const HighlightMask&, const ShadowMask&, unsigned char*);
