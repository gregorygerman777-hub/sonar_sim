#include "reconstruction.h"
#include <cmath>
#include <algorithm>
ProjectedBin project_vertex_to_bin(const Vec3& point, const Pose& pose, const SonarConfig& cfg) {
    const Vec3 offset=point-pose.position;
    const double x=dot(offset,pose.x_axis), y=dot(offset,pose.y_axis), z=dot(offset,pose.z_axis);
    const double range=norm(offset);
    if (range<=0) return {};
    const double bearing=std::atan2(x,y), elevation=std::atan2(z,std::hypot(x,y));
    const double fov=cfg.horizontal_fov_deg*M_PI/180, beam=cfg.vertical_beamwidth_deg*M_PI/180;
    // Elevation is used only to check beam support; it is absent from the pixel address.
    int a=std::floor((bearing/fov+0.5)*cfg.num_azimuth_bins);
    int r=std::floor(range/cfg.max_range_m*cfg.num_range_bins);
    bool observed=a>=0 && a<cfg.num_azimuth_bins && r>=0 && r<cfg.num_range_bins && std::fabs(elevation)<=beam/2;
    return {observed,a,r,elevation};
}
bool voxel_is_consistent(const Voxel& voxel, const SonarPose& pose,
                         const HighlightMask& highlight, const ShadowMask& shadow) {
    const auto p=project_vertex_to_bin(voxel.centre,pose.pose,pose.config);
    if (!p.observed) return true; // No measurement cannot establish empty space.
    const int na=pose.config.num_azimuth_bins,nr=pose.config.num_range_bins;
    for(int a=std::max(0,p.bearing-pose.bearing_tolerance_bins);a<=std::min(na-1,p.bearing+pose.bearing_tolerance_bins);++a)
        for(int r=std::max(0,p.range-pose.range_tolerance_bins);r<=std::min(nr-1,p.range+pose.range_tolerance_bins);++r)
            if(highlight[a*nr+r] || shadow[a*nr+r]) return true;
    return false;
}
void carve_voxels(const std::vector<Vec3>& points,const SonarPose& pose,
                  const HighlightMask& highlight,const ShadowMask& shadow,unsigned char* kept) {
    for(size_t i=0;i<points.size();++i)
        kept[i]=kept[i] && voxel_is_consistent({points[i]},pose,highlight,shadow);
}
