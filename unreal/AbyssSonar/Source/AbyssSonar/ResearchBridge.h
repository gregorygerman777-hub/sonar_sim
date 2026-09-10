#pragma once
#include "simulator.h"
#include "mesh.h"
#include "reconstruction.h"
#include <algorithm>
#include <cmath>
#include <string>

namespace ResearchBridge
{
// Unreal: X forward, Y right, Z up, centimetres. Core: X right, Y forward, Z up, metres.
// This swap changes handedness; reverse triangle winding when drawing core OBJ meshes.
inline Vec3 FromUnreal(double x,double y,double z) { return {y/100,x/100,z/100}; }
inline Vec3 ToUnreal(const Vec3& p) { return {p.y*100,p.x*100,p.z*100}; }
inline Vec3 DirectionFromUnreal(double x,double y,double z) { return {y,x,z}; }
inline SonarConfig DefaultConfig()
{
    SonarConfig c;
    c.num_azimuth_bins=97; c.num_range_bins=256; c.num_elevation_subrays=128;
    c.horizontal_fov_deg=60; c.vertical_beamwidth_deg=30; c.frequency_hz=600000;
    c.num_threads=4; c.beam_mode=1; c.max_range_m=10;
    return c;
}
struct Frame
{
    SonarConfig config;
    Pose pose;
    std::vector<double> raw;
    std::vector<double> display;
};
inline Frame Ping(const Scene& scene,const Pose& pose,const SonarConfig& config,bool speckle,unsigned seed)
{
    Frame f; f.config=config; f.pose=pose;
    f.raw.resize(config.num_azimuth_bins*config.num_range_bins);
    render(config,scene,pose,f.raw.data());
    f.display=f.raw;
    if(speckle) apply_speckle(f.display.data(),static_cast<int>(f.display.size()),seed);
    return f;
}
inline double DisplayLevel(double raw,double reference,double gainDb,double dynamicDb)
{
    if(raw<=0 || reference<=0) return 0;
    return std::clamp((10*std::log10(raw/reference)+gainDb+dynamicDb)/dynamicDb,0.,1.);
}
inline void Masks(const Frame& f,double threshold,HighlightMask& hi,ShadowMask& shadow)
{
    hi.assign(f.raw.size(),0);shadow.assign(f.raw.size(),0);
    const double peak=*std::max_element(f.raw.begin(),f.raw.end());
    for(int a=0;a<f.config.num_azimuth_bins;++a)
    {
        bool seen=false;
        for(int r=0;r<f.config.num_range_bins;++r)
        {
            int i=a*f.config.num_range_bins+r;
            hi[i]=f.raw[i]>0 && f.raw[i]>peak*threshold;
            seen=seen || hi[i]; shadow[i]=seen && !hi[i];
        }
    }
}
inline Scene Experiment(int mode,const std::string& assets)
{
    Scene s;
    if(mode==0) // Symmetric isolated target; no seabed, texture or multipath breaks the symmetry.
        s.spheres.push_back({{0,4,0.35},0.16,0.8,{}});
    else if(mode==1)
    {
        s.planes.push_back({{0,0,-1},{0,0,1},0.08,{}});
        s.spheres.push_back({{0,4,-0.5},0.5,0.8,{}});
    }
    else if(mode==2)
        s.spheres.push_back({{0,2,-0.15},0.08,0.8,{}});
    else
    {
        s.planes.push_back({{0,0,-1.5},{0,0,1},0.04,{}});
        s.meshes.push_back(load_obj(assets+"/concave_table.obj",1.4,{0,4,-0.6},0.8));
        s.meshes.push_back(load_obj(assets+"/industrial_pipe.obj",0.7,{-1,5,-1},0.6));
        s.meshes.push_back(load_obj(assets+"/coral_rock.obj",1,{1.4,6,-1},0.3));
    }
    return s;
}
}
