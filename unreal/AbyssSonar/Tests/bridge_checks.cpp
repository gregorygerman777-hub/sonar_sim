#include "ResearchBridge.h"
#include "physics.h"
#include <cassert>
#include <iostream>
#include <iomanip>
int main(int argc,char** argv)
{
    assert(argc==2);
    using namespace ResearchBridge;
    const Vec3 original{1.2,4.5,-.7};const auto u=ToUnreal(original);const auto back=FromUnreal(u.x,u.y,u.z);
    assert(norm(original-back)<1e-14);
    // UE forward maps to core +Y and UE right maps to core +X.
    assert(DirectionFromUnreal(1,0,0).y==1);assert(DirectionFromUnreal(0,1,0).x==1);
    auto c=DefaultConfig();c.num_threads=1;c.num_elevation_subrays=512;Pose p;
    auto scene=Experiment(0,argv[1]);auto a=Ping(scene,p,c,false,42);
    std::vector<double> independent(a.raw.size());render(c,scene,p,independent.data());
    assert(a.raw==independent);
    scene.spheres[0].centre.z*=-1;auto b=Ping(scene,p,c,false,42);
    double peak=0,error=0;
    for(size_t i=0;i<a.raw.size();++i){peak=std::max(peak,a.raw[i]);error=std::max(error,std::abs(a.raw[i]-b.raw[i]));}
    assert(peak>0);assert(error/peak<1e-12);
    auto noisy=Ping(scene,p,c,true,42);auto repeat=Ping(scene,p,c,true,42);
    assert(noisy.raw==b.raw);assert(noisy.display==repeat.display);assert(noisy.display!=noisy.raw);
    auto before=noisy.raw;for(double v:noisy.display)DisplayLevel(v,peak,12,40);assert(before==noisy.raw);
    auto plus=project_vertex_to_bin({0,4,.35},p,c),minus=project_vertex_to_bin({0,4,-.35},p,c);
    assert(plus.observed&&minus.observed&&plus.range==minus.range&&plus.bearing==minus.bearing);
    auto meshScene=Experiment(3,argv[1]);assert(meshScene.meshes.size()==3);
    auto meshFrame=Ping(meshScene,p,c,false,42);assert(*std::max_element(meshFrame.raw.begin(),meshFrame.raw.end())>0);
    HighlightMask hi,shadow;Masks(a,.03,hi,shadow);
    for(size_t i=0;i<hi.size();++i)assert(!(hi[i]&&shadow[i]));
    std::cout<<std::setprecision(12)<<"PASS: coordinate round trip, axes, exact core parity, raw/display separation, seed repeatability, projection, OBJ rendering, mask separation\n";
    std::cout<<"elevation_flip_relative_error="<<error/peak<<"\n";
    std::cout<<"config=97 bearings x 256 ranges x 512 subrays; 600 kHz; top-hat; 1 thread\n";
}
