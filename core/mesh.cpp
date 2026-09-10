#include "mesh.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <map>


Hit intersect_triangle(const Vec3& a, const Vec3& b, const Vec3& c,
                       double reflectivity, const Vec3& origin, const Vec3& direction) {
    // Moller-Trumbore: barycentric coordinates locate the hit inside the patch.
    const Vec3 e1=b-a, e2=c-a, h=cross(direction,e2);
    const double determinant=dot(e1,h);
    if (std::fabs(determinant)<1e-12) return {};
    const Vec3 s=origin-a;
    const double u=dot(s,h)/determinant;
    if (u<0 || u>1) return {};
    const Vec3 q=cross(s,e1);
    const double v=dot(direction,q)/determinant;
    if (v<0 || u+v>1) return {};
    const double t=dot(e2,q)/determinant;
    if (t<=1e-8) return {};
    Vec3 normal=normalize(cross(e1,e2));
    // Two-sided diffuse patches, consistent with existing analytic surfaces.
    if (dot(normal,direction)>0) normal=normal*(-1);
    return {true,t,normal,reflectivity};
}

Hit intersect_mesh(const TriangleMesh& mesh, const Vec3& origin, const Vec3& direction) {
    Hit nearest;
    for (size_t i=0;i<mesh.triangles.size();++i) {
        const auto& f=mesh.triangles[i];
        Hit h=intersect_triangle(mesh.vertices[f[0]],mesh.vertices[f[1]],mesh.vertices[f[2]],
                                 mesh.reflectivities[i],origin,direction);
        if (h.valid && (!nearest.valid || h.t<nearest.t)) nearest=h;
    }
    return nearest;
}

TriangleMesh load_obj(const std::string& path, double scale, const Vec3& translation,
                      double reflectivity) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open OBJ: "+path);
    TriangleMesh mesh; mesh.scale=scale;
    std::map<std::string,double> materials;
    double current=reflectivity;
    std::string line;
    int line_number=0;
    while (std::getline(file,line)) {
        ++line_number;
        std::istringstream row(line); std::string tag; row>>tag;
        if (tag=="v") {
            Vec3 v;
            if (!(row>>v.x>>v.y>>v.z)) throw std::runtime_error("invalid OBJ vertex");
            mesh.vertices.push_back(v*scale+translation);
        } else if (tag=="mtllib") {
            std::string name; row>>name;
            std::ifstream mtl(path.substr(0,path.find_last_of("/\\")+1)+name);
            if (!mtl) throw std::runtime_error("missing MTL: "+name);
            std::string entry, material;
            while (std::getline(mtl,entry)) {
                std::istringstream r(entry); std::string key;r>>key;
                if (key=="newmtl") r>>material;
                if (key=="Kd") {double x,y,z;r>>x>>y>>z;materials[material]=(x+y+z)/3;}
            }
        } else if (tag=="usemtl") {
            std::string name;row>>name;
            current=materials.count(name)?materials[name]:reflectivity;
        } else if (tag=="f") {
            std::vector<int> ids; std::string token;
            while (row>>token) {
                if (token[0]=='#') break;
                int i=std::stoi(token.substr(0,token.find('/')));
                i=i<0?static_cast<int>(mesh.vertices.size())+i:i-1;
                if(i<0 || i>=static_cast<int>(mesh.vertices.size()))
                    throw std::runtime_error("OBJ index out of bounds at line "+std::to_string(line_number));
                ids.push_back(i);
            }
            if (ids.size()!=3) throw std::runtime_error("OBJ requires triangular faces; triangulate explicitly");
            Vec3 n=cross(mesh.vertices[ids[1]]-mesh.vertices[ids[0]],mesh.vertices[ids[2]]-mesh.vertices[ids[0]]);
            if(norm(n)<1e-12) throw std::runtime_error("degenerate OBJ triangle");
            mesh.triangles.push_back({ids[0],ids[1],ids[2]});
            mesh.normals.push_back(normalize(n));mesh.reflectivities.push_back(current);
        }
    }
    if(mesh.triangles.empty()) throw std::runtime_error("OBJ has no triangles");
    return mesh;
}

void transform_mesh(TriangleMesh& mesh,const Vec3& x,const Vec3& y,const Vec3& z,const Vec3& centre) {
    for(auto& vertex:mesh.vertices) {
        const Vec3 p=vertex-centre;vertex=centre+x*p.x+y*p.y+z*p.z;
    }
    for(auto& n:mesh.normals) n=x*n.x+y*n.y+z*n.z;
}
