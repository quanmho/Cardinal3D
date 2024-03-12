
#include "../rays/tri_mesh.h"
#include "debug.h"
#include <iostream>

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect
  
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    Vec3 min = hmin(v_0.position, v_1.position);
    min = hmin(min, v_2.position);

    Vec3 max = hmax(v_0.position, v_1.position);
    max = hmax(max, v_2.position);

    // check for flat box (also check for box with 1 edge or zero volume)
    bool flat_in_x = (min.x == max.x);
    bool flat_in_y = (min.y == max.y);
    bool flat_in_z = (min.z == max.z);

    // if flat, make the box 2 * half_len thick
    // if box is zero-volume, it would make a cube with side length 2 * half_len
    float half_len = 0.1f;
    if(flat_in_x) {
        min.x -= half_len;
        max.x += half_len;
    }

    if(flat_in_y) {
        min.y -= half_len;
        max.y += half_len;
    }

    if(flat_in_z) {
        min.z -= half_len;
        max.z += half_len;
    }

    BBox box(min, max);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    // See rays/tri_mesh.h for a description of this struct
    
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    // here just to avoid unused variable warnings, students should remove the following three lines.
    //(void)v_0;
    //(void)v_1;
    //(void)v_2;
    
    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the above three points.
    // Intersection should yield a ray t-value, and a hit point (u,v) on the surface of the triangle
    
    Vec3 e1 = v_1.position - v_0.position;
    Vec3 e2 = v_2.position - v_0.position;
    Vec3 s = ray.point - v_0.position;

    Vec3 e1xd = cross(e1, ray.dir);
    Vec3 sxe2 = cross(s, e2);

    float scalar = 1.0f / (dot(e1xd, e2));
    float u = -1.0f * dot(sxe2, ray.dir) * scalar;
    float v = dot(e1xd, s) * scalar;
    float t = -1.0f * dot(sxe2, e1) * scalar;


    bool valid_hit = (t >= ray.dist_bounds.x && t <= ray.dist_bounds.y); // was there an intersection?
    //std::cout << t << std::endl;

    Trace ret;

    if(valid_hit) {
        // 2D point-in-triangle test

        // since triangle is unit triangle, inside-tri can be defined as
        // the region within lines u=0, v=0, u=1-v
        float alpha = 1.0f - u - v;
        bool inside_tri = (alpha >= 0 && alpha <= 1) && (u >= 0) && (v >= 0);

        ret.origin = ray.point;
        ret.hit = inside_tri; // was there an intersection?
        if(ret.hit) {
            ray.dist_bounds.y = t;
        }
        ret.distance = t;                       // at what distance did the intersection occur?
        ret.position = ray.point + t * ray.dir; // where was the intersection?
        ret.normal = alpha * v_0.normal + u * v_1.normal + v * v_2.normal; // what was the surface normal at the intersection?
    } else {
        ret.origin = ray.point;
        ret.hit = false;       
        ret.distance = 0.0f;   
        ret.position = Vec3{}; 
        ret.normal = Vec3{};   
    }

    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
