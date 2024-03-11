
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    float a = 1.0f; // ray.dir.norm_squared always equals 1.0f because it's a unit vector
    float neg_b = -2.0f * dot(ray.point, ray.dir);
    float c = ray.point.norm_squared() - radius * radius;
    float det = neg_b * neg_b - 4.0f * a * c;

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
    
    if(det >= 0) {
        // has at least one solution
        float sqrt_det = sqrt(det);
        float t0 = (neg_b - sqrt_det) / (2.0f * a);
        float t1 = (neg_b + sqrt_det) / (2.0f * a);

        float t_intersect = -1.0f;
        if(t0 < ray.dist_bounds.y && t0 > ray.dist_bounds.x) {
            t_intersect = t0;
            ret.hit = true;
        } else if (t1 < ray.dist_bounds.y && t1 > ray.dist_bounds.x) {
            t_intersect = t1;
            ret.hit = true;
        }

        if(ret.hit) {
            ret.distance = t_intersect;  
            ret.position = ray.point + t_intersect * ray.dir;
            ret.normal = ret.position.unit();  // sphere is centered at the origin
            ray.dist_bounds.y = t_intersect;
        }

    } else {
        // no solution
    }

    
    return ret;
}

} // namespace PT
