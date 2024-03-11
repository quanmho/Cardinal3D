
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    float t_x0, t_x1;
    float t_y0, t_y1;
    float t_z0, t_z1;

    // if ray is parallel to an axis, check if ray.point is within the axis planes
    if(ray.dir.x == 0) {
        if(ray.point.x > min.x && ray.point.x < max.x) { // not considering grazing the plane as intersection for now
            t_x0 = -FLT_MAX;
            t_x1 = FLT_MAX;
        } else {
            t_x0 = FLT_MAX;
            t_x1 = -FLT_MAX;
        }
    } else {
        t_x0 = (min.x - ray.point.x) / ray.dir.x;
        t_x1 = (max.x - ray.point.x) / ray.dir.x;
        // keep 1 > 0 for easier code later on
        if(t_x0 > t_x1) {
            float temp = t_x0;
            t_x0 = t_x1;
            t_x1 = temp;
        }
    }

    if(ray.dir.y == 0) {
        if(ray.point.y > min.y && ray.point.y < max.y) { // not considering grazing the plane as intersection for now
            t_y0 = -FLT_MAX;
            t_y1 = FLT_MAX;
        } else {
            t_y0 = FLT_MAX;
            t_y1 = -FLT_MAX;
        }
    } else {
        t_y0 = (min.y - ray.point.y) / ray.dir.y;
        t_y1 = (max.y - ray.point.y) / ray.dir.y;
        if(t_y0 > t_y1) {
            float temp = t_y0;
            t_y0 = t_y1;
            t_y1 = temp;
        }
    }

    if(ray.dir.z == 0) {
        if(ray.point.z > min.z && ray.point.z < max.z) { // not considering grazing the plane as intersection for now
            t_z0 = -FLT_MAX;
            t_z1 = FLT_MAX;
        } else {
            t_z0 = FLT_MAX;
            t_z1 = -FLT_MAX;
        }
    } else {
        t_z0 = (min.z - ray.point.z) / ray.dir.z;
        t_z1 = (max.z - ray.point.z) / ray.dir.z;
        if(t_z0 > t_z1) {
            float temp = t_z0;
            t_z0 = t_z1;
            t_z1 = temp;
        }
    }

    // find overlap time interval
    float tmin = std::max(t_x0, t_y0);
    tmin = std::max(tmin, t_z0);

    float tmax = std::min(t_x1, t_y1);
    tmax = std::min(tmax, t_z1);

    // if the ray exits the box at neg t, it cannot intersect the box
    bool hit = (tmin < tmax) && (tmax > 0.0f);

    // check if interval within the given times vector
    // bool in_interval = (tmin > times.x) && (tmin < times.y);
    // ray starting inside the box can have tmin < 0 so tmin have no lower bound
    bool in_interval = (tmin < times.y);

    if(hit && in_interval) {
        times.x = tmin;
        times.y = (times.y == std::numeric_limits<float>::infinity()) ? tmax : std::max(times.y, tmax);
        return true;
    } else {
        return false;
    }

}
