
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    //
    // The input screen_coord is a normalized screen coordinate [0,1]^2
    //
    // You need to transform this 2D point into a 3D position on the sensor plane, which is
    // located one unit away from the pinhole in camera space (aka view space).
    //
    // You'll need to compute this position based on the vertial field of view
    // (vert_fov) of the camera, and the aspect ratio of the output image (aspect_ratio).
    //
    // Tip: compute the ray direction in view space and use
    // the camera space to world space transform (iview) to transform the ray back into world space.

    // compute the 3D position on the sensor plane in camera space
    
    Vec3 r_start = Vec3(0, 0, 0);
    Vec3 r_dir;

    // transform z into world space to compute the corners of the sensor plane
    Vec3 pt_plane = Vec3(screen_coord.x - 0.5f, screen_coord.y - 0.5f, -1.0f);

    float vert_scalar = tanf(Radians(vert_fov) / 2) / 0.5f;

    pt_plane.y *= vert_scalar;
    pt_plane.x *= vert_scalar * aspect_ratio;


    r_dir = pt_plane - r_start;

    Ray r = Ray(r_start, r_dir);
    r.transform(iview);

    return r;
}
