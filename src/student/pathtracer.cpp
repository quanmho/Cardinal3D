
#include "../rays/pathtracer.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"
#include <iostream>

namespace PT {

// Return the radiance along a ray entering the camera and landing on a
// point within pixel (x,y) of the output image.
//
Spectrum Pathtracer::trace_pixel(size_t x, size_t y) {

    Vec2 xy((float)x, (float)y);
    Vec2 wh((float)out_w, (float)out_h);

    // TODO (PathTracer): Task 1

    // Generate a sample within the pixel with coordinates xy and return the
    // incoming light using trace_ray.

    // If n_samples is 1, please send the ray through the center of the pixel.
    // If n_samples > 1, please send the ray through any random point within the pixel

    // Tip: consider making a call to Samplers::Rect::Uniform

    // Tip: you may want to use log_ray for debugging. Given ray t, the following lines
    // of code will log .03% of all rays (see util/rand.h) for visualization in the app.
    // see student/debug.h for more detail.
    //if (RNG::coin_flip(0.0003f))
    //    log_ray(out, 10.0f);

    // As an example, the code below generates a ray through the bottom left of the
    // specified pixel
    //Ray out = camera.generate_ray(xy / wh);

    // Compute the normalized screen space point

    Vec2 norm_xy;
    if(n_samples == 1) {
        Vec2 s_xy = Vec2(xy.x + 0.5f, xy.y + 0.5f);
        norm_xy = Vec2(s_xy.x / wh.x, s_xy.y / wh.y);
    } else {
        Samplers::Rect::Uniform rect_samp = Samplers::Rect::Uniform();
        float pdf;
        Vec2 s_xy = rect_samp.sample(pdf);
        norm_xy = Vec2(xy.x + s_xy.x, xy.y + s_xy.y); // add the random offset
        norm_xy.x = norm_xy.x / wh.x; // normalize the pixel location
        norm_xy.y = norm_xy.y / wh.y; // normalize the pixel location

    }

    Ray out = camera.generate_ray(norm_xy);

    //if(RNG::coin_flip(0.0005f)) {
    //    log_ray(out, 10.0f);
    //}

    
    //if((xy.x == 0) && (xy.y == 0)) {
    //    log_ray(out, 10.0f);
    //}

    Spectrum final = Spectrum(0.0f);
    Vec3 xyz_total = Vec3(0.0f);
    for(float nm = 420.0f; nm <= 620.0f; nm = nm + 10.0f) {
        Ray wave_samp = Ray(out.point, out.dir);
        wave_samp.wavelength = nm;
        Vec3 xyz = xyzfit_1931(nm);
        Spectrum rgb_scalar = xyz2srgb(xyz);
        Spectrum radiance = trace_ray(wave_samp);
        final += rgb_scalar * radiance;
        //Vec3 ret_xyz = srgb2xyz(Vec3(radiance.r, radiance.g, radiance.b));
        //xyz_total += xyz * radiance.luma(); // assume white light, all white mat so radiance of all components are the same
    }

    //xyz_total *= 1.0f / 20.0f;
    //final = xyz2srgb(xyz_total);

    final *= 1.0f / 20.0f;

    //final = trace_ray(out);

    return final;
}

Spectrum Pathtracer::trace_ray(const Ray& ray) {

    if(ray.depth > Pathtracer::max_depth) {
        return Spectrum(0.0f);
    }

    //if(true) {
    //    Spectrum color = (ray.from_discrete) ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
    //    log_ray(ray, 2.0f, color);
    //}

    // Trace ray into scene. If nothing is hit, sample the environment
    Trace hit = scene.hit(ray);
    if(!hit.hit) {
        if(env_light.has_value()) {
            return env_light.value().sample_direction(ray.dir);
        }
        return {};
    }

    // If we're using a two-sided material, treat back-faces the same as front-faces
    const BSDF& bsdf = materials[hit.material];
    if(!bsdf.is_sided() && dot(hit.normal, ray.dir) > 0.0f) {
        hit.normal = -hit.normal;
    }

    // Set up a coordinate frame at the hit point, where the surface normal becomes {0, 1, 0}
    // This gives us out_dir and later in_dir in object space, where computations involving the
    // normal become much easier. For example, cos(theta) = dot(N,dir) = dir.y!
    Mat4 object_to_world = Mat4::rotate_to(hit.normal);
    Mat4 world_to_object = object_to_world.T();
    Vec3 out_dir = world_to_object.rotate(ray.point - hit.position).unit();

    // Debugging: if the normal colors flag is set, return the normal color
    if(debug_data.normal_colors) return Spectrum::direction(hit.normal);

    // Now we can compute the rendering equation at this point.
    // We split it into two stages:
    //  1. sampling direct lighting (i.e. directly connecting the current path to
    //     each light in the scene)
    //  2. sampling the BSDF to create a new path segment

    // TODO (PathTracer): Task 4
    // The starter code sets radiance_out to (0.25,0.25,0.25) so that you can test your geometry
    // queries before you implement real lighting in Tasks 4 and 5. (i.e, anything that gets hit is not black.)
    // You should change this to (0,0,0) and accumulate the direct and indirect lighting computed below.
    //Spectrum radiance_out = Spectrum(0.25f);
    Spectrum radiance_out = Spectrum(0.0f);
    {

        // lambda function to sample a light. Called in loop below.
        auto sample_light = [&](const auto& light) {
            // If the light is discrete (e.g. a point light), then we only need
            // one sample, as all samples will be equivalent
            int samples = light.is_discrete() ? 1 : (int)n_area_samples;
            for(int i = 0; i < samples; i++) {

                // Grab a sample of the light source. See rays/light.h for definition of this struct.
                // Most importantly for Task 4, it contains the distance to the light from hit.position. 
                Light_Sample sample = light.sample(hit.position);
                Vec3 in_dir = world_to_object.rotate(sample.direction);

                // If the light is below the horizon, ignore it
                float cos_theta = in_dir.y;
                if(cos_theta <= 0.0f) continue;

                // If the BSDF has 0 throughput in this direction, ignore it.
                // This is another opportunity to do Russian roulette on low-throughput rays,
                // which would allow us to skip the shadow ray cast, increasing efficiency.
                Spectrum attenuation = bsdf.evaluate(out_dir, in_dir);
                if(attenuation.luma() == 0.0f) continue;

                // TODO (PathTracer): Task 4
                // Construct a shadow ray and compute whether the intersected surface is
                // in shadow. Only accumulate light if not in shadow.

                // Tip: since you're creating the shadow ray at the intersection point, it may
                // intersect the surface at time=0. Similarly, if the ray is allowed to have
                // arbitrary length, it will hit the light it was cast at. Therefore, you should
                // modify the time_bounds of your shadow ray to account for this. Using EPS_F is
                // recommended.

                // Note: that along with the typical cos_theta, pdf factors, we divide by samples.
                // This is because we're doing another monte-carlo estimate of the lighting from
                // area lights here.
                
                Ray shadow_r = Ray(hit.position, sample.direction);
                shadow_r.dist_bounds.y = sample.distance - EPS_F;
                shadow_r.dist_bounds.x = EPS_F;

                Trace hit_r = scene.hit(shadow_r);
                if(!hit_r.hit) {
                  radiance_out += (cos_theta / (samples * sample.pdf)) * sample.radiance * attenuation;
                }
            }
        };

        // If the BSDF is discrete (i.e. uses dirac deltas/if statements), then we are never
        // going to hit the exact right direction by sampling lights, so ignore them.
        if(!bsdf.is_discrete()) {

            // loop over all the lights and accumulate radiance.
            for(const auto& light : lights)
                sample_light(light);
            if(env_light.has_value())
                sample_light(env_light.value());
        }
    }

    // TODO (PathTracer): Task 5
    // Compute an indirect lighting estimate using path tracing with Monte Carlo.

    // (1) Ray objects have a depth field; if it reaches max_depth, you should
    // terminate the path.

    // (2) Randomly select a new ray direction (it may be reflection or transmittance
    // ray depending on surface type) using bsdf.sample()

    // (3) Compute the throughput of the recursive ray. This should be the current ray's
    // throughput scaled by the BSDF attenuation, cos(theta), and BSDF sample PDF.
    // Potentially terminate the path using Russian roulette as a function of the new throughput.
    // Note that allowing the termination probability to approach 1 may cause extra speckling.

    // (4) Create new scene-space ray and cast it to get incoming light. As with shadow rays, you
    // should modify time_bounds so that the ray does not intersect at time = 0. Remember to
    // set the new throughput and depth values.

    // (5) Add contribution due to incoming light with proper weighting. Remember to add in
    // the BSDF sample emissive term.
    
    //if(bsdf.is_discrete() && ray.wavelength > 0.0f) {

    BSDF_Sample bsdf_samp = bsdf.sample(out_dir, ray.wavelength); // wavelength here doesn't matter

    if(ray.depth == 0 || ray.from_discrete) {
        radiance_out += bsdf_samp.emissive;
    }

    radiance_out += get_radiance_bsdf(bsdf_samp, bsdf.is_discrete(), object_to_world, ray, hit);
    


    

    return radiance_out;
}

Spectrum Pathtracer::get_radiance_bsdf(BSDF_Sample bsdf_samp, bool is_discrete, Mat4 object_to_world, Ray ray, Trace hit) {

    Spectrum radiance_out = Spectrum(0.0f);
    
    float cos_theta = (is_discrete) ? 1.0f : abs(bsdf_samp.direction.y);
    Vec3 in_dir = object_to_world.rotate(bsdf_samp.direction);
    // cos_theta = dot(in_dir, hit.normal);
    //  throughput for new ray
    Spectrum throughput_new = ray.throughput * (bsdf_samp.attenuation * cos_theta) / bsdf_samp.pdf;
    throughput_new.r = std::min(std::max(throughput_new.r, 0.0f), 1.0f);
    throughput_new.g = std::min(std::max(throughput_new.g, 0.0f), 1.0f);
    throughput_new.b = std::min(std::max(throughput_new.b, 0.0f), 1.0f);

    // float q = 1 - std::max(std::max(throughput_new.r, throughput_new.g), throughput_new.b);
    float q = 1 - throughput_new.luma();
    bool terminate = RNG::unit() < q;
    terminate = false;
    if(!terminate) {
        Ray new_ray = Ray(hit.position, in_dir);
        new_ray.dist_bounds.x = EPS_F;
        // throughput_new *= 1.0f / (1.0f - q);
        new_ray.throughput = throughput_new;
        new_ray.depth = ray.depth + 1;
        new_ray.from_discrete = is_discrete;
        new_ray.wavelength = ray.wavelength;
        radiance_out += throughput_new * trace_ray(new_ray);
    }

    return radiance_out;
}

} // namespace PT
