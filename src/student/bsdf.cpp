
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    Vec3 dir_norm = dir.unit();
    Vec3 out_dir = -1.0f * dir_norm + Vec3(0.0f, 2.0f * (dir_norm.y), 0.0f);
    return out_dir;
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    Vec3 out_dir_norm = out_dir.unit();
    Vec3 in_dir;
    float n_out, n_in;
    float cos_adj = out_dir_norm.y;
    if(out_dir_norm.y > 0) {
        n_out = 1.0f;
        n_in = index_of_refraction;
    } else { // assume if out_dir_norm.y == 0, out_dir is currently in the object
        n_in = index_of_refraction;
        n_out = 1.0f;
        cos_adj = -1.0f * out_dir_norm.y;
    }

    // Taken from CS148
    float n_ratio = n_out / n_in;
    float sqrt_term = 1.0f - (float)pow(n_ratio, 2) * (1.0f - (float)pow(cos_adj, 2));

    was_internal = (sqrt_term < 0);

    if(!was_internal) {
        in_dir = Vec3(0.0f, n_ratio * cos_adj - (float)sqrt(sqrt_term), 0.0f) - n_ratio * out_dir_norm;
    }

    return in_dir;
    //return -1.0f * out_dir;
}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir, float wavelength = 0) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.attenuation = evaluate(Vec3(0.0f), Vec3(0.0f)); // Lambertian evaluate doesn't use the vectors

    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir, float wavelength = 0) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
    ret.attenuation = reflectance; // What is the ratio of reflected/incoming light?
    ret.direction = reflect(out_dir);       // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir, float wavelength = 0) const {

    // TODO (PathTracer): Task 6

    // Implement glass BSDF.
    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    BSDF_Sample ret;
    ret.attenuation = Spectrum(); // What is the ratio of reflected/incoming light?
    ret.direction = Vec3();       // What direction should we sample incoming light from?
    ret.pdf = 0.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)

    // Compute Fresnel coefficient
    float n1 = 1.0f;
    float n2 = index_of_refraction;
    float cos_theta = abs(out_dir.y);
    float R_0 = (float)pow((n1 - n2) / (n1 + n2), 2);
    float F_r = R_0 + (1.0f - R_0) * (float)(pow(1.0f - cos_theta, 5));

    if(RNG::coin_flip(F_r)) {
        ret.attenuation = reflectance;    // What is the ratio of reflected/incoming light?
        ret.direction = reflect(out_dir); // What direction should we sample incoming light from?
        ret.pdf = F_r; // Was was the PDF of the sampled direction? (In this case, the PMF)
    } else {
        bool was_internal;
        ret.attenuation = transmittance; // What is the ratio of reflected/incoming light?
        ret.direction = refract(out_dir, index_of_refraction, was_internal); // What direction should we sample incoming light from?
        if(was_internal) {
            ret.direction = reflect(out_dir);
        }
        ret.pdf = 1.0f - F_r; // Was was the PDF of the sampled direction? (In this case, the PMF)
    }

    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir, float wavelength = 0) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir, float wavelength = -1.0f) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    float A = 1.7280f;
    float B = 0.01342f;
    float actual_index_of_refraction = (wavelength > 0.0f) ? cauchy_eqn(A, B, wavelength) : index_of_refraction;

    BSDF_Sample ret;
    bool was_internal;
    ret.attenuation = transmittance; // What is the ratio of reflected/incoming light?
    ret.direction = refract(out_dir, actual_index_of_refraction, was_internal); // What direction should we sample incoming light from?
    if(was_internal) {
        ret.direction = reflect(out_dir);
    }
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
