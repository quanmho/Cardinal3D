
#pragma once

#include <algorithm>
#include <cmath>

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

#include "line.h"
#include "plane.h"
#include "vec2.h"
#include "vec3.h"
#include "vec4.h"
#include "spectrum.h"

#define EPS_F 0.00001f
#define PI_F 3.14159265358979323846264338327950288f
#define Radians(v) ((v) * (PI_F / 180.0f))
#define Degrees(v) ((v) * (180.0f / PI_F))

template<typename T> inline T clamp(T x, T min, T max) {
    return std::min(std::max(x, min), max);
}
template<> inline Vec2 clamp(Vec2 v, Vec2 min, Vec2 max) {
    return Vec2(clamp(v.x, min.x, max.x), clamp(v.y, min.y, max.y));
}
template<> inline Vec3 clamp(Vec3 v, Vec3 min, Vec3 max) {
    return Vec3(clamp(v.x, min.x, max.x), clamp(v.y, min.y, max.y), clamp(v.z, min.z, max.z));
}
template<> inline Vec4 clamp(Vec4 v, Vec4 min, Vec4 max) {
    return Vec4(clamp(v.x, min.x, max.x), clamp(v.y, min.y, max.y), clamp(v.z, min.z, max.z),
                clamp(v.w, min.w, max.w));
}

template<typename T> T lerp(T start, T end, float t) {
    return start + (end - start) * t;
}

inline float sign(float x) {
    return x > 0.0f ? 1.0f : x < 0.0f ? -1.0f : 0.0f;
}

inline float frac(float x) {
    return x - (long long)x;
}

inline float smoothstep(float e0, float e1, float x) {
    float t = clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

inline float cauchy_eqn(float A, float B, float wavelength_nm) {
    float wavelength_um = wavelength_nm * 0.001f;
    return A + (B / pow(wavelength_um, 2.0f));
}

inline Vec3 xyzfit_1931(float wavelength_nm) {
    float t1_x = (wavelength_nm - 442.0f) * ((wavelength_nm < 442.0f) ? 0.0624f : 0.0374f);
    float t2_x = (wavelength_nm - 599.8f) * ((wavelength_nm < 599.8f) ? 0.0264f : 0.0323f);
    float t3_x = (wavelength_nm - 501.1f) * ((wavelength_nm < 501.1f) ? 0.0490f : 0.0382f);
    float x = 0.362f * expf(-0.5f * t1_x * t1_x) + 1.056f * expf(-0.5f * t2_x * t2_x) -
              0.065f * expf(-0.5f * t3_x * t3_x);

    float t1_y = (wavelength_nm - 568.8f) * ((wavelength_nm < 568.8f) ? 0.0213f : 0.0247f);
    float t2_y = (wavelength_nm - 530.9f) * ((wavelength_nm < 530.9f) ? 0.0613f : 0.0322f);
    float y = 0.821f * exp(-0.5f * t1_y * t1_y) + 0.286f * expf(-0.5f * t2_y * t2_y);

    float t1_z = (wavelength_nm - 437.0f) * ((wavelength_nm < 437.0f) ? 0.0845f : 0.0278f);
    float t2_z = (wavelength_nm - 459.0f) * ((wavelength_nm < 459.0f) ? 0.0385f : 0.0725f);
    float z = 1.217f * exp(-0.5f * t1_z * t1_z) + 0.681f * expf(-0.5f * t2_z * t2_z);

    return Vec3(x, y, z);
}

inline Spectrum xyz2srgb(Vec3 xyz) {
    float r = 3.2406255f * xyz.x + -1.537208f * xyz.y + -0.4986286f * xyz.z;
    float g = -0.9689307f * xyz.x + 1.8757561f * xyz.y + 0.0415175f * xyz.z;
    float b = 0.0557101f * xyz.x + -0.2040211f * xyz.y + 1.0569959f * xyz.z;
    r = clamp(r, 0.0f, 1.0f);
    g = clamp(g, 0.0f, 1.0f);
    b = clamp(b, 0.0f, 1.0f);
    return Spectrum(r, g, b);
}

inline Vec3 srgb2xyz(Vec3 rgb) {
    float x = 0.4124564f * rgb.x + 0.3575761f * rgb.y + 0.1804375f * rgb.z;
    float y = 0.2126729f * rgb.x + 0.7151522f * rgb.y + 0.0721750f * rgb.z;
    float z = 0.0193339f * rgb.x + 0.1191920f * rgb.y + 0.9503041f * rgb.z;
    x = clamp(x, 0.0f, 1.0f);
    y = clamp(y, 0.0f, 1.0f);
    z = clamp(z, 0.0f, 1.0f);
    return Vec3(x, y, z);
}

#include "bbox.h"
#include "mat4.h"
#include "quat.h"
#include "ray.h"
