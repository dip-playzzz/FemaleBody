#pragma once

#include <algorithm>
#include <cmath>

// Pure math, no game dependencies. Safe to unit test on a PC.
namespace femalebody {

struct Spring {
    float pos = 0.0f;
    float vel = 0.0f;
};

struct Spring3 {
    Spring right;   // sideways sway
    Spring up;      // vertical bounce
    Spring fwd;     // forward / backward sway
};

struct PhysicsParams {
    float stiffness = 170.0f;   // spring constant
    float damping   = 9.8f;     // velocity damping
    float bounce    = 1.0f;     // how strongly acceleration pushes the spring
    float sway      = 0.5f;     // multiplier for horizontal response
    float maxOffset = 0.07f;    // clamp, in blocks
};

// One spring, semi-implicit Euler, sub-stepped so low frame rates stay stable.
inline void stepSpring(Spring& s, float accel, float dt, const PhysicsParams& p) {
    dt = std::clamp(dt, 0.0f, 0.05f);
    constexpr float kMaxStep = 1.0f / 120.0f;
    int steps = std::max(1, static_cast<int>(std::ceil(dt / kMaxStep)));
    float h = dt / static_cast<float>(steps);
    for (int i = 0; i < steps; ++i) {
        float force = -p.stiffness * s.pos - p.damping * s.vel - p.bounce * accel;
        s.vel += force * h;
        s.pos += s.vel * h;
        if (s.pos > p.maxOffset)  { s.pos = p.maxOffset;  s.vel = 0.0f; }
        if (s.pos < -p.maxOffset) { s.pos = -p.maxOffset; s.vel = 0.0f; }
    }
}

// Acceleration in the body's local frame (blocks / s^2).
struct LocalAccel {
    float right = 0.0f;
    float up = 0.0f;
    float fwd = 0.0f;
};

inline void stepSpring3(Spring3& s, const LocalAccel& a, float dt, const PhysicsParams& p) {
    stepSpring(s.up, a.up, dt, p);
    stepSpring(s.right, a.right * p.sway, dt, p);
    stepSpring(s.fwd, a.fwd * p.sway, dt, p);
}

inline void resetSpring3(Spring3& s) { s = Spring3{}; }

// Turns a world-space acceleration into the body's local frame.
// yawDeg uses Bedrock's convention: forward = (-sin(yaw), cos(yaw)) in (x, z).
inline LocalAccel toLocal(float ax, float ay, float az, float yawDeg) {
    constexpr float kDegToRad = 3.14159265f / 180.0f;
    float yaw = yawDeg * kDegToRad;
    float fx = -std::sin(yaw), fz = std::cos(yaw);
    float rx = -fz, rz = fx;   // right-hand side when looking from above
    LocalAccel out;
    out.fwd = ax * fx + az * fz;
    out.right = ax * rx + az * rz;
    out.up = ay;
    return out;
}

// Moves bodyYaw toward headYaw the way vanilla does: it follows when moving,
// and is dragged along when the head turns more than maxDiff away.
inline float wrapDeg(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

inline float updateBodyYaw(float bodyYaw, float headYaw, bool moving, float dt, float maxDiff = 50.0f) {
    float diff = wrapDeg(headYaw - bodyYaw);
    if (moving) {
        bodyYaw += diff * std::min(1.0f, dt * 12.0f);
    } else if (std::fabs(diff) > maxDiff) {
        bodyYaw = headYaw - (diff > 0 ? maxDiff : -maxDiff);
    }
    return wrapDeg(bodyYaw);
}

}  // namespace femalebody
