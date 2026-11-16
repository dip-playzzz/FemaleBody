#include "femalebody.hpp"
#include "femalebody_physics.hpp"
#include "core/RenderState.hpp"
#include "core/memory/Hooks.hpp"
#include <bedrocktools/events/EventBus.hpp>
#include <bedrocktools/memory/Signatures.hpp>
#include <bedrocktools/sdk/Offsets.hpp>
#include <bedrocktools/sdk/Types.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace {

using bedrocktools::sdk::Vec3;
using namespace femalebody;

// ---- function types (same shapes the Hitbox module uses) -------------------
typedef void (*Tessellator_begin_t)(void*, void*, int, int, int);
typedef void (*Tessellator_color_t)(void*, float, float, float, float);
typedef void (*Tessellator_vertex_t)(void*, float, float, float);
typedef void (*MeshHelpers_renderMeshImmediately_t)(void*, void*, void*, char*);
typedef bool (*Actor_isPlayer_t)(void*);
typedef bool (*Actor_isInvisible_t)(void*);

struct DistanceSortedActor { void* mActor; float mDistance; float _pad; };
struct ActorVec { DistanceSortedActor* begin; DistanceSortedActor* end; DistanceSortedActor* cap; };
typedef ActorVec (*Actor_fetchNearbyActorsSorted_t)(void*, void*, int);

// If nothing shows up in game, try 1 (quad list) here. 2 = triangle list.
constexpr int kPrimitiveTriangles = 2;
constexpr int kMobActorType = 0x100;

Tessellator_begin_t s_tessBegin = nullptr;
Tessellator_color_t s_tessColor = nullptr;
Tessellator_vertex_t s_tessVertex = nullptr;
MeshHelpers_renderMeshImmediately_t s_renderMesh = nullptr;
Actor_isPlayer_t s_isPlayer = nullptr;
Actor_isInvisible_t s_isInvisible = nullptr;
Actor_fetchNearbyActorsSorted_t s_fetchNearby = nullptr;

void (*s_renderLevelOrig)(void*, void*, void*) = nullptr;
FemaleBodyModule* g_mod = nullptr;
void* g_localPlayer = nullptr;
std::uint32_t g_colorArgb = 0xFFE8B88A;

// ---- per-actor animation state ---------------------------------------------
struct ActorState {
    Vec3 prev{}, cur{}, renderPos{};
    double stamp = 0.0;
    double lastTime = 0.0;
    std::uint64_t lastFrame = 0;
    bool init = false;

    Vec3 smoothVel{};
    Vec3 lowAccel{};
    float bodyYaw = 0.0f;

    Spring3 left{}, right{};
};

std::unordered_map<void*, ActorState> s_states;
std::uint64_t s_frame = 0;

double nowSeconds() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

float lerpf(float a, float b, float t) { return a + (b - a) * t; }

bool readAABB(void* actor, bedrocktools::sdk::AABB& out) {
    if (!actor) return false;
    namespace off = bedrocktools::sdk::offsets;
    auto base = reinterpret_cast<std::uintptr_t>(actor);
    auto builtIn = *reinterpret_cast<std::uintptr_t*>(base + off::Actor::mStateVectorComponent);
    if (!builtIn) return false;
    auto comp = *reinterpret_cast<std::uintptr_t*>(base + off::Actor::mStateVectorComponent +
                                                  off::BuiltInActorComponents::mAABBShapeComponent);
    if (!comp) return false;
    auto v = *reinterpret_cast<bedrocktools::sdk::AABB*>(comp + off::AABBShapeComponent::mAABB);
    if (!std::isfinite(v.min.x) || !std::isfinite(v.min.y) || !std::isfinite(v.min.z) ||
        !std::isfinite(v.max.x) || !std::isfinite(v.max.y) || !std::isfinite(v.max.z)) return false;
    if (v.max.x <= v.min.x || v.max.y <= v.min.y || v.max.z <= v.min.z) return false;
    out = v;
    return true;
}

float readHeadYaw(void* actor) {
    namespace off = bedrocktools::sdk::offsets;
    auto base = reinterpret_cast<std::uintptr_t>(actor);
    auto comp = *reinterpret_cast<std::uintptr_t*>(base + off::Actor::mActorRotationComponent);
    if (!comp) return 0.0f;
    return reinterpret_cast<bedrocktools::sdk::Vec2*>(comp)->y;   // x = pitch, y = yaw
}

PhysicsParams makeParams(const FemaleBodyModule& m) {
    PhysicsParams p;
    p.stiffness = 40.0f + m.physicsSpringStrength * 260.0f;
    p.damping = 3.0f + m.physicsDampingStrength * 17.0f;
    p.bounce = m.physicsBounceStrength * 2.0f;
    p.sway = m.physicsSwayStrength * 2.0f;
    p.maxOffset = 0.07f;
    return p;
}

// Advances one actor's animation by one frame.
void updateState(ActorState& st, const Vec3& target, float headYaw, double now, const FemaleBodyModule& m) {
    float dt = static_cast<float>(now - st.lastTime);
    bool stale = !st.init || dt > 0.25f || dt <= 0.0f;

    if (stale) {
        st = ActorState{};
        st.init = true;
        st.prev = st.cur = st.renderPos = target;
        st.stamp = now;
        st.bodyYaw = headYaw;
        st.lastTime = now;
        st.lastFrame = s_frame;
        return;
    }

    // Positions only change on game ticks (20 Hz). Interpolate between them so the
    // boxes move as smoothly as the player model does.
    float dx = target.x - st.cur.x, dy = target.y - st.cur.y, dz = target.z - st.cur.z;
    float moved2 = dx * dx + dy * dy + dz * dz;
    if (moved2 > 1e-8f) {
        if (moved2 > 64.0f) {                 // teleport: snap
            st.prev = st.cur = st.renderPos = target;
        } else {
            st.prev = st.renderPos;
            st.cur = target;
        }
        st.stamp = now;
    }
    float alpha = std::clamp(static_cast<float>((now - st.stamp) / 0.05), 0.0f, 1.0f);
    Vec3 newPos{lerpf(st.prev.x, st.cur.x, alpha), lerpf(st.prev.y, st.cur.y, alpha),
                lerpf(st.prev.z, st.cur.z, alpha)};

    // Velocity -> acceleration, low-passed, with the slow component (gravity,
    // constant speed) removed so only real changes make things move.
    Vec3 rawVel{(newPos.x - st.renderPos.x) / dt, (newPos.y - st.renderPos.y) / dt,
                (newPos.z - st.renderPos.z) / dt};
    st.renderPos = newPos;

    float kVel = 1.0f - std::exp(-dt * 12.0f);
    Vec3 prevVel = st.smoothVel;
    st.smoothVel.x += (rawVel.x - st.smoothVel.x) * kVel;
    st.smoothVel.y += (rawVel.y - st.smoothVel.y) * kVel;
    st.smoothVel.z += (rawVel.z - st.smoothVel.z) * kVel;

    Vec3 accel{(st.smoothVel.x - prevVel.x) / dt, (st.smoothVel.y - prevVel.y) / dt,
               (st.smoothVel.z - prevVel.z) / dt};
    float kLow = 1.0f - std::exp(-dt * 4.0f);
    st.lowAccel.x += (accel.x - st.lowAccel.x) * kLow;
    st.lowAccel.y += (accel.y - st.lowAccel.y) * kLow;
    st.lowAccel.z += (accel.z - st.lowAccel.z) * kLow;

    float hSpeed = std::sqrt(st.smoothVel.x * st.smoothVel.x + st.smoothVel.z * st.smoothVel.z);
    st.bodyYaw = updateBodyYaw(st.bodyYaw, headYaw, hSpeed > 0.1f, dt);

    if (m.physics) {
        LocalAccel a = toLocal(accel.x - st.lowAccel.x, accel.y - st.lowAccel.y,
                               accel.z - st.lowAccel.z, st.bodyYaw);
        a.right = std::clamp(a.right, -60.0f, 60.0f);
        a.up = std::clamp(a.up, -60.0f, 60.0f);
        a.fwd = std::clamp(a.fwd, -60.0f, 60.0f);

        PhysicsParams p = makeParams(m);
        PhysicsParams pl = p, pr = p;
        pl.stiffness *= 0.97f;      // tiny difference so the two don't move in lockstep
        pr.stiffness *= 1.03f;
        stepSpring3(st.left, a, dt, pl);
        stepSpring3(st.right, a, dt, pr);
    } else {
        resetSpring3(st.left);
        resetSpring3(st.right);
    }

    st.lastTime = now;
    st.lastFrame = s_frame;
}

// ---- geometry ----------------------------------------------------------------
struct Box {
    Vec3 center;       // camera-relative
    Vec3 r, u, f;      // unit axes
    float hw, hh, hd;  // half extents
};

constexpr int kFaces[6][4] = {{1, 3, 7, 5}, {0, 4, 6, 2}, {2, 6, 7, 3},
                              {0, 1, 5, 4}, {4, 5, 7, 6}, {0, 2, 3, 1}};
constexpr float kShade[6] = {0.85f, 0.85f, 1.00f, 0.60f, 0.90f, 0.75f};

void emitBox(void* tess, const Box& b, float cr, float cg, float cb, float ca) {
    Vec3 c[8];
    for (int i = 0; i < 8; ++i) {
        float sx = (i & 1) ? b.hw : -b.hw;
        float sy = (i & 2) ? b.hh : -b.hh;
        float sz = (i & 4) ? b.hd : -b.hd;
        c[i] = {b.center.x + b.r.x * sx + b.u.x * sy + b.f.x * sz,
                b.center.y + b.r.y * sx + b.u.y * sy + b.f.y * sz,
                b.center.z + b.r.z * sx + b.u.z * sy + b.f.z * sz};
    }
    for (int f = 0; f < 6; ++f) {
        s_tessColor(tess, cr * kShade[f], cg * kShade[f], cb * kShade[f], ca);
        const int* q = kFaces[f];
        // Both windings, so it renders whether or not the material culls back faces.
        const int tri[12] = {0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2};
        for (int t = 0; t < 12; ++t) {
            const Vec3& v = c[q[tri[t]]];
            s_tessVertex(tess, v.x, v.y, v.z);
        }
    }
}

void collectBoxes(void* actor, const FemaleBodyModule& m, double now, const Vec3& cam,
                  std::vector<Box>& out) {
    bedrocktools::sdk::AABB aabb;
    if (!readAABB(actor, aabb)) return;

    Vec3 target{(aabb.min.x + aabb.max.x) * 0.5f, aabb.min.y, (aabb.min.z + aabb.max.z) * 0.5f};
    ActorState& st = s_states[actor];
    updateState(st, target, readHeadYaw(actor), now, m);

    constexpr float kDeg = 3.14159265f / 180.0f;
    float yaw = st.bodyYaw * kDeg;
    Vec3 f{-std::sin(yaw), 0.0f, std::cos(yaw)};
    Vec3 r{-f.z, 0.0f, f.x};
    Vec3 u{0.0f, 1.0f, 0.0f};

    float height = aabb.max.y - aabb.min.y;
    float chestY = st.renderPos.y + height * (static_cast<float>(m.heightPercent) / 100.0f);

    float s = m.sizeScale;
    float hw = 0.07f * s, hh = 0.06f * s, hd = 0.05f * s;
    float fwdBase = 0.125f + hd - 0.02f;          // slightly embedded in the torso
    float half = static_cast<float>(m.spacing) / 200.0f;

    for (int side = 0; side < 2; ++side) {
        const Spring3& sp = side == 0 ? st.left : st.right;
        float offR = (side == 0 ? -half : half) + sp.right.pos;
        float offF = fwdBase + sp.fwd.pos;
        float offU = sp.up.pos;
        Box b;
        b.center = {st.renderPos.x + r.x * offR + f.x * offF - cam.x,
                    chestY + offU - cam.y,
                    st.renderPos.z + r.z * offR + f.z * offF - cam.z};
        b.r = r; b.u = u; b.f = f;
        b.hw = hw; b.hh = hh; b.hd = hd;
        out.push_back(b);
    }
}

void renderLevelHook(void* self, void* screenContext, void* a3) {
    if (s_renderLevelOrig) s_renderLevelOrig(self, screenContext, a3);

    if (!g_mod || !g_mod->enabled || !g_localPlayer) return;
    if (!s_tessBegin || !s_tessColor || !s_tessVertex || !s_renderMesh) return;
    if (!screenContext || reinterpret_cast<std::uintptr_t>(screenContext) < 0x1000) return;

    namespace off = bedrocktools::sdk::offsets;
    auto tessPtr = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(screenContext) +
                                                      off::ScreenContext::mTessellator);
    if (tessPtr < 0x1000) return;
    void* tess = reinterpret_cast<void*>(tessPtr);

    auto lrp = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(self) +
                                                  off::LevelRenderer::mLevelRendererPlayer);
    if (lrp < 0x1000) return;
    Vec3 cam = *reinterpret_cast<Vec3*>(lrp + off::LevelRendererPlayer::mCamPos);

    ++s_frame;
    double now = nowSeconds();
    std::vector<Box> boxes;

    // Local player: only in third person, otherwise the camera sits inside the model.
    if (g_mod->showSelf) {
        bedrocktools::sdk::AABB ab;
        if (readAABB(g_localPlayer, ab)) {
            float ex = cam.x - (ab.min.x + ab.max.x) * 0.5f;
            float ey = cam.y - (ab.min.y + (ab.max.y - ab.min.y) * 0.9f);
            float ez = cam.z - (ab.min.z + ab.max.z) * 0.5f;
            if (ex * ex + ey * ey + ez * ez > 0.25f) collectBoxes(g_localPlayer, *g_mod, now, cam, boxes);
        }
    }

    if (g_mod->showOthers && s_fetchNearby && s_isPlayer) {
        Vec3 extent{30.0f, 30.0f, 30.0f};
        ActorVec actors = s_fetchNearby(g_localPlayer, &extent, kMobActorType);
        if (actors.begin && actors.end) {
            for (auto* it = actors.begin; it < actors.end; ++it) {
                void* ent = it->mActor;
                if (!ent || ent == g_localPlayer || !s_isPlayer(ent)) continue;
                if (s_isInvisible && s_isInvisible(ent)) continue;
                collectBoxes(ent, *g_mod, now, cam, boxes);
            }
        }
    }

    if ((s_frame % 300) == 0) {
        for (auto it = s_states.begin(); it != s_states.end();) {
            it = (s_frame - it->second.lastFrame > 120) ? s_states.erase(it) : std::next(it);
        }
    }

    if (boxes.empty()) return;

    bedrocktools::core::ScopedShaderColor shaderColor(screenContext);
    if (!shaderColor) return;

    float ca = ((g_colorArgb >> 24) & 0xFF) / 255.0f;
    float cr = ((g_colorArgb >> 16) & 0xFF) / 255.0f;
    float cg = ((g_colorArgb >> 8) & 0xFF) / 255.0f;
    float cb = (g_colorArgb & 0xFF) / 255.0f;

    void* material = reinterpret_cast<void*>(lrp + off::LevelRendererPlayer::mSelectionOverlayMaterial);

    int vertexCount = static_cast<int>(boxes.size()) * 6 * 12;
    s_tessBegin(tess, nullptr, kPrimitiveTriangles, vertexCount, 0);
    for (const Box& b : boxes) emitBox(tess, b, cr, cg, cb, ca);

    char pad[0x58];
    std::memset(pad, 0, sizeof(pad));
    s_renderMesh(screenContext, tess, material, pad);
}

std::uint32_t parseColor(const std::string& s, std::uint32_t fallback) {
    if (s.size() < 2 || s[0] != '#') return fallback;
    try {
        auto v = static_cast<std::uint32_t>(std::stoul(s.substr(1), nullptr, 16));
        if (s.size() <= 7) v |= 0xFF000000u;   // "#RRGGBB" -> opaque
        return v;
    } catch (...) {
        return fallback;
    }
}

}  // namespace

// ============================================================================

FemaleBodyModule::FemaleBodyModule()
    : Module("Female Body", "Customizable body model with bounce physics (client-side, visual only).") {
    g_mod = this;
}

FemaleBodyModule::~FemaleBodyModule() {
    if (g_mod == this) g_mod = nullptr;
}

void FemaleBodyModule::onInit() {
    using bedrocktools::memory::SignatureId;
    using bedrocktools::memory::resolve;

    m_target = reinterpret_cast<void*>(resolve(SignatureId::RenderLevel));
    s_tessBegin = reinterpret_cast<Tessellator_begin_t>(resolve(SignatureId::TessellatorBegin));
    s_tessColor = reinterpret_cast<Tessellator_color_t>(resolve(SignatureId::TessellatorColor));
    s_tessVertex = reinterpret_cast<Tessellator_vertex_t>(resolve(SignatureId::TessellatorVertex));
    s_renderMesh = reinterpret_cast<MeshHelpers_renderMeshImmediately_t>(
        resolve(SignatureId::MeshHelpersRenderMeshImmediately2));
    s_isPlayer = reinterpret_cast<Actor_isPlayer_t>(resolve(SignatureId::ActorIsPlayer));
    s_isInvisible = reinterpret_cast<Actor_isInvisible_t>(resolve(SignatureId::ActorIsInvisible));
    s_fetchNearby = reinterpret_cast<Actor_fetchNearbyActorsSorted_t>(
        resolve(SignatureId::ActorFetchNearbyActorsSorted));

    bedrocktools::events::bus().subscribe<bedrocktools::events::LocalPlayerTickEvent>(
        [](auto& event) { g_localPlayer = event.player; });
}

void FemaleBodyModule::onEnable() {
    if (m_hooked || !m_target) return;
    bedrocktools::hooks::install(m_target, reinterpret_cast<void*>(renderLevelHook),
                                 reinterpret_cast<void**>(&s_renderLevelOrig));
    m_hooked = true;
}

void FemaleBodyModule::onDisable() {
    s_states.clear();   // physics restarts cleanly next time
}

void FemaleBodyModule::loadConfig(const nlohmann::json& j) {
    Module::loadConfig(j);
    auto get = [&j](const char* key, auto& field) {
        try {
            if (j.contains(key)) field = j[key].get<std::decay_t<decltype(field)>>();
        } catch (...) {}
    };
    get("showSelf", showSelf);
    get("showOthers", showOthers);
    get("sizeScale", sizeScale);
    get("spacing", spacing);
    get("heightPercent", heightPercent);
    get("skinColor", skinColor);
    get("physics", physics);
    get("physicsSpringStrength", physicsSpringStrength);
    get("physicsDampingStrength", physicsDampingStrength);
    get("physicsBounceStrength", physicsBounceStrength);
    get("physicsSwayStrength", physicsSwayStrength);

    sizeScale = std::clamp(sizeScale, 0.1f, 5.0f);
    spacing = std::clamp(spacing, 0, 200);
    heightPercent = std::clamp(heightPercent, 0, 200);
    g_colorArgb = parseColor(skinColor, g_colorArgb);
}

void FemaleBodyModule::saveConfig(nlohmann::json& j) {
    Module::saveConfig(j);
    j["showSelf"] = showSelf;
    j["showOthers"] = showOthers;
    j["sizeScale"] = sizeScale;
    j["spacing"] = spacing;
    j["heightPercent"] = heightPercent;
    j["skinColor"] = skinColor;
    j["physics"] = physics;
    j["physicsSpringStrength"] = physicsSpringStrength;
    j["physicsDampingStrength"] = physicsDampingStrength;
    j["physicsBounceStrength"] = physicsBounceStrength;
    j["physicsSwayStrength"] = physicsSwayStrength;
}
