#pragma once

#include "../Module.hpp"
#include <cstdint>
#include <string>

// Client-side cosmetic body model with spring physics, in the spirit of the
// Java "female gender" mod. Every setting below shows up as a slider/toggle in
// the LeviLauncher mod menu automatically (the menu is built from saveConfig).
class FemaleBodyModule : public Module {
public:
    FemaleBodyModule();
    ~FemaleBodyModule() override;

    void onInit() override;
    void onEnable() override;
    void onDisable() override;
    void loadConfig(const nlohmann::json& j) override;
    void saveConfig(nlohmann::json& j) override;

    // Who gets drawn
    bool showSelf = true;
    bool showOthers = false;   // off by default: other players never opted in

    // Shape
    float sizeScale = 1.0f;    // overall size multiplier
    int spacing = 15;          // centre-to-centre distance, hundredths of a block
    int heightPercent = 66;    // chest height as % of the player's current height
    std::string skinColor = "#FFE8B88A";   // #AARRGGBB

    // Physics
    bool physics = true;
    float physicsSpringStrength = 0.5f;
    float physicsDampingStrength = 0.4f;
    float physicsBounceStrength = 0.5f;
    float physicsSwayStrength = 0.5f;

private:
    bool m_hooked = false;
    void* m_target = nullptr;
};
