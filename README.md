# FemaleBody

A customizable body model with bounce physics for Minecraft Bedrock on Android,
built as a native mod for [LeviLauncher](https://github.com/LiteLDev/LeviLaunchroid).
Client-side and visual only. It does not change gameplay and sends nothing to servers.

Based on the runtime of [BedrockTools](https://github.com/QYCottage/BedrockTools) (MIT).

## Settings

All settings appear in the LeviLauncher mod menu.

| Setting | Meaning |
|---|---|
| Show Self | Draw on your own character (third person only) |
| Show Others | Draw on other players too (off by default) |
| Size Scale | Overall size, 0.1 to 5 |
| Spacing | Distance between the two parts, hundredths of a block |
| Height Percent | Height on the body as a percent of the player's current height |
| Skin Color | Color of the shapes, `#AARRGGBB` or `#RRGGBB` |
| Physics | Turns the spring physics on or off |
| Physics Spring / Damping / Bounce / Sway Strength | Tune how it moves |

## Known limits

- Shapes are flat-colored boxes with simple shading, not skin-textured.
- Body direction is estimated from head direction, so it can drift while you turn your head.
- Signatures and offsets are version-specific. Targets `1.26.6X.X`.
- If nothing renders, set `kPrimitiveTriangles` to `1` in `src/modules/visual/femalebody.cpp`.

## Building

Requirements: Android NDK r28c, xmake, Python 3.

```sh
xmake f -y -p android -a arm64-v8a -m release --ndk=/path/to/android-ndk-r28c
xmake -y
```

This produces `libFemaleBody.so` and `FemaleBody.levipack`.
Pushing to `main` also builds it with GitHub Actions; download the
`FemaleBody-levipack` artifact from the Actions tab.

## Install

Import `FemaleBody.levipack` in LeviLauncher's mod manager and enable it.

Do not use this to violate Mojang or Microsoft's user agreements.
