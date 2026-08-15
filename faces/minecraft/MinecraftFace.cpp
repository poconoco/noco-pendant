/*****************************************************************************
* | File      	:   MinecraftFace.cpp
* | Author      :   Leonid, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation that cycles through hardcoded 8x8
*                   Minecraft block/mob icons, each fading in, holding, and
*                   fading out in turn.
******************************************************************************/
#include "MinecraftFace.h"

#include <array>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

namespace {

// How long each block stays on screen, in seconds. kFadeInS is the leading
// portion of kHoldS (not additional time) during which the block ramps up
// from black to full brightness; kFadeOutS is extra time tacked on after
// kHoldS during which it ramps back down to black before the next block's
// fade-in begins.
constexpr float kFadeInS = 0.5f;
constexpr float kHoldS = 3.0f;
constexpr float kFadeOutS = 0.5f;
constexpr float kCycleS = kHoldS + kFadeOutS;

constexpr float kColorDamping = 0.4f;

// Shared palette: every block's pixel grid below is a string of these
// letters, one per pixel, so blocks can be written and read as little
// pictures. Colors are weights in [0, 1] per channel, scaled by
// LED_BRIGHTNESS and this Face's fade multiplier at render time.
struct Palette {
    char key;
    Color color;
};

// Size is deduced via CTAD (see kBlocks below for the same trick) so removing
// or adding a color here never needs a count updated elsewhere. Only colors
// actually referenced by a kBlocks grid belong here -- paletteColor() below
// already falls back to black for any key it doesn't find.
constexpr std::array kPalette = {
    Palette{'G', {0.05f, 0.90f, 0.05f}}, // grass top -- vivid green
    Palette{'g', {0.05f, 0.80f, 0.05f}}, // grass/dirt transition
    Palette{'D', {0.25f, 0.18f, 0.05f}}, // dirt
    Palette{'d', {0.20f, 0.15f, 0.03f}}, // dirt speck
    Palette{'S', {0.00f, 0.00f, 0.00f}}, // stone -- left off; the physical grey mask over the LEDs already reads as grey when unlit
    Palette{'Y', {1.00f, 0.65f, 0.00f}}, // gold -- a bit redder than pure yellow
    Palette{'R', {1.00f, 0.00f, 0.00f}}, // TNT red -- vivid red
    Palette{'W', {1.00f, 1.00f, 0.95f}}, // TNT white stripe
    Palette{'K', {0.00f, 0.00f, 0.00f}}, // true black (features/outlines)
    Palette{'C', {0.05f, 0.85f, 0.05f}}, // creeper green -- vivid
    Palette{'e', {0.50f, 0.00f, 0.80f}}, // enderman eye purple -- vivid
    Palette{'E', {0.60f, 0.00f, 1.00f}}, // enderman eye purple -- vivid
    Palette{'I', {0.00f, 0.90f, 0.90f}}, // diamond cyan -- vivid
    Palette{'N', {0.90f, 0.85f, 0.70f}}, // birch bark
};

Color paletteColor(char c) {
    for (const auto &entry : kPalette) {
        if (entry.key == c) return entry.color;
    }
    return {0.0f, 0.0f, 0.0f};
}

using BlockGrid = std::array<const char *, FACE_HEIGHT>;

constexpr std::array kBlocks = {
    // Creeper face
    BlockGrid{{
        "CCCCCCCC",
        "CKKCCKKC",
        "CKKCCKKC",
        "CCCKKCCC",
        "CCKKKKCC",
        "CCKKKKCC",
        "CCKCCKCC",
        "CCCCCCCC",
    }},
    // Dirt with grass
    BlockGrid{{
        "GGGGGGGG",
        "gGggGGgD",
        "gDDgDDGD",
        "DdDDdDDD",
        "DDDDdDDD",
        "DdDDDDdD",
        "DDDdDDDD",
        "DDDDDDDD",
    }},
    // Gold ore
    BlockGrid{{
        "SSSSSSSS",
        "SYYSSSYS",
        "SSYSSYYS",
        "SSSSSYSS",
        "SSYSSSSS",
        "SSYYSSSS",
        "SSSSSYSS",
        "SYSSSSSS",
    }},
    // TNT
    BlockGrid{{
        "RRRRRRRR",
        "RRRRRRRR",
        "WWWWWWWW",
        "KKKKKKKK",
        "WKWKKWKW",
        "WWWWWWWW",
        "RRRRRRRR",
        "RRRRRRRR",
    }},
    // Enderman face
    BlockGrid{{
        "KKKKKKKK",
        "KKKKKKKK",
        "KKKKKKKK",
        "KKKKKKKK",
        "eEeKKeEe",
        "KKKKKKKK",
        "KKKKKKKK",
        "KKKKKKKK",
    }},
    // Diamond ore -- same speckled pattern as gold ore, cyan instead of yellow
    BlockGrid{{
        "SSSSSSSS",
        "SIISSSIS",
        "SSISSIIS",
        "SSSSSISS",
        "SSISSSSS",
        "SSIISSSS",
        "SSSSSISS",
        "SISSSSSS",
    }},
};

} // namespace

void MinecraftFace::feedImu(const ImuSample & /*sample*/) {
    // Block sequence is time-driven only; tilt is intentionally ignored.
}

FaceFrame MinecraftFace::getFrame(uint32_t dtUs) {
    elapsedS_ += dtUs / 1000000.0f;
    while (elapsedS_ >= kCycleS) {
        elapsedS_ -= kCycleS;
        blockIndex_ = (blockIndex_ + 1) % static_cast<int>(kBlocks.size());
    }

    float fade;
    if (elapsedS_ < kFadeInS) {
        fade = elapsedS_ / kFadeInS;
    } else if (elapsedS_ < kHoldS) {
        fade = 1.0f;
    } else {
        fade = 1.0f - (elapsedS_ - kHoldS) / kFadeOutS;
    }

    const BlockGrid &grid = kBlocks[blockIndex_];
    FaceFrame frame{};
    // Block art above is authored in the default "USB port up" orientation
    // (see Face.h's coordinate comment), but the board is worn rotated 180
    // degrees from that as a pendant, so each pixel is sampled pre-rotated
    // 180 degrees to compensate and appear upright to the wearer.
    for (int y = 0; y < FACE_HEIGHT; y++) {
        for (int x = 0; x < FACE_WIDTH; x++) {
            char c = grid[FACE_HEIGHT - 1 - y][FACE_WIDTH - 1 - x];
            Color color = paletteColor(c);
            frame.pixels[x][y] = Color{
                LED_BRIGHTNESS * color.r * fade * kColorDamping,
                LED_BRIGHTNESS * color.g * fade * kColorDamping,
                LED_BRIGHTNESS * color.b * fade * kColorDamping,
            };
        }
    }
    return frame;
}
