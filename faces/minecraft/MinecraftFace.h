/*****************************************************************************
* | File      	:   MinecraftFace.h
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation that cycles through hardcoded 8x8
*                    Minecraft block/mob icons, each fading in, holding, and
*                    fading out in turn.
******************************************************************************/
#ifndef MINECRAFTFACE_H
#define MINECRAFTFACE_H

#include "Face.h"

// Cycles through a fixed set of hardcoded 8x8 pixel-art blocks, showing one
// at a time: fades in, holds solid, fades out, then the next block fades
// in. Purely time-driven -- feedImu is a no-op, since the sequence doesn't
// react to tilt at all.
class MinecraftFace : public Face {
public:
    MinecraftFace() = default;

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    float elapsedS_ = 0.0f;
    int blockIndex_ = 0;
};

#endif // MINECRAFTFACE_H
