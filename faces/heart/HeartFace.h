/*****************************************************************************
* | File      	:   HeartFace.h
* | Author      :   Leonid, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation rendering a small rigid-body physics
*                    simulation of a heart, falling under tilt-driven gravity
*                    and bouncing off the walls.
******************************************************************************/
#ifndef HEARTFACE_H
#define HEARTFACE_H

#include "Face.h"

#include <array>

// A heart-shaped polygon (~6x6 pixels once mapped to the grid) simulated as
// a 2D rigid body in float world coordinates (only the final render step
// touches actual pixels). It falls under gravity steered by tilt -- same
// accelerometer axes/sign convention and same unflipped rendering
// orientation as FluidFace's water, per request -- and bounces off the 8x8
// playfield's walls, losing a bit of energy per bounce and picking up
// realistic spin from off-center impacts. Mass, center of mass, and
// rotational inertia are all computed from the polygon itself (assuming
// uniform density), not hardcoded. Rendered antialiased via supersampling
// with a gamma-shaped brightness curve (see shapeCoverage in the .cpp,
// since LEDs read perceptually nonlinear -- same idea as FlappyFace's
// coverage shaping), red on a fully black background.
class HeartFace : public Face {
public:
    HeartFace();

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    struct Vec2 {
        float x, y;
    };

    static constexpr int kNumVertices = 16;

    void updateWorldVerts();
    void resolveWallCollision(const Vec2 &normal, const Vec2 &wallPoint);
    static bool pointInPolygon(float px, float py, const std::array<Vec2, kNumVertices> &verts);

    // Body-frame vertices, relative to the polygon's own center of mass
    // (computed once at construction).
    std::array<Vec2, kNumVertices> localVerts_{};
    float mass_ = 1.0f;
    float momentOfInertia_ = 1.0f;

    // World-space rigid body state; pos_ is the center of mass.
    Vec2 pos_{0.0f, 0.0f};
    Vec2 vel_{0.0f, 0.0f};
    float theta_ = 0.0f;
    float omega_ = 0.0f;

    Vec2 gravity_{0.0f, 0.0f};

    // Refreshed by updateWorldVerts() from the current pos_/theta_ --
    // recomputed before each wall's collision check and again before
    // rendering, so it's always consistent with whatever positional
    // correction the previous wall check may have applied.
    std::array<Vec2, kNumVertices> worldVerts_{};
};

#endif // HEARTFACE_H
