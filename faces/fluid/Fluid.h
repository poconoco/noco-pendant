/*****************************************************************************
* | File      	:   Fluid.h
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   2D FLIP/PIC fluid solver sized for an 8x8 LED matrix
* | Info        :   Ported from Matthias Muller's "18-flip.html"
*                    (tenMinutePhysics, MIT licensed)
******************************************************************************/
#ifndef FLUID_H
#define FLUID_H

#include <array>
#include <cstdint>
#include <span>

// A point that pulls nearby particles towards it at roughly constant
// strength (softened near zero distance to avoid a singularity), rather
// than a uniform gravity field. Positions are in the same grid-cell units
// as everything else here (see gridWidth()/gridHeight()).
//
// When several Attractors are passed to Fluid::step() together, each
// particle is pulled towards whichever one is nearest to it, not the sum of
// all of them -- summing would make particles settle near the group's
// centroid instead of tracing the shape the attractors are arranged along
// (e.g. a curve or arc made of several points).
struct Attractor {
    float x, y, strength;
};

// 2D FLIP/PIC fluid solver sized for an 8x8 LED matrix.
class Fluid {
public:
    // Resets the grid and fills a block of particles near the bottom of the tank.
    Fluid();

    // Default FLIP/PIC blend ratio for step()'s flipRatio parameter below --
    // higher values (closer to 1) are pure FLIP (thinner, more energetic
    // splashing); lower values blend in more PIC (thicker/more viscous,
    // since PIC re-derives velocity from the smoothed grid every step
    // instead of mostly carrying the particle's own velocity forward).
    // Exposed so callers can temporarily pass a different value (e.g.
    // FluidFace's calm mode wants a more viscous feel) and restore this
    // default afterwards.
    static constexpr float kFlipRatio = 0.98f;

    // Advances the simulation by dt seconds under gravity vector
    // (gravityX, gravityY), expressed in the same units as the grid
    // (kCellSize per cell), plus an optional set of attractor points (see
    // Attractor above) applied on top of that uniform gravity -- pass an
    // empty span (the default) for plain uniform-gravity behavior.
    //
    // dampingPerSecond exponentially decays particle velocity (0 = no
    // damping, i.e. plain FLIP/PIC behavior). An attractor is a central
    // force, so a particle with any velocity component perpendicular to it
    // orbits rather than falling in -- exactly like a planet around a star
    // -- and stays orbiting indefinitely without something to bleed off
    // that momentum. Damping is that something; it's rarely wanted for
    // plain gravity (real water sloshing needs its momentum), so it
    // defaults to off.
    //
    // flipRatio overrides kFlipRatio above for just this call.
    void step(float dt, float gravityX, float gravityY, std::span<const Attractor> attractors = {}, float dampingPerSecond = 0.0f, float flipRatio = kFlipRatio);

    // Local water density at LED cell (x, y), x/y in [0, 7].
    float cellDensity(int x, int y) const;

    // Reference "full" density once the fluid has settled; 0 until then.
    float restDensity() const { return particleRestDensity_; }

    // Grid size in the same continuous units as particle positions and
    // Attractor coordinates, so callers can place attractors (e.g. at the
    // grid's center) without hardcoding the layout.
    static constexpr int gridWidth() { return kNumX; }
    static constexpr int gridHeight() { return kNumY; }

private:
    // Simulation grid: 8x8 usable cells surrounded by a 1-cell solid wall,
    // so grid indices 1..8 map 1:1 onto the 8x8 LED matrix.
    static constexpr int kNumX = 10;
    static constexpr int kNumY = 10;
    static constexpr int kNumCells = kNumX * kNumY;
    static constexpr float kCellSize = 1.0f;

    static constexpr float kParticleRadius = 0.25f;
    static constexpr int kMaxParticles = 150;

    // Spatial hash grid used for particle/particle separation, sized
    // generously for kParticleRadius above (see the constructor for the
    // real dimensions).
    static constexpr int kPGridDim = 25;
    static constexpr int kPGridCells = kPGridDim * kPGridDim;

    enum class CellType : uint8_t { Fluid, Air, Solid };

    static constexpr int idx(int i, int j) { return i * kNumY + j; }

    void integrateParticles(float dt, float gravityX, float gravityY, std::span<const Attractor> attractors, float dampingPerSecond);
    void pushParticlesApart(int numIters);
    void handleParticleCollisions();
    void updateParticleDensity();
    void transferVelocities(bool toGrid, float flipRatio);
    void solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift);

    // MAC grid: u lives on vertical cell faces, v on horizontal cell faces.
    std::array<float, kNumCells> u_{};
    std::array<float, kNumCells> v_{};
    std::array<float, kNumCells> du_{};
    std::array<float, kNumCells> dv_{};
    std::array<float, kNumCells> prevU_{};
    std::array<float, kNumCells> prevV_{};
    std::array<float, kNumCells> p_{};
    std::array<float, kNumCells> s_{}; // 0 = solid, 1 = open
    std::array<CellType, kNumCells> cellType_{};
    std::array<float, kNumCells> particleDensity_{};
    float particleRestDensity_ = 0.0f;
    float density_ = 1000.0f;

    // Particles (structure-of-arrays).
    std::array<float, kMaxParticles> particlePosX_{};
    std::array<float, kMaxParticles> particlePosY_{};
    std::array<float, kMaxParticles> particleVelX_{};
    std::array<float, kMaxParticles> particleVelY_{};
    int numParticles_ = 0;
    float particleRadius_ = kParticleRadius;

    // Spatial hash for pushParticlesApart.
    float pInvSpacing_ = 0.0f;
    int pNumX_ = 0, pNumY_ = 0, pNumCells_ = 0;
    std::array<int, kPGridCells> numCellParticles_{};
    std::array<int, kPGridCells + 1> firstCellParticle_{};
    std::array<int, kMaxParticles> cellParticleIds_{};
};

#endif // FLUID_H
