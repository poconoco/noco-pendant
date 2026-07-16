/*****************************************************************************
* | File      	:   Fluid.h
* | Function    :   2D FLIP/PIC fluid solver sized for an 8x8 LED matrix
* | Info        :   Ported from Matthias Muller's "18-flip.html"
*                    (tenMinutePhysics, MIT licensed)
******************************************************************************/
#ifndef FLUID_H
#define FLUID_H

#include <array>
#include <cstdint>

// 2D FLIP/PIC fluid solver sized for an 8x8 LED matrix.
class Fluid {
public:
    // Resets the grid and fills a block of particles near the bottom of the tank.
    Fluid();

    // Advances the simulation by dt seconds under gravity vector
    // (gravityX, gravityY), expressed in the same units as the grid
    // (kCellSize per cell).
    void step(float dt, float gravityX, float gravityY);

    // Local water density at LED cell (x, y), x/y in [0, 7].
    float cellDensity(int x, int y) const;

    // Reference "full" density once the fluid has settled; 0 until then.
    float restDensity() const { return particleRestDensity_; }

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

    void integrateParticles(float dt, float gravityX, float gravityY);
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
