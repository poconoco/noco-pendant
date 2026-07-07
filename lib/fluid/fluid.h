/*****************************************************************************
* | File      	:   fluid.h
* | Function    :   2D FLIP/PIC fluid solver sized for an 8x8 LED matrix
* | Info        :   Ported from Matthias Muller's "18-flip.html"
*                    (tenMinutePhysics, MIT licensed)
******************************************************************************/
#ifndef FLUID_H
#define FLUID_H

#include <stdint.h>

// Simulation grid: 8x8 usable cells surrounded by a 1-cell solid wall,
// so grid indices 1..8 map 1:1 onto the 8x8 LED matrix.
#define FLUID_NUM_X 10
#define FLUID_NUM_Y 10
#define FLUID_NUM_CELLS (FLUID_NUM_X * FLUID_NUM_Y)
#define FLUID_H 1.0f

#define FLUID_PARTICLE_RADIUS 0.25f
#define FLUID_MAX_PARTICLES 150

// Spatial hash grid used for particle/particle separation, sized generously
// for FLUID_PARTICLE_RADIUS above (see Fluid_init for the real dimensions).
#define FLUID_PGRID_DIM 25
#define FLUID_PGRID_CELLS (FLUID_PGRID_DIM * FLUID_PGRID_DIM)

typedef enum {
    FLUID_CELL = 0,
    AIR_CELL = 1,
    SOLID_CELL = 2,
} FluidCellType;

typedef struct {
    // MAC grid: u lives on vertical cell faces, v on horizontal cell faces.
    float u[FLUID_NUM_CELLS];
    float v[FLUID_NUM_CELLS];
    float du[FLUID_NUM_CELLS];
    float dv[FLUID_NUM_CELLS];
    float prevU[FLUID_NUM_CELLS];
    float prevV[FLUID_NUM_CELLS];
    float p[FLUID_NUM_CELLS];
    float s[FLUID_NUM_CELLS]; // 0 = solid, 1 = open
    uint8_t cellType[FLUID_NUM_CELLS];
    float particleDensity[FLUID_NUM_CELLS];
    float particleRestDensity;
    float density;

    // Particles (structure-of-arrays).
    float particlePosX[FLUID_MAX_PARTICLES];
    float particlePosY[FLUID_MAX_PARTICLES];
    float particleVelX[FLUID_MAX_PARTICLES];
    float particleVelY[FLUID_MAX_PARTICLES];
    int numParticles;
    float particleRadius;

    // Spatial hash for pushParticlesApart.
    float pInvSpacing;
    int pNumX, pNumY, pNumCells;
    int numCellParticles[FLUID_PGRID_CELLS];
    int firstCellParticle[FLUID_PGRID_CELLS + 1];
    int cellParticleIds[FLUID_MAX_PARTICLES];
} Fluid;

// Resets the grid, fills a block of particles near the bottom of the tank.
void Fluid_init(Fluid *f);

// Advances the simulation by dt seconds under gravity vector (gravityX, gravityY),
// expressed in the same units as the grid (FLUID_H per cell).
void Fluid_step(Fluid *f, float dt, float gravityX, float gravityY);

// Local water density at LED cell (x, y), x/y in [0, 7].
float Fluid_cellDensity(const Fluid *f, int x, int y);

// Reference "full" density once the fluid has settled; 0 until then.
float Fluid_restDensity(const Fluid *f);

#endif // FLUID_H
