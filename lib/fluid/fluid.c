/*****************************************************************************
* | File      	:   fluid.c
* | Function    :   2D FLIP/PIC fluid solver sized for an 8x8 LED matrix
* | Info        :   Ported from Matthias Muller's "18-flip.html"
*                    (tenMinutePhysics, MIT licensed). Algorithm unchanged;
*                    gravity is a full 2D vector instead of a fixed -y pull,
*                    and there is no drag obstacle.
******************************************************************************/
#include "fluid.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

#define PARTICLE_ITERS 2
#define PRESSURE_ITERS 30
#define OVER_RELAXATION 1.9f
#define FLIP_RATIO 0.9f

static inline int idx(int i, int j) { return i * FLUID_NUM_Y + j; }

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float minf(float a, float b) { return a < b ? a : b; }
static inline float maxf(float a, float b) { return a > b ? a : b; }
static inline int mini(int a, int b) { return a < b ? a : b; }
static inline int maxi(int a, int b) { return a > b ? a : b; }

void Fluid_init(Fluid *f) {
    memset(f, 0, sizeof(*f));

    f->density = 1000.0f;
    f->particleRadius = FLUID_PARTICLE_RADIUS;

    f->pInvSpacing = 1.0f / (2.2f * f->particleRadius);
    f->pNumX = (int)(FLUID_NUM_X * FLUID_H * f->pInvSpacing) + 1;
    f->pNumY = (int)(FLUID_NUM_Y * FLUID_H * f->pInvSpacing) + 1;
    f->pNumCells = f->pNumX * f->pNumY;

    // Solid border, open interior.
    for (int i = 0; i < FLUID_NUM_X; i++) {
        for (int j = 0; j < FLUID_NUM_Y; j++) {
            bool solid = (i == 0 || i == FLUID_NUM_X - 1 || j == 0 || j == FLUID_NUM_Y - 1);
            f->s[idx(i, j)] = solid ? 0.0f : 1.0f;
        }
    }

    // Seed a block of particles resting near the bottom of the tank.
    float r = f->particleRadius;
    float dx = 2.0f * r;
    float dy = sqrtf(3.0f) * 0.5f * dx;

    float minX = FLUID_H + r;
    float maxX = (FLUID_NUM_X - 1) * FLUID_H - r;
    float fillMinY = FLUID_H + r;
    float fillMaxY = fillMinY + 2.0f;

    int numX = (int)((maxX - minX) / dx);
    int numY = (int)((fillMaxY - fillMinY) / dy);
    if (numX * numY > FLUID_MAX_PARTICLES) {
        numY = FLUID_MAX_PARTICLES / numX;
    }

    int p = 0;
    for (int i = 0; i < numX; i++) {
        for (int j = 0; j < numY; j++) {
            f->particlePosX[p] = minX + dx * i + ((j % 2 == 0) ? 0.0f : r);
            f->particlePosY[p] = fillMinY + dy * j;
            p++;
        }
    }
    f->numParticles = p;
}

static void integrateParticles(Fluid *f, float dt, float gravityX, float gravityY) {
    for (int i = 0; i < f->numParticles; i++) {
        f->particleVelX[i] += dt * gravityX;
        f->particleVelY[i] += dt * gravityY;
        f->particlePosX[i] += f->particleVelX[i] * dt;
        f->particlePosY[i] += f->particleVelY[i] * dt;
    }
}

static void pushParticlesApart(Fluid *f, int numIters) {
    // Bucket particles into the spatial hash grid.
    memset(f->numCellParticles, 0, sizeof(int) * f->pNumCells);

    for (int i = 0; i < f->numParticles; i++) {
        int xi = (int)clampf(floorf(f->particlePosX[i] * f->pInvSpacing), 0, f->pNumX - 1);
        int yi = (int)clampf(floorf(f->particlePosY[i] * f->pInvSpacing), 0, f->pNumY - 1);
        f->numCellParticles[xi * f->pNumY + yi]++;
    }

    int first = 0;
    for (int i = 0; i < f->pNumCells; i++) {
        first += f->numCellParticles[i];
        f->firstCellParticle[i] = first;
    }
    f->firstCellParticle[f->pNumCells] = first;

    for (int i = 0; i < f->numParticles; i++) {
        int xi = (int)clampf(floorf(f->particlePosX[i] * f->pInvSpacing), 0, f->pNumX - 1);
        int yi = (int)clampf(floorf(f->particlePosY[i] * f->pInvSpacing), 0, f->pNumY - 1);
        int cellNr = xi * f->pNumY + yi;
        f->firstCellParticle[cellNr]--;
        f->cellParticleIds[f->firstCellParticle[cellNr]] = i;
    }

    float minDist = 2.0f * f->particleRadius;
    float minDist2 = minDist * minDist;

    for (int iter = 0; iter < numIters; iter++) {
        for (int i = 0; i < f->numParticles; i++) {
            float px = f->particlePosX[i];
            float py = f->particlePosY[i];

            int pxi = (int)floorf(px * f->pInvSpacing);
            int pyi = (int)floorf(py * f->pInvSpacing);
            int x0 = maxi(pxi - 1, 0);
            int y0 = maxi(pyi - 1, 0);
            int x1 = mini(pxi + 1, f->pNumX - 1);
            int y1 = mini(pyi + 1, f->pNumY - 1);

            for (int xi = x0; xi <= x1; xi++) {
                for (int yi = y0; yi <= y1; yi++) {
                    int cellNr = xi * f->pNumY + yi;
                    int first_ = f->firstCellParticle[cellNr];
                    int last_ = f->firstCellParticle[cellNr + 1];
                    for (int k = first_; k < last_; k++) {
                        int id = f->cellParticleIds[k];
                        if (id == i) continue;
                        float qx = f->particlePosX[id];
                        float qy = f->particlePosY[id];

                        float dx = qx - px;
                        float dy = qy - py;
                        float d2 = dx * dx + dy * dy;
                        if (d2 > minDist2 || d2 == 0.0f) continue;
                        float d = sqrtf(d2);
                        float s = 0.5f * (minDist - d) / d;
                        dx *= s;
                        dy *= s;
                        f->particlePosX[i] -= dx;
                        f->particlePosY[i] -= dy;
                        f->particlePosX[id] += dx;
                        f->particlePosY[id] += dy;
                    }
                }
            }
        }
    }
}

static void handleParticleCollisions(Fluid *f) {
    float h = FLUID_H;
    float r = f->particleRadius;

    float minX = h + r;
    float maxX = (FLUID_NUM_X - 1) * h - r;
    float minY = h + r;
    float maxY = (FLUID_NUM_Y - 1) * h - r;

    for (int i = 0; i < f->numParticles; i++) {
        float x = f->particlePosX[i];
        float y = f->particlePosY[i];

        if (x < minX) { x = minX; f->particleVelX[i] = 0.0f; }
        if (x > maxX) { x = maxX; f->particleVelX[i] = 0.0f; }
        if (y < minY) { y = minY; f->particleVelY[i] = 0.0f; }
        if (y > maxY) { y = maxY; f->particleVelY[i] = 0.0f; }

        f->particlePosX[i] = x;
        f->particlePosY[i] = y;
    }
}

static void updateParticleDensity(Fluid *f) {
    int n = FLUID_NUM_Y;
    float h = FLUID_H;
    float h1 = 1.0f / h;
    float h2 = 0.5f * h;

    float *d = f->particleDensity;
    memset(d, 0, sizeof(float) * FLUID_NUM_CELLS);

    for (int i = 0; i < f->numParticles; i++) {
        float x = clampf(f->particlePosX[i], h, (FLUID_NUM_X - 1) * h);
        float y = clampf(f->particlePosY[i], h, (FLUID_NUM_Y - 1) * h);

        int x0 = (int)floorf((x - h2) * h1);
        float tx = ((x - h2) - x0 * h) * h1;
        int x1 = mini(x0 + 1, FLUID_NUM_X - 2);

        int y0 = (int)floorf((y - h2) * h1);
        float ty = ((y - h2) - y0 * h) * h1;
        int y1 = mini(y0 + 1, FLUID_NUM_Y - 2);

        float sx = 1.0f - tx;
        float sy = 1.0f - ty;

        if (x0 < FLUID_NUM_X && y0 < FLUID_NUM_Y) d[x0 * n + y0] += sx * sy;
        if (x1 < FLUID_NUM_X && y0 < FLUID_NUM_Y) d[x1 * n + y0] += tx * sy;
        if (x1 < FLUID_NUM_X && y1 < FLUID_NUM_Y) d[x1 * n + y1] += tx * ty;
        if (x0 < FLUID_NUM_X && y1 < FLUID_NUM_Y) d[x0 * n + y1] += sx * ty;
    }

    if (f->particleRestDensity == 0.0f) {
        float sum = 0.0f;
        int numFluidCells = 0;
        for (int i = 0; i < FLUID_NUM_CELLS; i++) {
            if (f->cellType[i] == FLUID_CELL) {
                sum += d[i];
                numFluidCells++;
            }
        }
        if (numFluidCells > 0) {
            f->particleRestDensity = sum / numFluidCells;
        }
    }
}

static void transferVelocities(Fluid *f, bool toGrid, float flipRatio) {
    int n = FLUID_NUM_Y;
    float h = FLUID_H;
    float h1 = 1.0f / h;
    float h2 = 0.5f * h;

    if (toGrid) {
        memcpy(f->prevU, f->u, sizeof(f->u));
        memcpy(f->prevV, f->v, sizeof(f->v));

        memset(f->du, 0, sizeof(f->du));
        memset(f->dv, 0, sizeof(f->dv));
        memset(f->u, 0, sizeof(f->u));
        memset(f->v, 0, sizeof(f->v));

        for (int i = 0; i < FLUID_NUM_CELLS; i++) {
            f->cellType[i] = (f->s[i] == 0.0f) ? SOLID_CELL : AIR_CELL;
        }

        for (int i = 0; i < f->numParticles; i++) {
            int xi = (int)clampf(floorf(f->particlePosX[i] * h1), 0, FLUID_NUM_X - 1);
            int yi = (int)clampf(floorf(f->particlePosY[i] * h1), 0, FLUID_NUM_Y - 1);
            int cellNr = xi * n + yi;
            if (f->cellType[cellNr] == AIR_CELL) {
                f->cellType[cellNr] = FLUID_CELL;
            }
        }
    }

    for (int component = 0; component < 2; component++) {
        float dx = (component == 0) ? 0.0f : h2;
        float dy = (component == 0) ? h2 : 0.0f;

        float *farr = (component == 0) ? f->u : f->v;
        float *prevF = (component == 0) ? f->prevU : f->prevV;
        float *dArr = (component == 0) ? f->du : f->dv;

        for (int i = 0; i < f->numParticles; i++) {
            float x = clampf(f->particlePosX[i], h, (FLUID_NUM_X - 1) * h);
            float y = clampf(f->particlePosY[i], h, (FLUID_NUM_Y - 1) * h);

            int x0 = mini((int)floorf((x - dx) * h1), FLUID_NUM_X - 2);
            float tx = ((x - dx) - x0 * h) * h1;
            int x1 = mini(x0 + 1, FLUID_NUM_X - 2);

            int y0 = mini((int)floorf((y - dy) * h1), FLUID_NUM_Y - 2);
            float ty = ((y - dy) - y0 * h) * h1;
            int y1 = mini(y0 + 1, FLUID_NUM_Y - 2);

            float sx = 1.0f - tx;
            float sy = 1.0f - ty;

            float d0 = sx * sy, d1 = tx * sy, d2 = tx * ty, d3 = sx * ty;
            int nr0 = x0 * n + y0, nr1 = x1 * n + y0, nr2 = x1 * n + y1, nr3 = x0 * n + y1;

            if (toGrid) {
                float pv = (component == 0) ? f->particleVelX[i] : f->particleVelY[i];
                farr[nr0] += pv * d0; dArr[nr0] += d0;
                farr[nr1] += pv * d1; dArr[nr1] += d1;
                farr[nr2] += pv * d2; dArr[nr2] += d2;
                farr[nr3] += pv * d3; dArr[nr3] += d3;
            } else {
                int offset = (component == 0) ? n : 1;
                float valid0 = (f->cellType[nr0] != AIR_CELL || f->cellType[nr0 - offset] != AIR_CELL) ? 1.0f : 0.0f;
                float valid1 = (f->cellType[nr1] != AIR_CELL || f->cellType[nr1 - offset] != AIR_CELL) ? 1.0f : 0.0f;
                float valid2 = (f->cellType[nr2] != AIR_CELL || f->cellType[nr2 - offset] != AIR_CELL) ? 1.0f : 0.0f;
                float valid3 = (f->cellType[nr3] != AIR_CELL || f->cellType[nr3 - offset] != AIR_CELL) ? 1.0f : 0.0f;

                float v = (component == 0) ? f->particleVelX[i] : f->particleVelY[i];
                float dsum = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;

                if (dsum > 0.0f) {
                    float picV = (valid0 * d0 * farr[nr0] + valid1 * d1 * farr[nr1] +
                                  valid2 * d2 * farr[nr2] + valid3 * d3 * farr[nr3]) / dsum;
                    float corr = (valid0 * d0 * (farr[nr0] - prevF[nr0]) + valid1 * d1 * (farr[nr1] - prevF[nr1]) +
                                  valid2 * d2 * (farr[nr2] - prevF[nr2]) + valid3 * d3 * (farr[nr3] - prevF[nr3])) / dsum;
                    float flipV = v + corr;
                    float blended = (1.0f - flipRatio) * picV + flipRatio * flipV;
                    if (component == 0) f->particleVelX[i] = blended;
                    else f->particleVelY[i] = blended;
                }
            }
        }

        if (toGrid) {
            for (int i = 0; i < FLUID_NUM_CELLS; i++) {
                if (dArr[i] > 0.0f) farr[i] /= dArr[i];
            }

            for (int i = 0; i < FLUID_NUM_X; i++) {
                for (int j = 0; j < FLUID_NUM_Y; j++) {
                    bool solid = f->cellType[idx(i, j)] == SOLID_CELL;
                    if (solid || (i > 0 && f->cellType[idx(i - 1, j)] == SOLID_CELL)) {
                        f->u[idx(i, j)] = f->prevU[idx(i, j)];
                    }
                    if (solid || (j > 0 && f->cellType[idx(i, j - 1)] == SOLID_CELL)) {
                        f->v[idx(i, j)] = f->prevV[idx(i, j)];
                    }
                }
            }
        }
    }
}

static void solveIncompressibility(Fluid *f, int numIters, float dt, float overRelaxation, bool compensateDrift) {
    memset(f->p, 0, sizeof(f->p));
    memcpy(f->prevU, f->u, sizeof(f->u));
    memcpy(f->prevV, f->v, sizeof(f->v));

    int n = FLUID_NUM_Y;
    float cp = f->density * FLUID_H / dt;

    for (int iter = 0; iter < numIters; iter++) {
        for (int i = 1; i < FLUID_NUM_X - 1; i++) {
            for (int j = 1; j < FLUID_NUM_Y - 1; j++) {
                if (f->cellType[idx(i, j)] != FLUID_CELL) continue;

                int center = idx(i, j);
                int left = idx(i - 1, j);
                int right = idx(i + 1, j);
                int bottom = idx(i, j - 1);
                int top = idx(i, j + 1);

                float sx0 = f->s[left];
                float sx1 = f->s[right];
                float sy0 = f->s[bottom];
                float sy1 = f->s[top];
                float sSum = sx0 + sx1 + sy0 + sy1;
                if (sSum == 0.0f) continue;

                float div = f->u[right] - f->u[center] + f->v[top] - f->v[center];

                if (f->particleRestDensity > 0.0f && compensateDrift) {
                    float k = 1.0f;
                    float compression = f->particleDensity[center] - f->particleRestDensity;
                    if (compression > 0.0f) div -= k * compression;
                }

                float p = -div / sSum;
                p *= overRelaxation;
                f->p[center] += cp * p;

                f->u[center] -= sx0 * p;
                f->u[right] += sx1 * p;
                f->v[center] -= sy0 * p;
                f->v[top] += sy1 * p;
            }
        }
    }
}

void Fluid_step(Fluid *f, float dt, float gravityX, float gravityY) {
    integrateParticles(f, dt, gravityX, gravityY);
    pushParticlesApart(f, PARTICLE_ITERS);
    handleParticleCollisions(f);
    transferVelocities(f, true, 0.0f);
    updateParticleDensity(f);
    solveIncompressibility(f, PRESSURE_ITERS, dt, OVER_RELAXATION, true);
    transferVelocities(f, false, FLIP_RATIO);
}

float Fluid_cellDensity(const Fluid *f, int x, int y) {
    return f->particleDensity[idx(x + 1, y + 1)];
}

float Fluid_restDensity(const Fluid *f) {
    return f->particleRestDensity;
}
