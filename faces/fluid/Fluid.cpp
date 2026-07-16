/*****************************************************************************
* | File      	:   Fluid.cpp
* | Function    :   2D FLIP/PIC fluid solver sized for an 8x8 LED matrix
* | Info        :   Ported from Matthias Muller's "18-flip.html"
*                    (tenMinutePhysics, MIT licensed). Algorithm unchanged;
*                    gravity is a full 2D vector instead of a fixed -y pull,
*                    and there is no drag obstacle.
******************************************************************************/
#include "Fluid.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int kParticleIters = 2;
constexpr int kPressureIters = 30;
constexpr float kOverRelaxation = 1.9f;
constexpr float kFlipRatio = 0.98f;
} // namespace

Fluid::Fluid() {
    pInvSpacing_ = 1.0f / (2.2f * particleRadius_);
    pNumX_ = static_cast<int>(kNumX * kCellSize * pInvSpacing_) + 1;
    pNumY_ = static_cast<int>(kNumY * kCellSize * pInvSpacing_) + 1;
    pNumCells_ = pNumX_ * pNumY_;

    // Solid border, open interior.
    for (int i = 0; i < kNumX; i++) {
        for (int j = 0; j < kNumY; j++) {
            bool solid = (i == 0 || i == kNumX - 1 || j == 0 || j == kNumY - 1);
            s_[idx(i, j)] = solid ? 0.0f : 1.0f;
        }
    }

    // Seed a block of particles resting near the bottom of the tank.
    float r = particleRadius_;
    float dx = 2.0f * r;
    float dy = sqrtf(3.0f) * 0.5f * dx;

    float minX = kCellSize + r;
    float maxX = (kNumX - 1) * kCellSize - r;
    float fillMinY = kCellSize + r;
    float fillMaxY = fillMinY + 2.0f;

    int numX = static_cast<int>((maxX - minX) / dx);
    int numY = static_cast<int>((fillMaxY - fillMinY) / dy);
    if (numX * numY > kMaxParticles) {
        numY = kMaxParticles / numX;
    }

    int p = 0;
    for (int i = 0; i < numX; i++) {
        for (int j = 0; j < numY; j++) {
            particlePosX_[p] = minX + dx * i + ((j % 2 == 0) ? 0.0f : r);
            particlePosY_[p] = fillMinY + dy * j;
            p++;
        }
    }
    numParticles_ = p;
}

void Fluid::integrateParticles(float dt, float gravityX, float gravityY) {
    for (int i = 0; i < numParticles_; i++) {
        particleVelX_[i] += dt * gravityX;
        particleVelY_[i] += dt * gravityY;
        particlePosX_[i] += particleVelX_[i] * dt;
        particlePosY_[i] += particleVelY_[i] * dt;
    }
}

void Fluid::pushParticlesApart(int numIters) {
    // Bucket particles into the spatial hash grid.
    std::fill_n(numCellParticles_.begin(), pNumCells_, 0);

    for (int i = 0; i < numParticles_; i++) {
        int xi = static_cast<int>(std::clamp(floorf(particlePosX_[i] * pInvSpacing_), 0.0f, static_cast<float>(pNumX_ - 1)));
        int yi = static_cast<int>(std::clamp(floorf(particlePosY_[i] * pInvSpacing_), 0.0f, static_cast<float>(pNumY_ - 1)));
        numCellParticles_[xi * pNumY_ + yi]++;
    }

    int first = 0;
    for (int i = 0; i < pNumCells_; i++) {
        first += numCellParticles_[i];
        firstCellParticle_[i] = first;
    }
    firstCellParticle_[pNumCells_] = first;

    for (int i = 0; i < numParticles_; i++) {
        int xi = static_cast<int>(std::clamp(floorf(particlePosX_[i] * pInvSpacing_), 0.0f, static_cast<float>(pNumX_ - 1)));
        int yi = static_cast<int>(std::clamp(floorf(particlePosY_[i] * pInvSpacing_), 0.0f, static_cast<float>(pNumY_ - 1)));
        int cellNr = xi * pNumY_ + yi;
        firstCellParticle_[cellNr]--;
        cellParticleIds_[firstCellParticle_[cellNr]] = i;
    }

    float minDist = 2.0f * particleRadius_;
    float minDist2 = minDist * minDist;

    for (int iter = 0; iter < numIters; iter++) {
        for (int i = 0; i < numParticles_; i++) {
            float px = particlePosX_[i];
            float py = particlePosY_[i];

            int pxi = static_cast<int>(floorf(px * pInvSpacing_));
            int pyi = static_cast<int>(floorf(py * pInvSpacing_));
            int x0 = std::max(pxi - 1, 0);
            int y0 = std::max(pyi - 1, 0);
            int x1 = std::min(pxi + 1, pNumX_ - 1);
            int y1 = std::min(pyi + 1, pNumY_ - 1);

            for (int xi = x0; xi <= x1; xi++) {
                for (int yi = y0; yi <= y1; yi++) {
                    int cellNr = xi * pNumY_ + yi;
                    int first_ = firstCellParticle_[cellNr];
                    int last_ = firstCellParticle_[cellNr + 1];
                    for (int k = first_; k < last_; k++) {
                        int id = cellParticleIds_[k];
                        if (id == i) continue;
                        float qx = particlePosX_[id];
                        float qy = particlePosY_[id];

                        float dx = qx - px;
                        float dy = qy - py;
                        float d2 = dx * dx + dy * dy;
                        if (d2 > minDist2 || d2 == 0.0f) continue;
                        float d = sqrtf(d2);
                        float s = 0.5f * (minDist - d) / d;
                        dx *= s;
                        dy *= s;
                        particlePosX_[i] -= dx;
                        particlePosY_[i] -= dy;
                        particlePosX_[id] += dx;
                        particlePosY_[id] += dy;
                    }
                }
            }
        }
    }
}

void Fluid::handleParticleCollisions() {
    float h = kCellSize;
    float r = particleRadius_;

    float minX = h + r;
    float maxX = (kNumX - 1) * h - r;
    float minY = h + r;
    float maxY = (kNumY - 1) * h - r;

    for (int i = 0; i < numParticles_; i++) {
        float x = particlePosX_[i];
        float y = particlePosY_[i];

        if (x < minX) { x = minX; particleVelX_[i] = 0.0f; }
        if (x > maxX) { x = maxX; particleVelX_[i] = 0.0f; }
        if (y < minY) { y = minY; particleVelY_[i] = 0.0f; }
        if (y > maxY) { y = maxY; particleVelY_[i] = 0.0f; }

        particlePosX_[i] = x;
        particlePosY_[i] = y;
    }
}

void Fluid::updateParticleDensity() {
    int n = kNumY;
    float h = kCellSize;
    float h1 = 1.0f / h;
    float h2 = 0.5f * h;

    auto &d = particleDensity_;
    d.fill(0.0f);

    for (int i = 0; i < numParticles_; i++) {
        float x = std::clamp(particlePosX_[i], h, (kNumX - 1) * h);
        float y = std::clamp(particlePosY_[i], h, (kNumY - 1) * h);

        int x0 = static_cast<int>(floorf((x - h2) * h1));
        float tx = ((x - h2) - x0 * h) * h1;
        int x1 = std::min(x0 + 1, kNumX - 2);

        int y0 = static_cast<int>(floorf((y - h2) * h1));
        float ty = ((y - h2) - y0 * h) * h1;
        int y1 = std::min(y0 + 1, kNumY - 2);

        float sx = 1.0f - tx;
        float sy = 1.0f - ty;

        if (x0 < kNumX && y0 < kNumY) d[x0 * n + y0] += sx * sy;
        if (x1 < kNumX && y0 < kNumY) d[x1 * n + y0] += tx * sy;
        if (x1 < kNumX && y1 < kNumY) d[x1 * n + y1] += tx * ty;
        if (x0 < kNumX && y1 < kNumY) d[x0 * n + y1] += sx * ty;
    }

    if (particleRestDensity_ == 0.0f) {
        float sum = 0.0f;
        int numFluidCells = 0;
        for (int i = 0; i < kNumCells; i++) {
            if (cellType_[i] == CellType::Fluid) {
                sum += d[i];
                numFluidCells++;
            }
        }
        if (numFluidCells > 0) {
            particleRestDensity_ = sum / numFluidCells;
        }
    }
}

void Fluid::transferVelocities(bool toGrid, float flipRatio) {
    int n = kNumY;
    float h = kCellSize;
    float h1 = 1.0f / h;
    float h2 = 0.5f * h;

    if (toGrid) {
        prevU_ = u_;
        prevV_ = v_;

        du_.fill(0.0f);
        dv_.fill(0.0f);
        u_.fill(0.0f);
        v_.fill(0.0f);

        for (int i = 0; i < kNumCells; i++) {
            cellType_[i] = (s_[i] == 0.0f) ? CellType::Solid : CellType::Air;
        }

        for (int i = 0; i < numParticles_; i++) {
            int xi = static_cast<int>(std::clamp(floorf(particlePosX_[i] * h1), 0.0f, static_cast<float>(kNumX - 1)));
            int yi = static_cast<int>(std::clamp(floorf(particlePosY_[i] * h1), 0.0f, static_cast<float>(kNumY - 1)));
            int cellNr = xi * n + yi;
            if (cellType_[cellNr] == CellType::Air) {
                cellType_[cellNr] = CellType::Fluid;
            }
        }
    }

    for (int component = 0; component < 2; component++) {
        float dx = (component == 0) ? 0.0f : h2;
        float dy = (component == 0) ? h2 : 0.0f;

        std::array<float, kNumCells> &farr = (component == 0) ? u_ : v_;
        std::array<float, kNumCells> &prevF = (component == 0) ? prevU_ : prevV_;
        std::array<float, kNumCells> &dArr = (component == 0) ? du_ : dv_;

        for (int i = 0; i < numParticles_; i++) {
            float x = std::clamp(particlePosX_[i], h, (kNumX - 1) * h);
            float y = std::clamp(particlePosY_[i], h, (kNumY - 1) * h);

            int x0 = std::min(static_cast<int>(floorf((x - dx) * h1)), kNumX - 2);
            float tx = ((x - dx) - x0 * h) * h1;
            int x1 = std::min(x0 + 1, kNumX - 2);

            int y0 = std::min(static_cast<int>(floorf((y - dy) * h1)), kNumY - 2);
            float ty = ((y - dy) - y0 * h) * h1;
            int y1 = std::min(y0 + 1, kNumY - 2);

            float sx = 1.0f - tx;
            float sy = 1.0f - ty;

            float d0 = sx * sy, d1 = tx * sy, d2 = tx * ty, d3 = sx * ty;
            int nr0 = x0 * n + y0, nr1 = x1 * n + y0, nr2 = x1 * n + y1, nr3 = x0 * n + y1;

            if (toGrid) {
                float pv = (component == 0) ? particleVelX_[i] : particleVelY_[i];
                farr[nr0] += pv * d0; dArr[nr0] += d0;
                farr[nr1] += pv * d1; dArr[nr1] += d1;
                farr[nr2] += pv * d2; dArr[nr2] += d2;
                farr[nr3] += pv * d3; dArr[nr3] += d3;
            } else {
                int offset = (component == 0) ? n : 1;
                float valid0 = (cellType_[nr0] != CellType::Air || cellType_[nr0 - offset] != CellType::Air) ? 1.0f : 0.0f;
                float valid1 = (cellType_[nr1] != CellType::Air || cellType_[nr1 - offset] != CellType::Air) ? 1.0f : 0.0f;
                float valid2 = (cellType_[nr2] != CellType::Air || cellType_[nr2 - offset] != CellType::Air) ? 1.0f : 0.0f;
                float valid3 = (cellType_[nr3] != CellType::Air || cellType_[nr3 - offset] != CellType::Air) ? 1.0f : 0.0f;

                float v = (component == 0) ? particleVelX_[i] : particleVelY_[i];
                float dsum = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3;

                if (dsum > 0.0f) {
                    float picV = (valid0 * d0 * farr[nr0] + valid1 * d1 * farr[nr1] +
                                  valid2 * d2 * farr[nr2] + valid3 * d3 * farr[nr3]) / dsum;
                    float corr = (valid0 * d0 * (farr[nr0] - prevF[nr0]) + valid1 * d1 * (farr[nr1] - prevF[nr1]) +
                                  valid2 * d2 * (farr[nr2] - prevF[nr2]) + valid3 * d3 * (farr[nr3] - prevF[nr3])) / dsum;
                    float flipV = v + corr;
                    float blended = (1.0f - flipRatio) * picV + flipRatio * flipV;
                    if (component == 0) particleVelX_[i] = blended;
                    else particleVelY_[i] = blended;
                }
            }
        }

        if (toGrid) {
            for (int i = 0; i < kNumCells; i++) {
                if (dArr[i] > 0.0f) farr[i] /= dArr[i];
            }

            for (int i = 0; i < kNumX; i++) {
                for (int j = 0; j < kNumY; j++) {
                    bool solid = cellType_[idx(i, j)] == CellType::Solid;
                    if (solid || (i > 0 && cellType_[idx(i - 1, j)] == CellType::Solid)) {
                        u_[idx(i, j)] = prevU_[idx(i, j)];
                    }
                    if (solid || (j > 0 && cellType_[idx(i, j - 1)] == CellType::Solid)) {
                        v_[idx(i, j)] = prevV_[idx(i, j)];
                    }
                }
            }
        }
    }
}

void Fluid::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
    p_.fill(0.0f);
    prevU_ = u_;
    prevV_ = v_;

    float cp = density_ * kCellSize / dt;

    for (int iter = 0; iter < numIters; iter++) {
        for (int i = 1; i < kNumX - 1; i++) {
            for (int j = 1; j < kNumY - 1; j++) {
                if (cellType_[idx(i, j)] != CellType::Fluid) continue;

                int center = idx(i, j);
                int left = idx(i - 1, j);
                int right = idx(i + 1, j);
                int bottom = idx(i, j - 1);
                int top = idx(i, j + 1);

                float sx0 = s_[left];
                float sx1 = s_[right];
                float sy0 = s_[bottom];
                float sy1 = s_[top];
                float sSum = sx0 + sx1 + sy0 + sy1;
                if (sSum == 0.0f) continue;

                float div = u_[right] - u_[center] + v_[top] - v_[center];

                if (particleRestDensity_ > 0.0f && compensateDrift) {
                    float k = 1.0f;
                    float compression = particleDensity_[center] - particleRestDensity_;
                    if (compression > 0.0f) div -= k * compression;
                }

                float p = -div / sSum;
                p *= overRelaxation;
                p_[center] += cp * p;

                u_[center] -= sx0 * p;
                u_[right] += sx1 * p;
                v_[center] -= sy0 * p;
                v_[top] += sy1 * p;
            }
        }
    }
}

void Fluid::step(float dt, float gravityX, float gravityY) {
    integrateParticles(dt, gravityX, gravityY);
    pushParticlesApart(kParticleIters);
    handleParticleCollisions();
    transferVelocities(true, 0.0f);
    updateParticleDensity();
    solveIncompressibility(kPressureIters, dt, kOverRelaxation, true);
    transferVelocities(false, kFlipRatio);
}

float Fluid::cellDensity(int x, int y) const {
    return particleDensity_[idx(x + 1, y + 1)];
}
