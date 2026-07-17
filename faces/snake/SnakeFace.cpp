/*****************************************************************************
* | File      	:   SnakeFace.cpp
* | Function    :   Face implementation rendering a classic Snake game
*                    controlled by tilt.
******************************************************************************/
#include "SnakeFace.h"

#include "pico/time.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

namespace {

// Game logic works in the same nominal "USB port up" orientation as the
// rest of the codebase (see Face.h's coordinate comment); like
// MinecraftFace/FlappyFace, the board is worn rotated 180 degrees from that
// as a pendant, so getFrame applies the same 180-degree flip when writing
// pixels to keep every Face consistently oriented for the wearer.

// Tilt past this magnitude (in g) counts as a directional intent; below it,
// the snake just keeps going straight. Same axes and sign convention as
// FluidFace's gravityX_/gravityY_, so tilting a given way steers the snake
// the same direction water would drift under that tilt.
constexpr float kTiltThreshold = 0.25f;

constexpr float kMoveIntervalS = 0.4f; // seconds per grid step -- tune to taste

constexpr float kBlinkDuration = 0.75f;
constexpr float kBlinkPeriod = 0.15f;

constexpr Color kSnakeColor = {1.0f, 0.8f, 0.0f};
constexpr Color kFoodColor = {1.0f, 0.0f, 0.0f};
constexpr Color kOffColor = {0.0f, 0.0f, 0.0f};

} // namespace

bool SnakeFace::isOpposite(Direction a, Direction b) {
    return (a == Direction::Up && b == Direction::Down) || (a == Direction::Down && b == Direction::Up) ||
           (a == Direction::Left && b == Direction::Right) || (a == Direction::Right && b == Direction::Left);
}

SnakeFace::SnakeFace() {
    srand(static_cast<unsigned>(time_us_64()));
    resetGame();
}

void SnakeFace::resetGame() {
    headIndex_ = 0;
    length_ = 1;
    body_[0] = Cell{FACE_WIDTH / 2, FACE_HEIGHT / 2};
    direction_ = Direction::Right;
    lastMovedDirection_ = Direction::Right;
    moveTimer_ = 0.0f;
    blinkTimer_ = 0.0f;
    spawnFood();
}

SnakeFace::Cell SnakeFace::segmentAt(int indexFromHead) const {
    int idx = (headIndex_ - indexFromHead + kMaxLength) % kMaxLength;
    return body_[idx];
}

bool SnakeFace::isOccupied(int x, int y) const {
    for (int i = 0; i < length_; i++) {
        Cell c = segmentAt(i);
        if (c.x == x && c.y == y) return true;
    }
    return false;
}

void SnakeFace::spawnFood() {
    bool occupied[FACE_WIDTH][FACE_HEIGHT] = {};
    for (int i = 0; i < length_; i++) {
        Cell c = segmentAt(i);
        occupied[c.x][c.y] = true;
    }

    int freeCount = 0;
    for (int x = 0; x < FACE_WIDTH; x++) {
        for (int y = 0; y < FACE_HEIGHT; y++) {
            if (!occupied[x][y]) freeCount++;
        }
    }
    if (freeCount == 0) return; // board completely full; leave food as-is

    int choice = rand() % freeCount;
    for (int x = 0; x < FACE_WIDTH; x++) {
        for (int y = 0; y < FACE_HEIGHT; y++) {
            if (occupied[x][y]) continue;
            if (choice == 0) {
                foodX_ = x;
                foodY_ = y;
                return;
            }
            choice--;
        }
    }
}

void SnakeFace::feedImu(const ImuSample &sample) {
    // accel is in milli-g. Confirmed on-device: FluidFace's gravityX_/
    // gravityY_ sign convention (-ax, ay) reads backwards for Snake's
    // discrete up/down/left/right steering, so both axes are inverted here.
    float ax = sample.accel[0] / 1000.0f;
    float ay = sample.accel[1] / 1000.0f;
    tiltX_ = ax;
    tiltY_ = -ay;

    float absX = std::fabs(tiltX_);
    float absY = std::fabs(tiltY_);
    if (absX <= kTiltThreshold && absY <= kTiltThreshold) {
        return; // no clear intent; keep the current heading
    }

    Direction desired = (absX > absY) ? (tiltX_ > 0.0f ? Direction::Right : Direction::Left)
                                       : (tiltY_ > 0.0f ? Direction::Down : Direction::Up);

    // Guarded against the last *executed* move direction, not whatever
    // direction_ happens to be right now -- so a fast multi-axis tilt can't
    // sneak the snake into reversing on itself via an intermediate
    // perpendicular step between two grid ticks.
    if (!isOpposite(desired, lastMovedDirection_)) {
        direction_ = desired;
    }
}

FaceFrame SnakeFace::getFrame(uint32_t dtUs) {
    float dt = dtUs / 1000000.0f;

    if (blinkTimer_ > 0.0f) {
        // Frozen in place at the moment of the hit -- nothing moves, the
        // offending cell just blinks until the timer runs out, then resets.
        blinkTimer_ -= dt;
        if (blinkTimer_ <= 0.0f) {
            blinkTimer_ = 0.0f;
            resetGame();
        }
    } else {
        moveTimer_ += dt;
        while (moveTimer_ >= kMoveIntervalS) {
            moveTimer_ -= kMoveIntervalS;

            Cell head = segmentAt(0);
            int dx = 0, dy = 0;
            switch (direction_) {
                case Direction::Up: dy = -1; break;
                case Direction::Down: dy = 1; break;
                case Direction::Left: dx = -1; break;
                case Direction::Right: dx = 1; break;
            }
            int newX = head.x + dx;
            int newY = head.y + dy;

            bool hitWall = newX < 0 || newX >= FACE_WIDTH || newY < 0 || newY >= FACE_HEIGHT;
            bool willEat = !hitWall && newX == foodX_ && newY == foodY_;

            bool hitSelf = false;
            if (!hitWall) {
                // The tail cell is vacating this same tick unless the snake
                // is also growing here, so it's excluded from the check --
                // moving into the cell the tail is just leaving is legal.
                int checkLength = willEat ? length_ : length_ - 1;
                for (int i = 0; i < checkLength; i++) {
                    Cell c = segmentAt(i);
                    if (c.x == newX && c.y == newY) {
                        hitSelf = true;
                        break;
                    }
                }
            }

            if (hitWall || hitSelf) {
                blinkTimer_ = kBlinkDuration;
                blinkX_ = hitWall ? head.x : newX;
                blinkY_ = hitWall ? head.y : newY;
                break;
            }

            headIndex_ = (headIndex_ + 1) % kMaxLength;
            body_[headIndex_] = Cell{newX, newY};
            lastMovedDirection_ = direction_;
            if (willEat) {
                length_ = std::min(length_ + 1, kMaxLength);
                spawnFood();
            }
        }
    }

    bool blinkOn = blinkTimer_ > 0.0f && std::fmod(blinkTimer_, kBlinkPeriod) < kBlinkPeriod * 0.5f;

    FaceFrame frame{};
    for (int ny = 0; ny < FACE_HEIGHT; ny++) {
        for (int nx = 0; nx < FACE_WIDTH; nx++) {
            Color color = kOffColor;

            if (blinkTimer_ > 0.0f && nx == blinkX_ && ny == blinkY_) {
                color = blinkOn ? kSnakeColor : kOffColor;
            } else if (isOccupied(nx, ny)) {
                color = kSnakeColor;
            } else if (nx == foodX_ && ny == foodY_) {
                color = kFoodColor;
            }

            int px = FACE_WIDTH - 1 - nx;
            int py = FACE_HEIGHT - 1 - ny;
            frame.pixels[px][py] = Color{
                LED_BRIGHTNESS * color.r,
                LED_BRIGHTNESS * color.g,
                LED_BRIGHTNESS * color.b,
            };
        }
    }
    return frame;
}
