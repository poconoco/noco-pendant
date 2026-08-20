/*****************************************************************************
* | File      	:   SnakeFace.h
* | Author      :   poconoco, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation rendering a classic Snake game
*                    controlled by tilt.
******************************************************************************/
#ifndef SNAKEFACE_H
#define SNAKEFACE_H

#include "Face.h"

#include <array>

// A minimal classic Snake: a yellow snake moves on an integer 8x8 grid,
// steered by tilt (same accelerometer axes/sign convention as FluidFace's
// gravity), and grows by one segment each time it reaches a red food pixel.
// It can never reverse directly into itself -- tilting the opposite way
// while moving is simply ignored. Every position is a whole grid cell; there
// is no sub-pixel brightness blending here (unlike FlappyFace), since the
// classic blocky feel is the point. Hitting itself or a wall freezes the
// snake in place (the move that would have caused the hit never happens)
// while the offending cell blinks yellow/black, then the game resets --
// there's no game-over screen otherwise, it's meant to run forever as
// ambient decoration, same as the other Faces.
class SnakeFace : public Face {
public:
    SnakeFace();

    void feedImu(const ImuSample &sample) override;
    FaceFrame getFrame(uint32_t dtUs) override;

private:
    enum class Direction { Up, Down, Left, Right };
    struct Cell {
        int x, y;
    };

    void resetGame();
    void spawnFood();
    Cell segmentAt(int indexFromHead) const;
    bool isOccupied(int x, int y) const;
    static bool isOpposite(Direction a, Direction b);

    float tiltX_ = 0.0f;
    float tiltY_ = 0.0f;

    // direction_ is the live, tilt-steered heading applied on the next grid
    // step; lastMovedDirection_ is frozen at whatever direction the snake
    // actually last moved in, and is what the no-reversal check guards
    // against -- so a quick multi-axis tilt between two ticks can never
    // sneak the snake into reversing on itself, even via an intermediate
    // perpendicular direction.
    Direction direction_ = Direction::Right;
    Direction lastMovedDirection_ = Direction::Right;

    static constexpr int kMaxLength = FACE_WIDTH * FACE_HEIGHT;
    std::array<Cell, kMaxLength> body_{};
    int headIndex_ = 0;
    int length_ = 1;

    int foodX_ = 0;
    int foodY_ = 0;

    float moveTimer_ = 0.0f;

    // While positive, the snake is frozen at the moment of a hit and the
    // offending cell blinks yellow/black; reaching zero resets the game.
    float blinkTimer_ = 0.0f;
    int blinkX_ = 0;
    int blinkY_ = 0;
};

#endif // SNAKEFACE_H
