/*****************************************************************************
* | File      	:   Face.h
* | Function    :   Abstract interface for an 8x8 LED matrix visualization
*                    driven by the onboard IMU.
******************************************************************************/
#ifndef FACE_H
#define FACE_H

#include <cstdint>

#define FACE_WIDTH 8
#define FACE_HEIGHT 8

// One IMU sample. Units match the QMI8658 driver: accel in milli-g, gyro in
// degrees/sec, both in the board's own axes (see main.cpp's coordinate
// comment for the axis layout).
struct ImuSample {
    float accel[3];
    float gyro[3];

    // Gravity direction derived from accel, normalized to a unit vector.
    // This is the "combined orientation" reading -- which way is down, in
    // board axes -- computed once so Face implementations that only care
    // about tilt direction (not the raw magnitude) don't each have to
    // normalize it themselves. It's *not* a full attitude (no yaw, and it
    // says nothing about rotation rate); gyro is passed through raw for any
    // Face that wants more than that.
    float orientation[3];
};

// Channel intensities on the same scale as LED_BRIGHTNESS (see WS2812.h),
// i.e. 0..LED_BRIGHTNESS is "full on" for that channel. Kept as floats
// (rather than the uint8_t the hardware ultimately wants) so a Face can hand
// back sub-integer targets -- e.g. 0.4 -- for the caller to dither into an
// on/off pattern over successive frames instead of rounding it down to a
// flat, wrong zero every time. This is also the general-purpose 3-channel
// color type used anywhere a Face needs to name a color (see e.g.
// MinecraftFace's palette and FlappyFace's sprite colors).
struct Color {
    float r, g, b;
};

struct FaceFrame {
    Color pixels[FACE_WIDTH][FACE_HEIGHT];
};

// A Face owns one visualization's state and turns IMU input into an 8x8
// color frame over time. The QMI8658 driver here is polled, not
// interrupt/callback-driven, so Face is fed via a push method rather than a
// registered callback: the caller reads a sample, calls feedImu(), then
// calls getFrame() with the time elapsed since the previous frame.
class Face {
public:
    virtual ~Face() = default;

    // Called once per new IMU sample.
    virtual void feedImu(const ImuSample &sample) = 0;

    // Advances the visualization by dtUs microseconds -- matching the
    // microsecond-resolution clock already used for timing elsewhere, since
    // a millisecond integer would round away most of a sub-millisecond frame
    // time -- and returns the frame to display.
    virtual FaceFrame getFrame(uint32_t dtUs) = 0;
};

#endif // FACE_H
