/*****************************************************************************
* | File      	:   DoubleTapDetector.h
* | Function    :   Double-tap gesture detector from a single accelerometer axis.
******************************************************************************/
#ifndef DOUBLETAP_H
#define DOUBLETAP_H

#include <cmath>
#include <cstdint>

// Detects the classic phone-style "double tap" gesture from one raw
// accelerometer axis (in mg). A tap is a fast, brief spike *on top of
// whatever the board is currently doing* -- not just an absolute level,
// since slow handling/tilting also moves the reading around. A
// slowly-adapting baseline tracks the "current normal" for that axis
// (gravity's current component on it, plus any slow motion); only a sudden
// jump away from that baseline, lasting a moment and then dropping back,
// counts as a tap. Two such taps close together count as a double-tap.
class DoubleTapDetector {
public:
    // Feed the latest reading (mg) and the seconds elapsed + absolute
    // microsecond timestamp for this sample; returns true exactly once when
    // a double-tap completes.
    bool update(float value, float dtSeconds, uint64_t nowUs) {
        if (!haveBaseline_) {
            baseline_ = value;
            haveBaseline_ = true;
        }

        float deviation = value - baseline_;

        // Only let the baseline follow the signal while it's not in the
        // middle of a spike, so a tap doesn't drag its own reference point
        // up with it.
        if (fabsf(deviation) < kDeviationThresholdMg) {
            float alpha = clampf(dtSeconds / kBaselineTauS, 0.0f, 1.0f);
            baseline_ += alpha * (value - baseline_);
        }

        // A knock can spike either positive or negative depending on which
        // way it jolts the sensor, so trigger on the deviation's magnitude.
        bool tapEvent = false;
        if (armed_ && fabsf(deviation) > kDeviationThresholdMg) {
            tapEvent = true;
            armed_ = false; // require settling back near baseline before the next tap can fire
        } else if (!armed_ && fabsf(deviation) < kReleaseDeviationMg) {
            armed_ = true;
        }

        if (!tapEvent) {
            if (haveFirstTap_ && (nowUs - firstTapUs_) > kDoubleTapWindowUs) {
                haveFirstTap_ = false; // first tap went stale, forget it
            }
            return false;
        }

        // A single knock isn't a clean spike -- the board/sensor rings for a
        // bit and can cross the thresholds more than once. Anything this
        // soon after an accepted tap is the same knock settling down, not a
        // second tap, and is ignored.
        if (haveLastTap_ && (nowUs - lastTapUs_) < kDebounceUs) {
            return false;
        }
        lastTapUs_ = nowUs;
        haveLastTap_ = true;

        if (haveFirstTap_ && (nowUs - firstTapUs_) <= kDoubleTapWindowUs) {
            haveFirstTap_ = false;
            return true;
        }

        firstTapUs_ = nowUs;
        haveFirstTap_ = true;
        return false;
    }

private:
    static float clampf(float x, float lo, float hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }

    static constexpr float kBaselineTauS = 0.25f;           // how fast the baseline follows slow motion (bigger = slower to follow, more resistant to being fooled by a tap)
    static constexpr float kDeviationThresholdMg = 2500.0f; // spike size above baseline that counts as a tap candidate
    static constexpr float kReleaseDeviationMg = 900.0f;    // must settle back this close to baseline before the next tap can fire
    static constexpr uint64_t kDoubleTapWindowUs = 400ull * 1000;
    static constexpr uint64_t kDebounceUs = 80ull * 1000;

    bool haveBaseline_ = false;
    float baseline_ = 0.0f;
    bool armed_ = true;
    bool haveFirstTap_ = false;
    uint64_t firstTapUs_ = 0;
    bool haveLastTap_ = false;
    uint64_t lastTapUs_ = 0;
};

#endif // DOUBLETAP_H
