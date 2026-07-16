/*****************************************************************************
* | File      	:   FaceSwitcher.h
* | Function    :   Shows one Face at a time from a fixed set, advancing to
*                    the next on a double-tap gesture.
******************************************************************************/
#ifndef FACESWITCHER_H
#define FACESWITCHER_H

#include "DoubleTapDetector.h"
#include "Face.h"

#include <span>

// Wraps a fixed array of Faces and displays exactly one at a time,
// advancing to the next when the board's face is double-tapped (see
// DoubleTapDetector, applied to the accelerometer's Z axis -- the board's
// face normal -- so sideways nudges don't trigger it). Implements Face
// itself, so it's a drop-in stand-in for a single Face from the caller's
// point of view.
class FaceSwitcher : public Face {
public:
    // faces: a view over a caller-owned, contiguous sequence of Face
    // pointers (e.g. a std::array<Face*, N>), kept alive for the lifetime of
    // this FaceSwitcher. A span carries its own size, so there's no separate
    // count parameter that could drift out of sync with it. Must contain at
    // least one face.
    explicit FaceSwitcher(std::span<Face *> faces) : faces_(faces) {}

    void feedImu(const ImuSample &sample) override {
        lastSample_ = sample;
        haveSample_ = true;
        faces_[activeIndex_]->feedImu(sample);
    }

    FaceFrame getFrame(uint32_t dtUs) override {
        nowUs_ += dtUs;

        if (haveSample_) {
            float dtSeconds = dtUs / 1000000.0f;
            // Only the Z axis (the board's face normal) counts as a tap
            // here, so a knock on the face switches faces but sideways
            // handling doesn't.
            if (tapDetector_.update(lastSample_.accel[2], dtSeconds, nowUs_)) {
                activeIndex_ = (activeIndex_ + 1) % faces_.size();
                // The newly active face hasn't seen the latest sample yet.
                faces_[activeIndex_]->feedImu(lastSample_);
            }
        }

        return faces_[activeIndex_]->getFrame(dtUs);
    }

private:
    std::span<Face *> faces_;
    std::size_t activeIndex_ = 0;

    DoubleTapDetector tapDetector_;
    ImuSample lastSample_{};
    bool haveSample_ = false;
    uint64_t nowUs_ = 0;
};

#endif // FACESWITCHER_H
