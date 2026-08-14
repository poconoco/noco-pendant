/*****************************************************************************
* | File      	:   FaceSwitcher.h
* | Function    :   Shows one Face at a time from a fixed list of factories,
*                    creating it on demand and advancing to the next on a
*                    double-tap gesture.
******************************************************************************/
#ifndef FACESWITCHER_H
#define FACESWITCHER_H

#include "DoubleTapDetector.h"
#include "Face.h"

#include <span>

// Displays exactly one Face at a time out of a fixed list of FaceFactory
// (see Face.h), advancing to the next when the board's face is
// double-tapped (see
// DoubleTapDetector, applied to the accelerometer's Z axis -- the board's
// face normal -- so sideways nudges don't trigger it). Implements Face
// itself, so it's a drop-in stand-in for a single Face from the caller's
// point of view.
//
// Only the Face on screen exists: switching deletes the outgoing one before
// creating the incoming one, so listing a Face costs nothing until it's
// actually shown. That matters because some Faces are large and slow to
// build (a FluidFace carries the whole ~12KB FLIP/PIC solver and seeds its
// particles in the constructor), and building every one of them up front
// would waste most of the board's RAM and stall startup for no benefit.
//
// The flip side is that a Face keeps no state across a switch -- coming back
// to e.g. SnakeFace starts a new game rather than resuming the one left
// behind.
class FaceSwitcher : public Face {
public:
    // factories: a view over a caller-owned, contiguous sequence of
    // FaceFactory (e.g. a `static constexpr FaceFactory kFaces[] = {...}`),
    // kept alive for the lifetime of this FaceSwitcher. A span carries its
    // own size, so there's no separate count parameter that could drift out
    // of sync with it. Must contain at least one factory.
    explicit FaceSwitcher(std::span<const FaceFactory> factories) : factories_(factories) {
        activate(0);
    }

    ~FaceSwitcher() override { delete active_; }

    void feedImu(const ImuSample &sample) override {
        lastSample_ = sample;
        haveSample_ = true;
        active_->feedImu(sample);
    }

    FaceFrame getFrame(uint32_t dtUs) override {
        nowUs_ += dtUs;

        if (haveSample_) {
            float dtSeconds = dtUs / 1000000.0f;
            // Only the Z axis (the board's face normal) counts as a tap
            // here, so a knock on the face switches faces but sideways
            // handling doesn't.
            if (tapDetector_.update(lastSample_.accel[2], dtSeconds, nowUs_)) {
                activate((activeIndex_ + 1) % factories_.size());
                // The newly active face hasn't seen the latest sample yet.
                active_->feedImu(lastSample_);
            }
        }

        return active_->getFrame(dtUs);
    }

private:
    // Destroys whichever Face is live and creates the one at `index` in its
    // place -- in that order, so only one Face is ever allocated at a time
    // and the heap never has to hold two. Deletion goes through Face's
    // virtual destructor, so the outgoing Face is destroyed as its real type
    // even though only a Face * is held here.
    void activate(std::size_t index) {
        delete active_;
        activeIndex_ = index;
        active_ = factories_[index]();
    }

    std::span<const FaceFactory> factories_;
    Face *active_ = nullptr;
    std::size_t activeIndex_ = 0;

    DoubleTapDetector tapDetector_;
    ImuSample lastSample_{};
    bool haveSample_ = false;
    uint64_t nowUs_ = 0;
};

#endif // FACESWITCHER_H
