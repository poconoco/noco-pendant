/*****************************************************************************
* | File      	:   CatFace.cpp
* | Author      :   Leonid, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation rendering a small pixel-art cat that
*                   falls under tilt-driven gravity and steps around to keep
*                   its feet pointing downhill.
******************************************************************************/
#include "CatFace.h"

#include <array>
#include <cmath>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

namespace {

// Sprite dimensions, in the canonical "feet at the bottom" authoring
// orientation. Rotating by an odd number of quarter turns swaps these (see
// CatFace::footprintW/H).
constexpr int kCatW = 6;
constexpr int kCatH = 4;

// Converts accelerometer tilt (in g) into gravity units, same axes and sign
// convention as HeartFace and FluidFace so the cat falls the way the water
// does. A little gentler than HeartFace's heart: a cat that rockets across
// the display on the slightest tilt doesn't read as a cat.
constexpr float kGravityScale = 92.0f;

// Wall bounce keeps this fraction of the incoming speed. Deliberately much
// lower than HeartFace's 0.7 -- a cat lands, it doesn't bounce.
constexpr float kRestitution = 0.18f;

// Tangential speed kept through a wall hit, and how fast a resting cat bleeds
// off sliding (per second, exponential). Together these let it slide slowly
// downhill on a steady tilt instead of either skating around forever or
// sticking in place unnaturally.
constexpr float kWallFriction = 0.55f;
constexpr float kGroundDrag = 4.5f;

// Speed into a wall that separates a real impact from merely resting or
// sliding against one. The distinction matters because a cat held against a
// wall by gravity re-penetrates it on every single frame, so treating each of
// those as a fresh collision applied kWallFriction hundreds of times a second
// -- which pinned a falling cat in mid-air by its tail the moment it brushed
// a side wall, and made the damping depend on the frame rate into the bargain.
// A resting contact only ever carries one frame's worth of gravity, so the
// threshold sits above that (at the slowest frame the caller allows) and well
// below anything earned by actually falling.
constexpr float kImpactSpeed = 8.0f;

// How much of a slope the cat's feet can simply grip, as the tangent of the
// steepest angle it holds without sliding -- ordinary static friction, which
// the drag above is not: viscous damping only ever sets a sliding speed, it
// never brings a cat to a stop on a gradient. Without this the cat slides
// downhill faster than kWalkSpeed at anything past a few degrees, so it gets
// pinned against the low wall and every walk it attempts is dragged straight
// back. Chosen to hold a little beyond the 20 degrees that separates walking
// from bracing (tan 24 degrees), so the whole walking range is firmly held,
// while a genuine slope still slides it down into the corner as before.
constexpr float kFootGrip = 0.45f;

// How close the footprint has to be to a wall to count as standing on it.
constexpr float kContactEpsilon = 0.08f;

constexpr float kPi = 3.14159265f;
constexpr float kHalfPi = kPi / 2.0f;
constexpr float kQuarterPi = kPi / 4.0f;

// Past a quarter turn's worth of lean (45 degrees) there is a closer
// 90-degree step available, so the cat steps around. The margin is pure
// hysteresis on top of that threshold, and the dwell is how long the tilt has
// to stay past it before the step is committed -- without both, a board held
// near 45 degrees would have the cat flip-flopping every other frame.
constexpr float kSnapMarginRad = 0.14f; // ~8 degrees
constexpr float kSnapDwellS = 0.10f;

// That margin has a sting in its tail: it is wider than the 45 degree tie
// point, so without this second, slower trigger there is an 8 degree band on
// either side in which a rotation that is NOT the nearest one is perfectly
// stable, and the cat parks in it and never corrects. Which one it ends up in
// then depends only on how the board got there, so the same hang can leave the
// cat visibly more askew than it needs to be -- leaning 50 degrees when a 40
// degree pose was available, dropping its downhill end twice as far below its
// feet. Anything past 45 degrees therefore still gets corrected eventually,
// just patiently: the small margin keeps a board held exactly at the tie point
// from chattering, and the long dwell means only a sustained lean counts.
constexpr float kSettleMarginRad = 0.035f; // ~2 degrees
constexpr float kSettleDwellS = 0.5f;

// The step-around animation: total length, and the push-off it starts with.
// The rotation itself lands at the halfway point, between the splay-footed
// push and the settling step, so the sprite is mid-motion exactly when it
// jumps a quadrant -- which is what sells a 90-degree snap as a step rather
// than a teleport.
//
// The hop has to be sized against the pixel grid, not just picked to feel
// right in float: drawing is quantized to whole pixels, so a push-off that
// peaks less than a pixel up is simply never visible. kHopSpeed is set to
// clear a little over one pixel (rise = v^2 / 2g), which also puts the hop's
// airtime (2v / g) at almost exactly kReorientS -- so the cat pushes off,
// turns at the top of the arc, and lands on the beat.
constexpr float kReorientS = 0.26f;
constexpr float kHopSpeed = 13.0f;
constexpr float kScootSpeed = 4.0f;

// Which way round the cat stands: it turns to put its head on the uphill
// side and its tail downhill, the way an animal stands across a slope. The
// deadband is how much of gravity has to pull along the body's own axis
// before that counts as a slope worth turning for -- it is the sine of the
// lean angle, since that pull is |g| * sin(lean), so 0.34 is 20 degrees.
// Below that the board counts as level, and the cat wanders instead (see
// kWalkSpeed). The flip itself is a short animation through a hunched,
// symmetric frame, which is what keeps a mirrored sprite from simply
// snapping inside out.
constexpr float kFacingDeadband = 0.342f * kGravityScale; // sin(20 degrees)
constexpr float kFacingDwellS = 0.15f;
constexpr float kFlipS = 0.18f;

// Strolling, which is what the cat does instead of turning while the board is
// within kFacingDeadband of level. Each outing covers one or two whole
// pixels at this speed, slow enough that the borrowed shuffle frames read as
// footfalls rather than a scurry. Walking into the wall ahead turns the cat
// around, so the next outing sets off the other way.
constexpr float kWalkSpeed = 2.5f;


// Movement below this speed (in pixels/sec) counts as settled, which is what
// gates the idle fidgets.
constexpr float kSettleSpeed = 0.6f;
constexpr float kSettleS = 0.45f;

// Idle fidget lengths, and the range of quiet time between them.
constexpr float kTailFlickS = 0.5f;
constexpr float kEarTwitchS = 0.3f;
constexpr float kShuffleS = 0.6f;
constexpr float kIdleGapMinS = 1.5f;
constexpr float kIdleGapMaxS = 4.5f;

// Quiet time between walks, on their own clock so wandering can be made more
// or less frequent without touching how often the cat flicks its tail. The
// walk itself and the turn at the wall add most of a second on top of this,
// so the realised interval runs longer than the gap alone suggests.
constexpr float kWalkGapMinS = 0.4f;
constexpr float kWalkGapMaxS = 2.6f;

// Blinking keeps its own clock rather than joining the fidget rotation above.
// It drives a different sprite row, so it can happen partway through a tail
// flick the way a real cat blinks while doing something else -- and it wants
// its own rate, not whatever share of the shared one it would get.
//
// Gaps are drawn from an exponential distribution, i.e. blinks are a Poisson
// process: that is what makes them land unevenly, two in quick succession and
// then a long stare, rather than on a metronome. kBlinkMeanGapS backs the
// blink's own duration and the minimum gap out of the target period, so what
// actually averages kBlinkPerSecond is the whole eyes-open-to-eyes-open cycle.
constexpr float kBlinkPerSecond = 0.6f; // 6 per 10 seconds
constexpr float kBlinkS = 0.12f;
constexpr float kBlinkMinGapS = 0.25f;
constexpr float kBlinkMeanGapS = 1.0f / kBlinkPerSecond - kBlinkS - kBlinkMinGapS;

// Half-pixel quantization deadband, in pixels: the drawn position only moves
// once the true position is this far past the midpoint between two pixels.
// Without it a cat resting with its center on a .5 boundary would flicker
// between two columns as the last physics digits wobble.
constexpr float kPixelHysteresis = 0.12f;

// The same orange as the orange FluidFace in main.cpp's face list.
constexpr Color kCatColor = {1.0f, 0.1f, 0.0f};

// The sprite, authored 6 wide and 4 tall with the head and ears on the left,
// the tail on the right, and the feet on the bottom row:
//
//     # . # . . #     ears (x0, x2) and tail (x5)
//     # # # . . #     head and back
//     . # # # # .     body
//     . # . . # .     front foot (x1) and back foot (x4)
//
// Body row is static in every pose
constexpr const char *kBodyRow = ".####.";

// Top row variants: ears at x0/x2, tail at x5.
constexpr const char *kTopNeutral = "#.#..#";  // tail straight up
constexpr const char *kTopTailFwd = "#.#.#.";  // tail swung over the back
constexpr const char *kTopEarBack = ".##..#";  // front ear flattened

// Second row from the top: the eyes, at x0 and x2, which both go dark for a
// blink. Everything else on the row stays put.
constexpr const char *kTop2Neutral = "###..#";
constexpr const char *kTop2Blink = ".#...#";

// Bottom row variants: front foot near the head, back foot near the tail.
constexpr const char *kFeetStand = ".#..#."; // standing square
constexpr const char *kFeetBack  = "#...#.";  // front foot stepped back
constexpr const char *kFeetFwd   = ".#...#";   // back foot stepped out
constexpr const char *kFeetSplay = "#....#"; // braced, both feet planted wide
constexpr const char *kFeetTuck  = "......";  // pulled in -- airborne

// Mid-turn frame: the cat hunched into a solid block while it swaps which
// way it faces. Left-right symmetric on purpose -- it has to read the same
// whichever facing it is coming from or going to, so the mirror lands on a
// shape that has no facing of its own.
constexpr const char *kTurnTop   = "..##..";
constexpr const char *kTurnUpper = "..##..";
constexpr const char *kTurnBody  = "..##..";
constexpr const char *kTurnFeet  = "..##..";

// Foot pattern cycle for the idle shuffle, one step per kShuffleStepS.
constexpr float kShuffleStepS = 0.15f;
constexpr std::array kShuffleCycle = {kFeetStand, kFeetBack, kFeetStand, kFeetFwd};

// Wraps an angle difference into [-pi, pi], so comparisons against the
// 45-degree threshold don't break when the two angles straddle the seam.
float wrapPi(float angle) {
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle < -kPi) angle += 2.0f * kPi;
    return angle;
}

// True for the first half of every `period`-long window, i.e. a square wave.
// Used to blink sprite parts on and off during a fidget.
bool alternates(float t, float period) {
    return fmodf(t, 2.0f * period) < period;
}

// Maps a canonical sprite cell (cx, cy) to its offset inside the rotated
// footprint, for `rotation` quarter turns clockwise. In screen axes (x right,
// y down) one clockwise quarter turn is (x, y) -> (height - 1 - y, x), which
// sends the bottom row -- the feet -- to the left edge.
void rotateCell(int cx, int cy, int rotation, int &dx, int &dy) {
    switch (rotation) {
        case 1:  dx = kCatH - 1 - cy; dy = cx;               break;
        case 2:  dx = kCatW - 1 - cx; dy = kCatH - 1 - cy;   break;
        case 3:  dx = cy;             dy = kCatW - 1 - cx;   break;
        default: dx = cx;             dy = cy;               break;
    }
}

// Quantizes a float coordinate to a pixel, but only lets the result move once
// the input is kPixelHysteresis past the halfway point, so a coordinate
// jittering around a boundary keeps the previous pixel.
int snapWithHysteresis(float value, int current) {
    if (value > static_cast<float>(current) + 0.5f + kPixelHysteresis ||
        value < static_cast<float>(current) - 0.5f - kPixelHysteresis) {
        return static_cast<int>(floorf(value + 0.5f));
    }
    return current;
}

// Applies one wall contact to the velocity. normalVel is the component along
// the wall's own axis, tangentVel the one across it, and normal is +1 when the
// wall pushes the cat in the positive direction of that axis. A real impact
// bounces and scrubs off sideways speed; anything gentler is the cat simply
// leaning on the wall, which just cancels the motion into it and leaves the
// sliding alone.
void applyWallContact(float &normalVel, float &tangentVel, float normal) {
    float into = -normalVel * normal;
    if (into <= 0.0f) return; // already moving away from this wall
    if (into > kImpactSpeed) {
        normalVel = -normalVel * kRestitution;
        tangentVel *= kWallFriction;
    } else {
        normalVel = 0.0f;
    }
}

int clampInt(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} // namespace

CatFace::CatFace() {
    pos_ = {FACE_WIDTH / 2.0f, FACE_HEIGHT / 2.0f};
    originX_ = static_cast<int>(floorf(pos_.x - kCatW / 2.0f + 0.5f));
    originY_ = static_cast<int>(floorf(pos_.y - kCatH / 2.0f + 0.5f));
    idleGapS_ = randomRange(kIdleGapMinS, kIdleGapMaxS);
    walkGapS_ = randomRange(kWalkGapMinS, kWalkGapMaxS);
    blinkGapS_ = nextBlinkGap();
}

uint32_t CatFace::nextRandom() {
    randomState_ ^= randomState_ << 13;
    randomState_ ^= randomState_ >> 17;
    randomState_ ^= randomState_ << 5;
    return randomState_;
}

float CatFace::randomRange(float lo, float hi) {
    // Top 24 bits only: the low bits of an xorshift word are the weakest, and
    // 24 is plenty of resolution for a fidget delay.
    float unit = static_cast<float>(nextRandom() >> 8) / static_cast<float>(1 << 24);
    return lo + unit * (hi - lo);
}

float CatFace::footprintW() const {
    return static_cast<float>((rotation_ % 2 == 0) ? kCatW : kCatH);
}

float CatFace::footprintH() const {
    return static_cast<float>((rotation_ % 2 == 0) ? kCatH : kCatW);
}

void CatFace::feedImu(const ImuSample &sample) {
    float ax = sample.accel[0] / 1000.0f;
    float ay = sample.accel[1] / 1000.0f;
    gravity_.x = -ax * kGravityScale;
    gravity_.y = ay * kGravityScale;
}

void CatFace::beginReorient(int direction) {
    reorienting_ = true;
    reorientApplied_ = false;
    reorientT_ = 0.0f;
    reorientDirection_ = direction;
    snapTimer_ = 0.0f;

    // Push off the ground into the turn: a hop straight against gravity plus
    // a scoot along the wall toward the new downhill side. Only when there's
    // actually ground to push against -- a cat mid-fall just turns.
    float gMag = sqrtf(gravity_.x * gravity_.x + gravity_.y * gravity_.y);
    if (onGround_ && gMag > 0.0f) {
        Vec2 up{-gravity_.x / gMag, -gravity_.y / gMag};
        Vec2 along{-up.y, up.x}; // up rotated a quarter turn clockwise
        vel_.x += up.x * kHopSpeed - along.x * kScootSpeed * static_cast<float>(direction);
        vel_.y += up.y * kHopSpeed - along.y * kScootSpeed * static_cast<float>(direction);
    }
}

void CatFace::updateOrientation(float dt) {
    if (reorienting_) {
        reorientT_ += dt / kReorientS;
        if (!reorientApplied_ && reorientT_ >= 0.5f) {
            rotation_ = (rotation_ + reorientDirection_ + 4) % 4;
            reorientApplied_ = true;
            // Turning a quarter also swaps which end of the body is downhill,
            // so the facing is re-picked here rather than left to trigger a
            // second, separate flip animation the moment this one ends. The
            // sprite is already mid-change, so it costs nothing visually.
            adoptDownhillFacing();
            // The footprint just went from 6x4 to 4x6 (or back), so the cat
            // may now be poking through a wall it was clear of a moment ago.
            resolveWalls();
        }
        if (reorientT_ >= 1.0f) {
            reorienting_ = false;
        }
        return;
    }

    // How far gravity has drifted from the direction the cat's feet point.
    // Feet point at +y for rotation 0 and advance a quarter turn from there,
    // so the feet direction is (rotation + 1) * 90 degrees.
    float gravityAngle = atan2f(gravity_.y, gravity_.x);
    float feetAngle = kHalfPi * static_cast<float>(rotation_ + 1);
    float lean = wrapPi(gravityAngle - feetAngle);

    // Comfortably inside 45 degrees this is the closest quarter turn there is,
    // so the cat simply leans and nothing happens.
    float excess = fabsf(lean) - kQuarterPi;
    if (excess <= kSettleMarginRad) {
        snapTimer_ = 0.0f;
        return;
    }

    // Past 45 degrees a closer quarter turn does exist, so the cat will step
    // around either way -- how urgently is all that differs. A big lean is a
    // board actively being tilted and gets the short dwell; a lean only just
    // past the tie point is a pose that merely wants tidying up, and waits
    // long enough that noise around the threshold can never trigger it.
    snapDirection_ = (lean > 0.0f) ? 1 : -1;
    snapTimer_ += dt;
    float dwell = (excess > kSnapMarginRad) ? kSnapDwellS : kSettleDwellS;
    if (snapTimer_ >= dwell) {
        // One quadrant at a time even when gravity is a full half turn away,
        // so flipping the board upside down has the cat scrambling around in
        // two visible steps rather than instantly appearing inverted.
        beginReorient(snapDirection_);
    }
}

float CatFace::downhillAlignment() const {
    // The body's head-to-tail axis is a quarter turn off the feet direction,
    // so it lies at rotation_ * 90 degrees -- and points the other way when
    // the sprite is mirrored.
    float axisAngle = kHalfPi * static_cast<float>(rotation_);
    float ax = cosf(axisAngle);
    float ay = sinf(axisAngle);
    float along = gravity_.x * ax + gravity_.y * ay;
    return facingFlipped_ ? -along : along;
}

void CatFace::adoptDownhillFacing() {
    // Snap the facing to whichever way puts the head uphill, with no
    // animation. Only used when the sprite is already changing anyway (mid
    // step-around); a slope too shallow to call is left alone.
    float axisAngle = kHalfPi * static_cast<float>(rotation_);
    float along = gravity_.x * cosf(axisAngle) + gravity_.y * sinf(axisAngle);
    if (fabsf(along) > kFacingDeadband) {
        facingFlipped_ = (along < 0.0f);
    }
    flipping_ = false;
    facingTimer_ = 0.0f;
}

void CatFace::beginFlip() {
    flipping_ = true;
    flipApplied_ = false;
    flipT_ = 0.0f;
    facingTimer_ = 0.0f;
}

CatFace::Vec2 CatFace::feetDir() const {
    // Feet point at +y for rotation 0 and advance a quarter turn from there.
    float angle = kHalfPi * static_cast<float>(rotation_ + 1);
    return {cosf(angle), sinf(angle)};
}

CatFace::Vec2 CatFace::forwardDir() const {
    // The body axis runs head-to-tail at rotation * 90 degrees when the
    // sprite is drawn unmirrored, so the head -- and therefore forward -- is
    // the other way along it.
    float axisAngle = kHalfPi * static_cast<float>(rotation_);
    float sign = facingFlipped_ ? 1.0f : -1.0f;
    return {cosf(axisAngle) * sign, sinf(axisAngle) * sign};
}

int CatFace::stepsAhead() const {
    Vec2 forward = forwardDir();
    float hw = footprintW() * 0.5f;
    float hh = footprintH() * 0.5f;

    float room;
    if (forward.x > 0.5f) room = FACE_WIDTH - hw - pos_.x;
    else if (forward.x < -0.5f) room = pos_.x - hw;
    else if (forward.y > 0.5f) room = FACE_HEIGHT - hh - pos_.y;
    else room = pos_.y - hh;

    // Rounded, not truncated, and this matters more than it looks. The cat's
    // position is float and gravity nudges it a fraction of a pixel off the
    // grid between walks, so a gap that is visibly one whole pixel measures
    // 0.94. Truncating calls that no room at all -- which is why the cat
    // would stop a pixel short of the wall, and why it would sometimes turn
    // on the spot without taking a step, having been handed a walk of zero
    // pixels. Rounding reads the gap the way the display draws it.
    int steps = static_cast<int>(lroundf(room));
    return steps < 0 ? 0 : steps;
}

void CatFace::updateWalk(float dt) {
    Vec2 forward = forwardDir();
    float step = kWalkSpeed * dt;
    if (step > walkRemaining_) step = walkRemaining_;

    // Measured as distance along the walking direction, so a wall clamping
    // the move shows up as the cat simply not having got anywhere.
    float before = pos_.x * forward.x + pos_.y * forward.y;
    pos_.x += forward.x * step;
    pos_.y += forward.y * step;
    resolveWalls();
    float travelled = (pos_.x * forward.x + pos_.y * forward.y) - before;

    walkRemaining_ -= step;

    bool blocked = travelled < step * 0.5f;
    if (!blocked && walkRemaining_ > 0.0f) return;

    idleEvent_ = Idle::None;
    walkGapS_ = randomRange(kWalkGapMinS, kWalkGapMaxS);
    walkRemaining_ = 0.0f;

    // Turn around on arriving at the wall, not merely on being stopped by it:
    // a walk that ends exactly flush with the edge is not "blocked", so
    // without the second test the cat would stand nose to the wall until the
    // next outing, then spend it turning on the spot having gone nowhere.
    // Either way the turn is what aims the following walk back the other way,
    // since the cat only ever walks head-first.
    if (blocked || stepsAhead() <= 0) beginFlip();
}

void CatFace::updateFacing(float dt) {
    if (flipping_) {
        flipT_ += dt / kFlipS;
        if (!flipApplied_ && flipT_ >= 0.5f) {
            facingFlipped_ = !facingFlipped_;
            flipApplied_ = true;
        }
        if (flipT_ >= 1.0f) flipping_ = false;
        return;
    }

    // A step-around re-picks the facing itself, so nothing to do mid-turn.
    if (reorienting_) {
        facingTimer_ = 0.0f;
        return;
    }

    // Only turn around when gravity is clearly pulling toward the head end,
    // and has been for a moment -- anything shallower is either a cat
    // standing level or noise.
    if (downhillAlignment() >= -kFacingDeadband) {
        facingTimer_ = 0.0f;
        return;
    }

    facingTimer_ += dt;
    if (facingTimer_ >= kFacingDwellS) {
        beginFlip();
    }
}

void CatFace::resolveWalls() {
    float hw = footprintW() * 0.5f;
    float hh = footprintH() * 0.5f;

    if (pos_.x < hw) {
        pos_.x = hw;
        applyWallContact(vel_.x, vel_.y, 1.0f);
    } else if (pos_.x > FACE_WIDTH - hw) {
        pos_.x = FACE_WIDTH - hw;
        applyWallContact(vel_.x, vel_.y, -1.0f);
    }

    if (pos_.y < hh) {
        pos_.y = hh;
        applyWallContact(vel_.y, vel_.x, 1.0f);
    } else if (pos_.y > FACE_HEIGHT - hh) {
        pos_.y = FACE_HEIGHT - hh;
        applyWallContact(vel_.y, vel_.x, -1.0f);
    }
}

void CatFace::updatePhysics(float dt) {
    // Gravity is applied unconditionally, however weak it is. An earlier
    // version treated a near-zero X/Y reading as "board lying flat" and
    // pinned the cat in place -- but that also declared it to be standing,
    // wherever it happened to be, so any transient that briefly cancelled
    // in-plane gravity (a swing, being lifted, the board leaning back) left
    // the cat frozen in mid-air against whatever edge it was passing, idly
    // flicking its tail. Letting accelerometer noise jostle it around while
    // the board is flat is both more honest and better looking.
    Vec2 pull = gravity_;
    if (onGround_) {
        // Static friction, applied by letting only the part of gravity the
        // feet cannot hold through. Split it into the push into the ground
        // and the drag along it; anything up to kFootGrip times the former is
        // simply gripped, and only the excess accelerates the cat downhill.
        Vec2 normal = feetDir();
        Vec2 along{-normal.y, normal.x};
        float intoGround = pull.x * normal.x + pull.y * normal.y;
        float alongGround = pull.x * along.x + pull.y * along.y;
        float grip = kFootGrip * fabsf(intoGround);
        float excess = fabsf(alongGround) - grip;
        alongGround = (excess > 0.0f) ? copysignf(excess, alongGround) : 0.0f;
        pull = {normal.x * intoGround + along.x * alongGround,
                normal.y * intoGround + along.y * alongGround};
    }

    vel_.x += pull.x * dt;
    vel_.y += pull.y * dt;
    if (onGround_) {
        // Standing friction. Applied to both axes: the component into the
        // wall is about to be clamped away anyway, and the one along it
        // is the slide this is meant to slow down.
        float damp = expf(-kGroundDrag * dt);
        vel_.x *= damp;
        vel_.y *= damp;
    }

    pos_.x += vel_.x * dt;
    pos_.y += vel_.y * dt;
    resolveWalls();

    // Standing means the wall the cat's own feet point at is right there. It
    // is deliberately a fact about the feet and not about gravity: asking
    // instead which wall gravity points into gets both ends of this wrong.
    // With the board flat, gravity is nothing but noise pointing in a random
    // direction, so the cat is hardly ever touching "the" wall and hangs in
    // the falling pose indefinitely -- the flying cat. And a cat propped
    // against a wall by its tail satisfied it just as well as one on its
    // feet, so it collected ground friction while balanced on its tail.
    float hw = footprintW() * 0.5f;
    float hh = footprintH() * 0.5f;
    Vec2 feet = feetDir();
    if (feet.y > 0.5f) {
        onGround_ = pos_.y >= FACE_HEIGHT - hh - kContactEpsilon;
    } else if (feet.y < -0.5f) {
        onGround_ = pos_.y <= hh + kContactEpsilon;
    } else if (feet.x > 0.5f) {
        onGround_ = pos_.x >= FACE_WIDTH - hw - kContactEpsilon;
    } else {
        onGround_ = pos_.x <= hw + kContactEpsilon;
    }
}

float CatFace::nextBlinkGap() {
    // Strictly positive: logf(0) is not a number, and a unit of exactly 1
    // simply yields the shortest gap the floor allows.
    float unit = static_cast<float>((nextRandom() >> 8) + 1) /
                 static_cast<float>((1 << 24) + 1);
    return kBlinkMinGapS - kBlinkMeanGapS * logf(unit);
}

void CatFace::updateBlink(float dt) {
    // Only runs while the everyday standing sprite is the one on screen. The
    // step-around, the turn and the falling pose all replace the eye row
    // anyway, so a blink counted down underneath them would just be time
    // thrown away -- and the rate is meant to be what you actually see.
    if (reorienting_ || flipping_ || !onGround_) return;

    if (blinking_) {
        blinkT_ += dt;
        if (blinkT_ >= kBlinkS) {
            blinking_ = false;
            blinkGapS_ = nextBlinkGap();
        }
        return;
    }

    blinkGapS_ -= dt;
    if (blinkGapS_ <= 0.0f) {
        blinking_ = true;
        blinkT_ = 0.0f;
    }
}

void CatFace::updateAnimation(float dt) {
    updateBlink(dt);

    float speed = sqrtf(vel_.x * vel_.x + vel_.y * vel_.y);
    if (reorienting_ || !onGround_ || speed > kSettleSpeed) {
        // Busy: drop any fidget in progress and make the next one wait until
        // the cat has been still for a while again.
        restTimer_ = 0.0f;
        idleEvent_ = Idle::None;
        idleGapS_ = randomRange(kIdleGapMinS, kIdleGapMaxS);
        return;
    }

    restTimer_ += dt;
    if (restTimer_ < kSettleS) return;

    if (idleEvent_ == Idle::None) {
        // Walking is scheduled on a clock of its own rather than as a share of
        // the fidget rotation. As one entry among several its frequency was
        // capped by the rotation itself -- it could never exceed the rate the
        // whole rotation ran at, however heavily it was weighted -- and
        // raising it meant dragging the tail flicks and ear twitches up with
        // it. On its own gap the two tune independently.
        //
        // On anything worth calling a slope the cat does not wander at all: it
        // has just turned to face uphill, and strolling off along a gradient
        // it is braced against would undo that. It only fidgets there.
        if (fabsf(downhillAlignment()) < kFacingDeadband) {
            walkGapS_ -= dt;
            if (walkGapS_ <= 0.0f) {
                // Only ever whole pixels, and never more than there is floor
                // left -- a walk trimmed to fit still ends flush with the
                // wall, which is what triggers the turn.
                int room = stepsAhead();
                int wanted = (nextRandom() & 1u) ? 1 : 2;
                if (wanted > room) wanted = room;
                if (wanted <= 0) {
                    // Already nose to the wall, so there is no walk to take:
                    // turn around now rather than burning the outing standing
                    // still, which is what made it look like a pointless spin.
                    walkGapS_ = randomRange(kWalkGapMinS, kWalkGapMaxS);
                    beginFlip();
                } else {
                    idleEvent_ = Idle::Walk;
                    idleT_ = 0.0f;
                    walkRemaining_ = static_cast<float>(wanted);
                }
                return;
            }
        }

        idleGapS_ -= dt;
        if (idleGapS_ <= 0.0f) {
            constexpr std::array kFidgets = {Idle::TailFlick, Idle::EarTwitch, Idle::Shuffle};
            idleEvent_ = kFidgets[nextRandom() % kFidgets.size()];
            idleT_ = 0.0f;
        }
        return;
    }

    idleT_ += dt;

    // A walk ends when the ground runs out or the pixels do, not on a clock.
    if (idleEvent_ == Idle::Walk) {
        updateWalk(dt);
        return;
    }

    float duration = kTailFlickS;
    if (idleEvent_ == Idle::EarTwitch) duration = kEarTwitchS;
    if (idleEvent_ == Idle::Shuffle) duration = kShuffleS;

    if (idleT_ >= duration) {
        idleEvent_ = Idle::None;
        idleGapS_ = randomRange(kIdleGapMinS, kIdleGapMaxS);
    }
}

CatFace::Pose CatFace::currentPose() const {
    if (reorienting_) {
        // First half is the braced push-off, second half the settling step,
        // with the tail over the back throughout for balance.
        if (reorientT_ < 0.5f) return {kTopTailFwd, kTop2Neutral, kBodyRow, kFeetSplay};
        return {kTopTailFwd, kTop2Neutral, kBodyRow,
                (reorientDirection_ > 0) ? kFeetFwd : kFeetBack};
    }

    if (flipping_) {
        // Hunched into a symmetric block while it turns end for end.
        return {kTurnTop, kTurnUpper, kTurnBody, kTurnFeet};
    }

    if (!onGround_) {
        // Falling: legs pulled in, tail streaming, and eyes open -- a cat
        // in the air is looking where it is going.
        return {kTopTailFwd, kTop2Neutral, kBodyRow, kFeetTuck};
    }

    // Blinking rides on top of whatever fidget is playing, rather than being
    // one of them, so a blink can land partway through a tail flick.
    const char *eyes = blinking_ ? kTop2Blink : kTop2Neutral;

    switch (idleEvent_) {
        case Idle::TailFlick:
            return {alternates(idleT_, 0.12f) ? kTopTailFwd : kTopNeutral,
                    eyes, kBodyRow, kFeetStand};
        case Idle::EarTwitch:
            return {alternates(idleT_, 0.1f) ? kTopEarBack : kTopNeutral,
                    eyes, kBodyRow, kFeetStand};
        case Idle::Shuffle:
        case Idle::Walk: {
            // Walking reuses the shuffle's footfalls; the only difference is
            // that the cat is actually going somewhere while they play.
            int step = static_cast<int>(idleT_ / kShuffleStepS);
            return {kTopNeutral, eyes, kBodyRow,
                    kShuffleCycle[step % kShuffleCycle.size()]};
        }
        default:
            return {kTopNeutral, eyes, kBodyRow, kFeetStand};
    }
}

FaceFrame CatFace::getFrame(uint32_t dtUs) {
    float dt = dtUs / 1000000.0f;

    updateOrientation(dt);
    updateFacing(dt);
    updatePhysics(dt);
    updateAnimation(dt);

    int w = static_cast<int>(footprintW());
    int h = static_cast<int>(footprintH());

    // Float position quantized to a pixel origin exactly once, here: the
    // sprite is pixel art, so it's either on a pixel or it isn't, and nothing
    // downstream ever sees a fractional coverage.
    originX_ = snapWithHysteresis(pos_.x - footprintW() * 0.5f, originX_);
    originY_ = snapWithHysteresis(pos_.y - footprintH() * 0.5f, originY_);
    originX_ = clampInt(originX_, 0, FACE_WIDTH - w);
    originY_ = clampInt(originY_, 0, FACE_HEIGHT - h);

    Pose pose = currentPose();
    const char *rows[kCatH] = {pose.top, pose.upper, pose.body, pose.feet};

    // Rendered unflipped, matching HeartFace and FluidFace -- and in any case
    // the cat's own orientation is derived from the gravity vector rather than
    // baked into the art, so which way the board is worn only decides which
    // way the cat happens to face, never whether it stands on its feet.
    FaceFrame frame{};
    for (int cy = 0; cy < kCatH; cy++) {
        for (int cx = 0; cx < kCatW; cx++) {
            if (rows[cy][cx] != '#') continue;

            // Mirroring end for end is what turns the cat around; the feet
            // row is untouched, so it stays standing either way.
            int cxDraw = facingFlipped_ ? (kCatW - 1 - cx) : cx;
            int dx, dy;
            rotateCell(cxDraw, cy, rotation_, dx, dy);
            int x = originX_ + dx;
            int y = originY_ + dy;
            if (x < 0 || x >= FACE_WIDTH || y < 0 || y >= FACE_HEIGHT) continue;

            frame.pixels[x][y] = Color{
                LED_BRIGHTNESS * kCatColor.r,
                LED_BRIGHTNESS * kCatColor.g,
                LED_BRIGHTNESS * kCatColor.b,
            };
        }
    }
    return frame;
}
