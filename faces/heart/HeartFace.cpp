/*****************************************************************************
* | File      	:   HeartFace.cpp
* | Author      :   Leonid, https://www.youtube.com/@nocomake
* | Assisted by :   Claude AI code generation
* | License     :   MIT
* | Function    :   Face implementation rendering a small rigid-body physics
*                    simulation of a heart, falling under tilt-driven gravity
*                    and bouncing off the walls.
******************************************************************************/
#include "HeartFace.h"

#include <cmath>

extern "C" {
#include "WS2812.h" // for LED_BRIGHTNESS, the hardware safety brightness cap
}

namespace {

// Converts accelerometer tilt (in g) into gravity units. Same axes and sign
// convention as FluidFace's gravityX_/gravityY_ (and same unflipped render
// orientation below), per request that this should fall "just like water in
// FluidFace". Raise for a snappier reaction.
constexpr float kGravityScale = 108.0f;

// Bounce keeps this fraction of the incoming normal-relative speed, so each
// wall hit loses a bit of energy rather than bouncing perfectly or going
// dead.
constexpr float kRestitution = 0.7f;

// Antialiasing supersamples: kSupersample x kSupersample sub-points per
// pixel are point-in-polygon tested and averaged into a coverage fraction.
constexpr int kSupersample = 4;

// Perceived LED brightness isn't linear in coverage fraction (see
// FlappyFace's identical reasoning for shapeCoverage) -- a sliver of
// coverage still reads as clearly lit unless pushed down hard first. A
// cubic curve damps low coverage much harder than FlappyFace's square while
// still leaving full coverage (1.0) completely undamped.
constexpr float kCoverageGamma = 3.0f;

float shapeCoverage(float coverage) {
    return powf(coverage, kCoverageGamma);
}

constexpr Color kHeartColor = {1.0f, 0.0f, 0.0f};

} // namespace

HeartFace::HeartFace() {
    // Heart polygon, hand-tuned directly in world units (1 unit = 1 pixel;
    // roughly a 6x6 bounding box, "approximately 6x6 pixels"). +y is down
    // (see Face.h's coordinate comment), so the point is the large positive
    // y and the two lobes are up top at negative y. Edit these directly to
    // reshape the heart -- kNumVertices in HeartFace.h must match the count
    // here. Not yet centered on the center of mass; that's computed and
    // applied below.
    std::array<Vec2, kNumVertices> raw = {{
        {0.0f, 0.2f},    // top notch -- deepened so the crease reads clearly at low res
        {0.5f, -1.6f},   // narrowed back down -- the wider version made the crease too broad
        {1.3f, -2.5f},
        {2.2f, -2.4f},
        {2.9f, -1.6f},
        {3.0f, -0.6f},
        {2.5f, 0.5f},
        {1.7f, 1.7f},
        {0.0f, 3.3f},    // bottom point
        {-1.7f, 1.7f},
        {-2.5f, 0.5f},
        {-3.0f, -0.6f},
        {-2.9f, -1.6f},
        {-2.2f, -2.4f},
        {-1.3f, -2.5f},
        {-0.5f, -1.6f},  // mirror of the shoulder above
    }};

    // Signed area, centroid, and second moment of area about the origin via
    // the standard polygon-integral (shoelace-family) formulas, assuming
    // uniform density -- then parallel-axis-shifted to get mass (= area,
    // unit density) and rotational inertia about the centroid. Everything
    // here works out winding-direction-agnostic since area, the centroid
    // numerator, and the moment sum all flip sign together if the sampled
    // curve happens to wind clockwise instead of counter-clockwise.
    float area2 = 0.0f, cxNum = 0.0f, cyNum = 0.0f, polarSum = 0.0f;
    for (int i = 0; i < kNumVertices; i++) {
        const Vec2 &a = raw[i];
        const Vec2 &b = raw[(i + 1) % kNumVertices];
        float cross = a.x * b.y - b.x * a.y;
        area2 += cross;
        cxNum += (a.x + b.x) * cross;
        cyNum += (a.y + b.y) * cross;
        polarSum += cross * (a.x * a.x + a.x * b.x + b.x * b.x + a.y * a.y + a.y * b.y + b.y * b.y);
    }
    float area = area2 * 0.5f;
    float cx = cxNum / (3.0f * area2);
    float cy = cyNum / (3.0f * area2);
    float polarMoment = polarSum / 12.0f; // about the origin
    float signedInertia = polarMoment - area * (cx * cx + cy * cy); // parallel axis -> about centroid

    mass_ = std::fabs(area);
    momentOfInertia_ = std::fabs(signedInertia);

    for (int i = 0; i < kNumVertices; i++) {
        localVerts_[i] = {raw[i].x - cx, raw[i].y - cy};
    }

    pos_ = {FACE_WIDTH / 2.0f, FACE_HEIGHT / 2.0f};
}

void HeartFace::feedImu(const ImuSample &sample) {
    float ax = sample.accel[0] / 1000.0f;
    float ay = sample.accel[1] / 1000.0f;
    gravity_.x = -ax * kGravityScale;
    gravity_.y = ay * kGravityScale;
}

void HeartFace::updateWorldVerts() {
    float c = cosf(theta_);
    float s = sinf(theta_);
    for (int i = 0; i < kNumVertices; i++) {
        const Vec2 &lv = localVerts_[i];
        worldVerts_[i] = {
            pos_.x + c * lv.x - s * lv.y,
            pos_.y + s * lv.x + c * lv.y,
        };
    }
}

void HeartFace::resolveWallCollision(const Vec2 &normal, const Vec2 &wallPoint) {
    updateWorldVerts();

    // Find the vertex penetrating this wall the most, if any. Resolving one
    // contact per wall per frame is a simplification (a true multi-contact
    // solver would handle simultaneous corner hits more precisely), but is
    // stable and cheap enough for this small polygon at this scale.
    int deepestIndex = -1;
    float deepestSigned = 0.0f;
    for (int i = 0; i < kNumVertices; i++) {
        float signedDist = (worldVerts_[i].x - wallPoint.x) * normal.x + (worldVerts_[i].y - wallPoint.y) * normal.y;
        if (signedDist < deepestSigned) {
            deepestSigned = signedDist;
            deepestIndex = i;
        }
    }
    if (deepestIndex < 0) return; // clear of this wall

    Vec2 contact = worldVerts_[deepestIndex];
    Vec2 r{contact.x - pos_.x, contact.y - pos_.y};

    // Velocity of the material point at the contact, including the
    // rotational contribution (2D omega x r = omega * (-r.y, r.x)).
    Vec2 vAtContact{
        vel_.x - omega_ * r.y,
        vel_.y + omega_ * r.x,
    };
    float vn = vAtContact.x * normal.x + vAtContact.y * normal.y;

    if (vn < 0.0f) {
        // Standard impulse-based collision response against an immovable
        // wall, with restitution and the rotational (angular) term that
        // makes off-center impacts realistically induce spin.
        float rCrossN = r.x * normal.y - r.y * normal.x;
        float invMass = 1.0f / mass_;
        float invI = 1.0f / momentOfInertia_;
        float denom = invMass + rCrossN * rCrossN * invI;
        float j = -(1.0f + kRestitution) * vn / denom;
        Vec2 impulse{j * normal.x, j * normal.y};

        vel_.x += impulse.x * invMass;
        vel_.y += impulse.y * invMass;
        omega_ += invI * (r.x * impulse.y - r.y * impulse.x);
    }

    // Positional correction: push the whole body back out along the wall
    // normal by the penetration depth so it doesn't visibly sink into the
    // wall over successive frames.
    float penetration = -deepestSigned;
    pos_.x += normal.x * penetration;
    pos_.y += normal.y * penetration;
}

bool HeartFace::pointInPolygon(float px, float py, const std::array<Vec2, kNumVertices> &verts) {
    bool inside = false;
    for (int i = 0, j = kNumVertices - 1; i < kNumVertices; j = i++) {
        const Vec2 &vi = verts[i];
        const Vec2 &vj = verts[j];
        bool crosses = (vi.y > py) != (vj.y > py);
        if (crosses && px < (vj.x - vi.x) * (py - vi.y) / (vj.y - vi.y) + vi.x) {
            inside = !inside;
        }
    }
    return inside;
}

FaceFrame HeartFace::getFrame(uint32_t dtUs) {
    float dt = dtUs / 1000000.0f;

    vel_.x += gravity_.x * dt;
    vel_.y += gravity_.y * dt;
    pos_.x += vel_.x * dt;
    pos_.y += vel_.y * dt;
    theta_ += omega_ * dt;

    resolveWallCollision({1.0f, 0.0f}, {0.0f, 0.0f});                                  // left wall, x = 0
    resolveWallCollision({-1.0f, 0.0f}, {static_cast<float>(FACE_WIDTH), 0.0f});        // right wall, x = FACE_WIDTH
    resolveWallCollision({0.0f, 1.0f}, {0.0f, 0.0f});                                   // top wall, y = 0
    resolveWallCollision({0.0f, -1.0f}, {0.0f, static_cast<float>(FACE_HEIGHT)});       // bottom wall, y = FACE_HEIGHT

    updateWorldVerts();

    // No 180-degree flip here (unlike MinecraftFace/FlappyFace/SnakeFace) --
    // this renders in the same unflipped orientation as FluidFace, per
    // request that gravity should behave "just like water in FluidFace".
    FaceFrame frame{};
    for (int y = 0; y < FACE_HEIGHT; y++) {
        for (int x = 0; x < FACE_WIDTH; x++) {
            int insideCount = 0;
            for (int sy = 0; sy < kSupersample; sy++) {
                for (int sx = 0; sx < kSupersample; sx++) {
                    float px = static_cast<float>(x) + (static_cast<float>(sx) + 0.5f) / static_cast<float>(kSupersample);
                    float py = static_cast<float>(y) + (static_cast<float>(sy) + 0.5f) / static_cast<float>(kSupersample);
                    if (pointInPolygon(px, py, worldVerts_)) insideCount++;
                }
            }
            float coverage = static_cast<float>(insideCount) / static_cast<float>(kSupersample * kSupersample);
            float shaped = shapeCoverage(coverage);
            frame.pixels[x][y] = Color{
                LED_BRIGHTNESS * kHeartColor.r * shaped,
                LED_BRIGHTNESS * kHeartColor.g * shaped,
                LED_BRIGHTNESS * kHeartColor.b * shaped,
            };
        }
    }
    return frame;
}
