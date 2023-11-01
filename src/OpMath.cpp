// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpMath.h"
#include <cmath>

bool OpRoots::replaceClosest(float root) {
    size_t best = OpMax;
    float bestDistance = OpInfinity;
    for (size_t index = 0; index < count; ++index) {
        float distance = fabsf(root - roots[index]);
        if (bestDistance < distance)
            continue;
        bestDistance = distance;
        best = index;
    }
    if (best >= count || bestDistance >= .5)
        return false;
//    assert(bestDistance <= OpEpsilon * 2);    // !!! number may be large (see example dean 2)
    roots[best] = root;
    return true;
}

// if t is nearly end of range, make it end of range
// motivation for this is test cubics_d, which generates yExtrema very nearly equal to 1.
// 'interior' is only used for extrema and inflections
OpRoots OpRoots::keepInteriorTs(float start, float end) {
    (void) keepValidTs(start, end);
    int interiorRoots = 0;
    for (unsigned index = 0; index < count; ++index) {
        float tValue = roots[index];
        if (start >= tValue - OpEpsilon || tValue + OpEpsilon >= end)
            continue;
        roots[interiorRoots++] = tValue;
    }
    count = interiorRoots;
    return *this;
}

OpRoots OpRoots::keepValidTs(float start, float end) {
    size_t foundRoots = 0;
    for (unsigned index = 0; index < count; ++index) {
        float tValue = roots[index];
        if (OpMath::IsNaN(tValue) || start > tValue || tValue > end)
            continue;
        if (tValue < start + OpEpsilon)
            tValue = start;
        else if (tValue > end - OpEpsilon)
            tValue = end;
        for (size_t idx2 = 0; idx2 < foundRoots; ++idx2) {
            if (roots[idx2] == tValue) {
                goto notUnique;
            }
        }
        roots[foundRoots++] = tValue;
    notUnique:
        ;
    }
    count = foundRoots;
    return *this;
}

// move zero and one to the front so they get processed first
void OpRoots::prioritize01() {
    if (count <= 1)
        return;
    sort(); // zero, if present, is moved to front
    if (roots[count - 1] != 1)
        return; 
    if (roots[0] != 0) {
        std::swap(roots[0], roots[count - 1]);  // move 1 to front
        return;
    }
    if (count > 2)
        std::swap(roots[1], roots[count - 1]);  // zero in front, move 1 to second position
}

OpVector OpVector::normalize(bool* overflow) {
    float len = length();
    *overflow = false;
    if (!OpMath::IsFinite(len))
        *overflow = true;
    else if (len) {
        float inverseLength = 1 / len;
        dx *= inverseLength;
        dy *= inverseLength;
    }
    return *this;
}

bool OpVector::isFinite() const {
    return OpMath::IsFinite(dx) && OpMath::IsFinite(dy);
}

// Use the axis with the greatest change to decide if mid is between start and end.
// While this doesn't work in general, the callers are dealing with coincident curves
// which may be close, but not on top of each other. When they are axis-aligned, the
// larger provides a better metric of whether multiple curves overlap.
bool OpPoint::Between(OpPoint start, OpPoint mid, OpPoint end) {
    OpVector scope = end - start;
    XyChoice xy = scope.larger();
    return OpMath::Between(start.choice(xy), mid.choice(xy), end.choice(xy));
}

bool OpPoint::isFinite() const {
#if OP_DEBUG
    if (OpMath::IsNaN(x)) {
        int32_t xBits = OpDebugFloatToBits(x);
        OP_ASSERT(!(xBits & 1));
    }
    if (OpMath::IsNaN(y)) {
        int32_t yBits = OpDebugFloatToBits(y);
        OP_ASSERT(!(yBits & 1));
    }
#endif
    return *this * 0 == OpPoint(0, 0);
    // return OpMath::IsFinite(x) && OpMath::IsFinite(y);
}

void OpPoint::pin(const OpPoint a, const OpPoint b) {
    x = OpMath::Pin(a.x, x, b.x);
    y = OpMath::Pin(a.y, y, b.y);
}

void OpPoint::pin(const OpRect& r) {
    x = OpMath::Pin(r.left, x, r.right);
    y = OpMath::Pin(r.top, y, r.bottom);
}

bool OpRect::isFinite() const {
    return OpMath::IsFinite(left) && OpMath::IsFinite(top)
        && OpMath::IsFinite(right) && OpMath::IsFinite(bottom);
}

#if 0 // use std::cbrt instead
/* from: http://metamerist.blogspot.com/2007/09/faster-cube-root-iii.html
we'll divide the integer representation by 3 for an approximation of cbrt(x), which we
can then refine using an iterative method such as Newton-Raphson or Halley's. The
following function returns a cube root approximation with max relative error around 5%
(in my tests). The exponent bias is subtracted, the integer representation is divided
by three and then the exponent bias is restored. Caveat: It is assumed x >= 0. If
negative x is possible, the sign bit needs to be saved and restored.*/
float cbrta(float x)
{
    int& i = (int&)x;
    i = (i - (127 << 23)) / 3 + (127 << 23);
    return x;
}

static float cbrta_halley(const float a, const float R) {
    const float a3 = a * a * a;
    return a * (a3 + R + R) / (a3 + a3 + R);
}

// cube root approximation using 2 iterations of Halley's method (float)
static float halley_cbrt3(float d) {
    float a = cbrta(d);
    a = cbrta_halley(a, d);
    return cbrta_halley(a, d);
}

float OpMath::CubeRoot(float x) {
    if (0 == x) {
        return 0;
    }
    return fabsf(halley_cbrt3(fabsf(x)));
}
#endif

#if 0
// here's an example where the 32 bit float version doesn't work:
// cubic: {3, 4}, {2.8873, 4.1127}, {2.8, 4.16189}, {2.7381, 4.16189}
// (hex: {0x40400000, 0x40800000}, {0x4038c97e, 0x40839b42}, {0x40333334, 0x40852e3c}, {0x402f3d1c, 0x40852e3e})
// intersected with vertical line at: 2.86905241
// give roots: -1.53410935, 39953.0625, 5.97319698
// because Q3=5.57735578e+24  R2=5.57735520e+24
#define ACOS acosf
#define COS cosf
#define SQRT sqrtf
#define ONE 1.f
#define FABS fabsf
#define PI OpPI
#define CUBE_ROOT std::cbrtf
#else
#define ACOS acos
#define COS cos
#define SQRT sqrt
#define ONE 1.
#define FABS fabs
#define PI 3.1415926535897931
#define CUBE_ROOT std::cbrt
#endif

OpRoots OpMath::CubicRootsReal(OpCubicFloatType A, OpCubicFloatType B,
        OpCubicFloatType C, OpCubicFloatType D) {
    if (0 == A)
        return QuadRootsReal(B, C, D);
    if (0 == D) {  // 0 is one root
        OpRoots roots = QuadRootsReal(A, B, C);
        for (unsigned i = 0; i < roots.count; ++i) {
            if (0 == roots.roots[i])
                return roots;
        }
        roots.roots[roots.count++] = 0;
        return roots;
    }
    if (0 == A + B + C + D) {  // 1 is one root
        OpRoots roots = QuadRootsReal(A, A + B, -D);
        for (unsigned i = 0; i < roots.count; ++i) {
            if (1 == roots.roots[i]) {
                return roots;
            }
        }
        roots.roots[roots.count++] = 1;
        return roots;
    }
    OpCubicFloatType invA = 1 / A;
    OpCubicFloatType a = B * invA;
    OpCubicFloatType b = C * invA;
    OpCubicFloatType c = D * invA;
    OpCubicFloatType a2 = a * a;
    OpCubicFloatType Q = (a2 - b * 3) / 9;
    OpCubicFloatType R = (2 * a2 * a - 9 * a * b + 27 * c) / 54;
    OpCubicFloatType R2 = R * R;
    OpCubicFloatType Q3 = Q * Q * Q;
    OpCubicFloatType R2MinusQ3 = R2 - Q3;
    OpCubicFloatType adiv3 = a / 3;
    OpCubicFloatType r;
    OpRoots roots;
    float* rootPtr = &roots.roots[0];
    if (R2MinusQ3 < 0) {   // we have 3 real roots
        // the divide/root can, due to finite precisions, be slightly outside of -1...1
        OpCubicFloatType theta = ACOS(std::max(std::min(ONE, R / SQRT(Q3)), -ONE));
        OpCubicFloatType neg2RootQ = -2 * SQRT(Q);
        r = neg2RootQ * COS(theta / 3) - adiv3;
        *rootPtr++ = r;
        r = neg2RootQ * COS((theta + 2 * PI) / 3) - adiv3;
        if (roots.roots[0] != r)
            *rootPtr++ = r;
        r = neg2RootQ * COS((theta - 2 * PI) / 3) - adiv3;
        if (roots.roots[0] != r && (rootPtr - &roots.roots[0] == 1 || roots.roots[1] != r))
            *rootPtr++ = r;
    } else {  // we have 1 real root
        OpCubicFloatType sqrtR2MinusQ3 = SQRT(R2MinusQ3);
        // !!! need to rename this 'A' something else; since parameter is also 'A'
        A = FABS(R) + sqrtR2MinusQ3;
        A = CUBE_ROOT(A);
        if (R > 0)
            A = -A;
        if (A != 0)
            A += Q / A;
        r = A - adiv3;
        *rootPtr++ = r;
        if (R2 == Q3) {
            r = -A / 2 - adiv3;
            if (roots.roots[0] != r) {
                *rootPtr++ = r;
            }
        }
    }
    roots.count = static_cast<int>(rootPtr - &roots.roots[0]);
    return roots;
}

// min, max not necessarily sorted (between works regardless)
float OpMath::Pin(float min, float value, float max) {
    if (Between(min, value, max))
        return value;
    if (min > max)
        std::swap(min, max);
    return std::max(min, std::min(value, max));
}

