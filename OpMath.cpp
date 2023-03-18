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

OpVector OpVector::normalize() {
    float len = length();
    if (len) {
        float inverseLength = 1 / len;
        dx *= inverseLength;
        dy *= inverseLength;
    }
    return *this;
}

bool OpVector::isFinite() const {
    return OpMath::IsFinite(dx) && OpMath::IsFinite(dy);
}

bool OpPoint::isFinite() const {
    return OpMath::IsFinite(x) && OpMath::IsFinite(y);
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

// if t is nearly end of range, make it end of range
// motivation for this is test cubics_d, which generates yExtrema very nearly equal to 1.
// 'interior' is only used for extrema and inflections
int OpMath::KeepInteriorTs(rootCellar& s, int realRoots, float start, float end) {
    int foundRoots = KeepValidTs(s, realRoots, start, end);
    int interiorRoots = 0;
    for (int index = 0; index < foundRoots; ++index) {
        float tValue = s[index];
        if (start >= tValue - OpEpsilon || tValue + OpEpsilon >= end)
            continue;
        s[interiorRoots++] = tValue;
    }
    return interiorRoots;
}

int OpMath::KeepValidTs(rootCellar& s, int realRoots, float start, float end) {
    size_t foundRoots = 0;
    for (int index = 0; index < realRoots; ++index) {
        float tValue = s[index];
        if (OpMath::IsNaN(tValue) || start > tValue || tValue > end)
            continue;
        for (size_t idx2 = 0; idx2 < foundRoots; ++idx2) {
            if (s[idx2] == tValue) {
                goto notUnique;
            }
        }
        s[foundRoots++] = tValue;
    notUnique:
        ;
    }
    return foundRoots;
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

int OpMath::CubicRootsReal(OpCubicFloatType A, OpCubicFloatType B,
        OpCubicFloatType C, OpCubicFloatType D, rootCellar& s) {
    if (0 == A)
        return QuadRootsReal(B, C, D, s);
    if (0 == D) {  // 0 is one root
        int num = QuadRootsReal(A, B, C, s);
        for (int i = 0; i < num; ++i) {
            if (0 == s[i])
                return num;
        }
        s[num++] = 0;
        return num;
    }
    if (0 == A + B + C + D) {  // 1 is one root
        int num = QuadRootsReal(A, A + B, -D, s);
        for (int i = 0; i < num; ++i) {
            if (1 == s[i]) {
                return num;
            }
        }
        s[num++] = 1;
        return num;
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
    float* roots = &s[0];
    if (R2MinusQ3 < 0) {   // we have 3 real roots
        // the divide/root can, due to finite precisions, be slightly outside of -1...1
        OpCubicFloatType theta = ACOS(std::max(std::min(ONE, R / SQRT(Q3)), -ONE));
        OpCubicFloatType neg2RootQ = -2 * SQRT(Q);
        r = neg2RootQ * COS(theta / 3) - adiv3;
        *roots++ = r;
        r = neg2RootQ * COS((theta + 2 * PI) / 3) - adiv3;
        if (s[0] != r)
            *roots++ = r;
        r = neg2RootQ * COS((theta - 2 * PI) / 3) - adiv3;
        if (s[0] != r && (roots - &s[0] == 1 || s[1] != r))
            *roots++ = r;
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
        *roots++ = r;
        if (R2 == Q3) {
            r = -A / 2 - adiv3;
            if (s[0] != r) {
                *roots++ = r;
            }
        }
    }
    return static_cast<int>(roots - &s[0]);
}

// min, max not necessarily sorted (between works regardless)
float OpMath::Pin(float min, float value, float max) {
    if (Between(min, value, max))
        return value;
    if (min > max)
        std::swap(min, max);
    return std::max(min, std::min(value, max));
}

