#include "OpCurve.h"
#include <cmath>

int OpCubic::axisRawHit(Axis axis, float axisIntercept, rootCellar& cepts) const {
    OpCubicCoefficients coeff = coefficients(axis);
    coeff.d -= axisIntercept;
    return OpMath::CubicRootsReal(coeff.a, coeff.b, coeff.c, coeff.d, cepts);
}

#if 0   // replace this with calls to axis raw hit followed by keep valid ts
int OpCubic::axisRayHit(Axis axis, float axisIntercept, rootCellar& cepts) const {
    assert(debugGlobalIntersect == debugIntersect);
    OpCubicCoefficients coeff = coefficients(axis);
    coeff.d -= axisIntercept;
    return OpMath::CubicRootsValidT(coeff.a, coeff.b, coeff.c, coeff.d, cepts);
}
#endif

#if 0   // only called by test code; comment out for now
int OpCubic::axisRayHit(Axis axis, float axisIntercept, OpPtT startPtT, OpPtT endPtT, 
        rootCellar& cepts) const {
#if OP_DEBUG
    assert(OpDebugIntersect::edge == debugGlobalIntersect);
    assert(OpDebugIntersect::edge == debugIntersect);
    rootCellar debugCepts;
    int debugRoots = axisRayHit(axis, axisIntercept, debugCepts);
    debugRoots = OpMath::KeepValidTs(debugCepts, debugRoots, startPtT.t, endPtT.t);
    // this gets more precise versions of OpPoint. Needs justification
//    startPtT.pt = doublePtAtT(startPtT.t);
//    endPtT.pt = doublePtAtT(endPtT.t);
#endif
    float startT = startPtT.t;
    float endT = endPtT.t;
    float start = startPtT.pt.choice(axis);
    float end = endPtT.pt.choice(axis);
    if (!OpMath::Between(start, axisIntercept, end)) {
//        assert(!debugRoots);
        return 0;
    }
    float midT;
    do {
        midT = (startT + endT) / 2;
        if (startT >= midT || midT >= endT)
            break;
        float mid = ptAtT(midT).choice(axis);   // was: doublePtAtT: justify
        if (!OpMath::Between(start, mid, end)) {
#if 0 && OP_DEBUG
            // if out of order, should just treat answer as 'close enough'
            float dS = doublePtAtT(startT).choice(axis);
            float dM = doublePtAtT(midT).choice(axis);
            float dE = doublePtAtT(endT).choice(axis);

            float debugStart = ptAtT(startT).choice(axis);
            float debugMid = ptAtT(midT).choice(axis);
            float debugEnd = ptAtT(endT).choice(axis);
            assert(dS || dM || dE || debugStart || debugMid || debugEnd);
#endif
            break;
        }
        bool inFirstHalf = OpMath::Between(start, axisIntercept, mid);
#if OP_DEBUG
        bool inSecondHalf = mid != end && OpMath::Between(std::nextafterf(mid, end), axisIntercept, end);
        assert(inFirstHalf != inSecondHalf);    // only one or the other
#endif
        if (inFirstHalf) {
            endT = midT;
            end = mid;
        } else {
            startT = midT;
            start = mid;
        }
    } while (true);
    cepts[0] = midT;
#if OP_DEBUG
//    assert(1 == debugRoots);
//    assert(fabsf(cepts[0] - debugCepts[0]) <= 6*OpEpsilon);
#endif
    return 1;
}
#endif

OpCubicCoefficients OpCubic::coefficients(Axis axis) const {
    const float* ptr = pts[0].asPtr(axis);
    OpCubicFloatType A = ptr[6];   // d
    OpCubicFloatType B = ptr[4] * 3;  // 3*c
    OpCubicFloatType C = ptr[2] * 3;  // 3*b
    OpCubicFloatType D = ptr[0];   // a
    A -= D - C + B;     // A =   -a + 3*b - 3*c + d
    B += 3 * D - 2 * C; // B =  3*a - 6*b + 3*c
    C -= 3 * D;         // C = -3*a + 3*b
    return { A, B, C, D };
}

int OpCubic::extrema(XyChoice offset, rootCellar& t) const {
    const float* ptr = &pts[0].x + +offset;
    float a = ptr[0];
    float b = ptr[2];
    float c = ptr[4];
    float d = ptr[6];
    float A = d - a + 3 * (b - c);
    float B = 2 * (a - b - b + c);
    float C = b - a;
    return OpMath::QuadRootsInteriorT(A, B, C, t);  // don't keep roots ~0, ~1
}

int OpCubic::inflections(rootCellar& t) const {
    OpPoint A = pts[1] - pts[0];
    OpPoint B = pts[2] - 2 * pts[1] + pts[0];
    OpPoint C = pts[3] + 3 * (pts[1] - pts[2]) - pts[0];
    return OpMath::QuadRootsInteriorT(B.x * C.y - B.y * C.x, A.x * C.y - A.y * C.x,
            A.x * B.y - A.y * B.x, t);  // don't keep roots ~0, ~1
}

OpPoint OpCubic::interp(float t) const {
    OpPoint ab = OpMath::Interp(pts[0], pts[1], t);
    OpPoint bc = OpMath::Interp(pts[1], pts[2], t);
    OpPoint cd = OpMath::Interp(pts[2], pts[3], t);
    OpPoint abc = OpMath::Interp(ab, bc, t);
    OpPoint bcd = OpMath::Interp(bc, cd, t);
    OpPoint abcd = OpMath::Interp(abc, bcd, t);
    return abcd;
}

int OpCubic::rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const {
    if (line[0].x == line[1].x)
        return axisRawHit(Axis::vertical, line[0].x, cepts);
    if (line[0].y == line[1].y)
        return axisRawHit(Axis::horizontal, line[0].y, cepts);
    OpCubic rotated;
    toVertical(line, rotated);
    return rotated.axisRawHit(Axis::vertical, 0, cepts);
}

int OpCubic::rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const {
    if (line[0].x == line[1].x)
        return axisRayHit(Axis::vertical, line[0].x, cepts);
    if (line[0].y == line[1].y)
        return axisRayHit(Axis::horizontal, line[0].y, cepts);
    OpCubic rotated;
    toVertical(line, rotated);
    return rotated.axisRayHit(Axis::vertical, 0, cepts);
}

bool OpCubic::monotonic(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    return OpMath::Between(ptr[0], ptr[2], ptr[6])
        && OpMath::Between(ptr[0], ptr[4], ptr[6]);
}

OpVector OpCubic::normal(float t) const {
    OpVector tan = tangent(t);
    return { -tan.dy, tan.dx };
}

OpPoint OpCubic::ptAtT(float t) const {
    if (0 == t) {
        return pts[0];
    }
    if (1 == t) {
        return pts[3];
    }
    float one_t = 1 - t;
    float one_t2 = one_t * one_t;
    float a = one_t2 * one_t;
    float b = 3 * one_t2 * t;
    float t2 = t * t;
    float c = 3 * one_t * t2;
    float d = t2 * t;
    return a * pts[0] + b * pts[1] + c * pts[2] + d * pts[3];
}

OpPoint OpCubic::doublePtAtT(float t) const {
    if (0 == t) {
        return pts[0];
    }
    if (1 == t) {
        return pts[3];
    }
    double one_t = 1. - t;
    double one_t2 = one_t * one_t;
    double a = one_t2 * one_t;
    double b = 3 * one_t2 * t;
    double t2 = t * t;
    double c = 3 * one_t * t2;
    double d = t2 * t;
    return {
        (float)(a * pts[0].x + b * pts[1].x + c * pts[2].x + d * pts[3].x),
        (float)(a * pts[0].y + b * pts[1].y + c * pts[2].y + d * pts[3].y)
    };
}

void OpCubic::pinCtrls(XyChoice offset) {
    OP_DEBUG_CODE(float orig1 = pts[1].choice(offset));
    OP_DEBUG_CODE(float orig2 = pts[2].choice(offset));
    *pts[1].asPtr(offset) = OpMath::Pin(pts[0].choice(offset), pts[1].choice(offset), pts[3].choice(offset));
    *pts[2].asPtr(offset) = OpMath::Pin(pts[0].choice(offset), pts[2].choice(offset), pts[3].choice(offset));
#if OP_DEBUG
    if (orig1 != pts[1].choice(offset))
        *debugOriginalCtrls[0].asPtr(offset) = orig1;
    if (orig2 != pts[2].choice(offset))
        *debugOriginalCtrls[1].asPtr(offset) = orig2;
#endif
}

void OpCubic::subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dst) const {
    dst[0] = ptT1.pt;
    dst[3] = ptT2.pt;
    if (ptT1.t == 0 && ptT2.t == 1) {
        dst[1] = pts[1];
        dst[2] = pts[2];
        return;
    }
    OpPoint a = interp(ptT1.t);
    OpPoint e = interp((ptT1.t * 2 + ptT2.t) / 3);
    OpPoint f = interp((ptT1.t + ptT2.t * 2) / 3);
    OpPoint d = interp(ptT2.t);
    OpVector m = e * 27 - a * 8 - d;
    OpVector n = f * 27 - a - d * 8;
    /* b = */ dst[1] = (m * 2 - n) / 18;
    /* c = */ dst[2] = (n * 2 - m) / 18;
    dst[1].pin(dst[0], dst[3]);
    dst[2].pin(dst[0], dst[3]);
}

// !!! this does not handle if t == 0 and pt[0] == pt[1] or if t == 1 and pt[2] == pt[3]
// don't think this is needed for pathops, but is used by debugging (image drawing)
// for now, put the additional logic there
float OpCubic::tangent(XyChoice offset, double t) const {
    const float* ptr = &pts[0].x + +offset;
    double one_t = 1 - t;
    double a = ptr[0];
    double b = ptr[2];
    double c = ptr[4];
    double d = ptr[6];
    return 3 * ((b - a) * one_t * one_t + 2 * (c - b) * t * one_t + (d - c) * t * t);
}

OpVector OpCubic::tangent(float t) const {
    return { tangent(XyChoice::inX, t), tangent(XyChoice::inY, t) };
}
