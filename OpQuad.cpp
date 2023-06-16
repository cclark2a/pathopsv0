#include "OpCurve.h"
#include "OpTightBounds.h"

OpRoots OpQuad::axisRawHit(Axis axis, float axisIntercept) const {
    OpQuadCoefficients coeff = coefficients(axis);
    coeff.c -= axisIntercept;
    return OpMath::QuadRootsReal(coeff.a, coeff.b, coeff.c);
}

OpQuadCoefficients OpQuad::coefficients(Axis axis) const {
    const float* ptr = pts[0].asPtr(axis);
    float a = ptr[4];
    float b = ptr[2];
    float c = ptr[0];
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    return { a, 2 * b, c };
}

OpRoots OpQuad::extrema(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    float a = ptr[0];
    float b = ptr[2];
    float c = ptr[4];
    float numerator = a - b;
    float denominator = numerator - b + c;
    if (denominator) {
        float result = numerator / denominator;
        if (OpEpsilon <= result && result < 1)
            return OpRoots(result);
    }
    return OpRoots();
}

OpRoots OpQuad::rawIntersect(const LinePts& line) const {
    if (line.pts[0].x == line.pts[1].x)
        return axisRawHit(Axis::vertical, line.pts[0].x);
    if (line.pts[0].y == line.pts[1].y)
        return axisRawHit(Axis::horizontal, line.pts[0].y);
    OpCurve rotated = toVertical(line);
    return rotated.asQuad().axisRawHit(Axis::vertical, 0);
}

OpRoots OpQuad::rayIntersect(const LinePts& line) const {
    if (line.pts[0].x == line.pts[1].x)
        return axisRayHit(Axis::vertical, line.pts[0].x);
    if (line.pts[0].y == line.pts[1].y)
        return axisRayHit(Axis::horizontal, line.pts[0].y);
    OpCurve rotated = toVertical(line);
    return rotated.asQuad().axisRayHit(Axis::vertical, 0);
}

bool OpQuad::monotonic(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    return OpMath::Between(ptr[0], ptr[2], ptr[4]);
}

OpVector OpQuad::normal(float t) const {
    OpVector tan = tangent(t);
    return { -tan.dy, tan.dx };
}

OpPoint OpQuad::ptAtT(float t) const {
    if (0 == t) {
        return pts[0];
    }
    if (1 == t) {
        return pts[2];
    }
    float one_t = 1 - t;
    float a = one_t * one_t;
    float b = 2 * one_t * t;
    float c = t * t;
    return a * pts[0] + b * pts[1] + c * pts[2];
}

CurvePts OpQuad::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    CurvePts result;
    result.pts[0] = ptT1.pt;
    result.pts[2] = ptT2.pt;
    result.weight = 1;
    if (0 == ptT1.t && 1 == ptT2.t)  // called by opsegment addquad if there's no extrema
        result.pts[1] = pts[1];
    else {
        result.pts[1] = 2 * ptAtT((ptT1.t + ptT2.t) / 2) - (result.pts[0] + result.pts[2]) / 2;
        // control point may be (incorrectly) outside bounds formed by end points; so, pin it
        result.pts[1].pin(result.pts[0], result.pts[2]);
    }
    return result;
}

OpVector OpQuad::tangent(float t) const {
    float a = t - 1;
    float b = 1 - 2 * t;
    float c = t;
    return a * pts[0] + b * pts[1] + c * pts[2];
}
