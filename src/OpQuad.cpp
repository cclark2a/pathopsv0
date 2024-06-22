// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"
#include "OpTightBounds.h"

OpRoots OpQuad::axisRawHit(Axis axis, float axisIntercept) const {
    OpQuadCoefficients coeff = coefficients(axis);
    coeff.c -= axisIntercept;
    return OpMath::QuadRootsDouble(coeff.a, coeff.b, coeff.c);  // required for testQuads3759897
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
        if (OpEpsilon <= result && result < 1)  // !!! curious: only epsilon use in conic/cubic/quad
            return OpRoots(result);
    }
    return OpRoots();
}

OpRoots OpQuad::rawIntersect(const LinePts& line) const {
    OP_ASSERT(!newInterface);
    if (line.pts[0].x == line.pts[1].x)
        return axisRawHit(Axis::vertical, line.pts[0].x);
    if (line.pts[0].y == line.pts[1].y)
        return axisRawHit(Axis::horizontal, line.pts[0].y);
    OpCurve rotated = toVertical(line);
    return rotated.asQuad().axisRawHit(Axis::vertical, 0);
}

bool OpQuad::monotonic(XyChoice offset) const {
    const float* ptr = &pts[0].x + +offset;
    return OpMath::Between(ptr[0], ptr[2], ptr[4]);
}

OpVector OpQuad::normal(float t) const {
    OP_ASSERT(!newInterface);
    OpVector tan = tangent(t);
    return { -tan.dy, tan.dx };
}

void OpQuad::pinCtrl() {
    pts[1].pin(pts[0], pts[2]);
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

OpCurve OpQuad::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    OpCurve result;
    result.type = OpType::quad;
    result.pts[0] = ptT1.pt;
    result.pts[2] = ptT2.pt;
    result.weight = 1;
    if (0 == ptT1.t && 1 == ptT2.t)  // called by opsegment addquad if there's no extrema
        result.pts[1] = pts[1];
    else {
        result.pts[1] = 2 * ptAtT((ptT1.t + ptT2.t) / 2) - (result.pts[0] + result.pts[2]) / 2;
        result.asQuad().pinCtrl();  // control point may be outside bounds formed by end points
    }
    return result;
}

OpVector OpQuad::tangent(float t) const {
    OP_ASSERT(!newInterface);
    if ((0 == t && pts[0] == pts[1]) || (1 == t && pts[2] == pts[1]))
        return pts[2] - pts[0];
    float a = t - 1;
    float b = 1 - 2 * t;
    float c = t;
    return a * pts[0] + b * pts[1] + c * pts[2];
}

// given a pair of t values, return a pair of x values
OpPair OpQuad::xyAtT(OpPair t, XyChoice xy) const {
    OpPair one_t = 1 - t;
    OpPair a = one_t * one_t;
    OpPair b = 2 * one_t * t;
    OpPair c = t * t;
    return a * pts[0].choice(xy) + b * pts[1].choice(xy) + c * pts[2].choice(xy);
}


// new interface idea

#include "PathOps.h"

namespace PathOpsV0Lib {

OpPoint QuadPointAtT(OpPoint start, OpPoint control, OpPoint end, float t) {
    if (0 == t)
        return start;
    if (1 == t)
        return end;
    float one_t = 1 - t;
    float a = one_t * one_t;
    float b = 2 * one_t * t;
    float c = t * t;
    OpPoint result = a * start + b * control + c * end;
    return result;
}

OpPair QuadXYAtT(OpPoint start, OpPoint control, OpPoint end, OpPair t, XyChoice xy) {
    OpPair one_t = 1 - t;
    OpPair a = one_t * one_t;
    OpPair b = 2 * one_t * t;
    OpPair c = t * t;
    return a * start.choice(xy) + b * control.choice(xy) + c * end.choice(xy);
}

OpPoint QuadControlPt(OpPoint start, OpPoint control, OpPoint end, OpPtT ptT1, OpPtT ptT2) {
    OpPoint midPt = QuadPointAtT(start, control, end, (ptT1.t + ptT2.t) / 2);
    OpPoint avgPt = (ptT1.pt + ptT2.pt) / 2;
    OpPoint result = 2 * midPt - avgPt;
    result.pin(ptT1.pt, ptT2.pt);
    return result;
}

OpRoots QuadAxisRawHit(OpPoint start, OpPoint control, OpPoint end, Axis axis, float axisIntercept) {
    float a = end.choice(axis);
    float b = control.choice(axis);
    float c = start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    return OpMath::QuadRootsDouble(a, 2 * b, c - axisIntercept);  // double req'd: testQuads3759897
}

OpVector QuadTangent(OpPoint start, OpPoint control, OpPoint end, float t) {
    if ((0 == t && start == control) || (1 == t && end == control))
        return end - start;
    float a = t - 1;
    float b = 1 - 2 * t;
    float c = t;
    return a * start + b * control + c * end;
}

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the quadratic Bezier
size_t AddQuads(AddCurve curve, AddWinding windings) {
    CurveData& q = *(CurveData*) curve.points;
    OpPoint& control = *(OpPoint*) q.optionalAdditionalData;
    auto [left, right] = std::minmax(q.start.x, q.end.x);
    bool monotonicInX = left <= control.x && control.x <= right;
    auto [top, bottom] = std::minmax(q.start.y, q.end.y);
    bool monotonicInY = top <= control.y && control.y <= bottom;
    if (monotonicInX && monotonicInY) {
        Add(curve, windings);
        return 1;
    }
    // control point is not inside bounds formed by end points; split quad into parts
    std::vector<float> tValues { 0, 1 };
    auto addExtrema = [&tValues](float s, float c, float e) {
        float numerator = s - c;
        float denominator = numerator - c + e;
        if (0 == denominator)
            return;
        float extrema = numerator / denominator;
        if (OpEpsilon <= extrema && extrema < 1)
            tValues.push_back(extrema);
    };
    if (!monotonicInX)
        addExtrema(q.start.x, control.x, q.end.x);
    if (!monotonicInY)
        addExtrema(q.start.y, control.y, q.end.y);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { q.start, 0 };
    ptTs.back() = { q.end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { QuadPointAtT(q.start, control, q.end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        OpPoint curveData[3] { ptTs[index].pt, ptTs[index + 1].pt,
            QuadControlPt(q.start, control, q.end, ptTs[index], ptTs[index + 1]) };
        Add({ curve.context, curveData, curve.size, curve.type }, windings );
    }
    return curvesAdded;
}

}
