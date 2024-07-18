// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

inline OpPoint QuadPointAtT(OpPoint start, OpPoint control, OpPoint end, float t) {
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

inline OpPair QuadXYAtT(OpPoint start, OpPoint control, OpPoint end, OpPair t, XyChoice xy) {
    OpPair one_t = 1 - t;
    OpPair a = one_t * one_t;
    OpPair b = 2 * one_t * t;
    OpPair c = t * t;
    return a * start.choice(xy) + b * control.choice(xy) + c * end.choice(xy);
}

inline OpPoint QuadControlPt(OpPoint start, OpPoint control, OpPoint end, OpPtT ptT1, OpPtT ptT2) {
    OpPoint midPt = QuadPointAtT(start, control, end, (ptT1.t + ptT2.t) / 2);
    OpPoint avgPt = (ptT1.pt + ptT2.pt) / 2;
    OpPoint result = 2 * midPt - avgPt;
    result.pin(ptT1.pt, ptT2.pt);
    return result;
}

inline OpRoots QuadAxisRawHit(OpPoint start, OpPoint control, OpPoint end, Axis axis, float axisIntercept) {
    float a = end.choice(axis);
    float b = control.choice(axis);
    float c = start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    return OpMath::QuadRootsDouble(a, 2 * b, c - axisIntercept);  // double req'd: testQuads3759897
}

inline OpVector QuadTangent(OpPoint start, OpPoint control, OpPoint end, float t) {
    if ((0 == t && start == control) || (1 == t && end == control))
        return end - start;
    float a = t - 1;
    float b = 1 - 2 * t;
    float c = t;
    return a * start + b * control + c * end;
}

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the quadratic Bezier
inline size_t AddQuads(AddCurve curve, AddWinding windings) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[1];
    OpPoint control = curve.points[2];
    auto [left, right] = std::minmax(start.x, end.x);
    bool monotonicInX = left <= control.x && control.x <= right;
    auto [top, bottom] = std::minmax(start.y, end.y);
    bool monotonicInY = top <= control.y && control.y <= bottom;
    if (monotonicInX && monotonicInY) {
        if (start == end)
            return 0;
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
        addExtrema(start.x, control.x, end.x);
    if (!monotonicInY)
        addExtrema(start.y, control.y, end.y);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { QuadPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        OpPoint curveData[3] { ptTs[index].pt, ptTs[index + 1].pt,
            QuadControlPt(start, control, end, ptTs[index], ptTs[index + 1]) };
        Add({ curveData, curve.size, curve.type }, windings );
    }
    return curvesAdded;
}

inline OpPoint quadControlPt(Curve c) {
    OpPoint result;
    OP_ASSERT(sizeof(OpPoint) == c.size - offsetof(CurveData, optionalAdditionalData));
    std::memcpy(&result, c.data->optionalAdditionalData, sizeof(OpPoint));
    return result;
}

inline void quadSetControl(Curve c, OpPoint pt) {
    OP_ASSERT(sizeof(OpPoint) == c.size - offsetof(CurveData, optionalAdditionalData));
    std::memcpy(c.data->optionalAdditionalData, &pt, sizeof(OpPoint));
}

// callback functions
inline size_t quadPtCount() {
    return 3;
}

inline bool quadNearly(Curve c) {
    OpPoint ctrlPt = quadControlPt(c);
	return ctrlPt.isNearly(c.data->start) || ctrlPt.isNearly(c.data->end);
}

inline bool quadIsFinite(Curve c) {
    return quadControlPt(c).isFinite();
}

inline bool quadIsLine(Curve ) {
    return false;
}

inline OpRoots quadAxisRawHit(Curve c, Axis axis, float axisIntercept, MatchEnds ends) {
    return QuadAxisRawHit(c.data->start, quadControlPt(c), c.data->end, axis, axisIntercept);
}

inline OpPoint quadPtAtT(Curve c, float t) {
    return QuadPointAtT(c.data->start, quadControlPt(c), c.data->end, t);
}

inline OpPair quadXYAtT(Curve c, OpPair t, XyChoice xyChoice) {
    return QuadXYAtT(c.data->start, quadControlPt(c), c.data->end, t, xyChoice);
}

inline bool quadsEqual(Curve one, Curve two) {
    OpPoint ctrlPt1 = quadControlPt(one);
    OpPoint ctrlPt2 = quadControlPt(two);
    return ctrlPt1 == ctrlPt2;
}

inline OpVector quadTangent(Curve c, float t) {
    return QuadTangent(c.data->start, quadControlPt(c), c.data->end, t);
}

inline OpVector quadNormal(Curve c, float t) {
    OpVector tan = quadTangent(c, t);
    return { -tan.dy, tan.dx };
}

inline void quadPinCtrl(Curve c) {
    OpPoint ctrlPt = quadControlPt(c);
    ctrlPt.pin(c.data->start, c.data->end);
    quadSetControl(c, ctrlPt);
}

inline void quadRotate(Curve c, const LinePts& line, float adj, float opp, Curve result) {
    OpPoint ctrlPt = quadControlPt(c);
    OpVector v = ctrlPt - line.pts[0];
    OpPoint rotated(v.dy * adj - v.dx * opp, v.dy * opp + v.dx * adj);
    quadSetControl(result, rotated);
}

inline void quadSetBounds(Curve c, OpRect& bounds) {
    bounds.add(quadControlPt(c));
}

inline void quadSubDivide(Curve c, OpPtT ptT1, OpPtT ptT2, Curve result) {
    result.data->start = ptT1.pt;
    result.data->end = ptT2.pt;
    OpPoint subControl = QuadControlPt(c.data->start, quadControlPt(c), c.data->end, ptT1, ptT2);
    quadSetControl(result, subControl);
}

inline OpPoint quadHull(Curve c, int index) {
    if (1 == index)
        return quadControlPt(c);
    OP_ASSERT(0); // should never be called
    return OpPoint();
}

inline bool quadIsLinear(Curve c) {
    OpPoint controlPt = quadControlPt(c);
    OpVector diffs[] { controlPt - c.data->start, c.data->end - c.data->start };
    float cross = diffs[0].cross(diffs[1]);
    return fabsf(cross) <= OpEpsilon;
}

#if 0 && OP_DEBUG_DUMP
inline size_t quadDebugDumpSize() {
    return offsetof(CurveData, optionalAdditionalData) + sizeof(OpPoint);
}

inline void quadDumpSet(Curve c, const char*& str) {
    quadControlPt(c).dumpSet(str);
}
#endif

}
