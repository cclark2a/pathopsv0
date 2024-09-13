// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOpsTypes.h"

namespace PathOpsV0Lib {

struct PointWeight {
    PointWeight() {
    }

    PointWeight(Curve c) {
        OP_ASSERT(sizeof(PointWeight) == c.size - offsetof(CurveData, optionalAdditionalData));
        const char* data = c.data->optionalAdditionalData;
        std::memcpy(&pt, data, sizeof pt);
        data += sizeof pt;
        std::memcpy(&weight, data, sizeof weight);
    }

    PointWeight(OpPoint p, float w)
        : pt(p)
        , weight(w) {
    }

    void copyTo(Curve c) {
        OP_ASSERT(sizeof(PointWeight) == c.size - offsetof(CurveData, optionalAdditionalData));
        char* data = c.data->optionalAdditionalData;
        std::memcpy(data, &pt, sizeof pt);
        data += sizeof pt;
        std::memcpy(data, &weight, sizeof weight);
    }

    OpPoint pt;
    float weight;
};

inline float ConicDenom(float weight, float t) {
    float B = 2 * (weight - 1);
    float C = 1;
    float A = -B;
    return (A * t + B) * t + C;
}

inline OpPoint ConicNumer(OpPoint start, PointWeight control, OpPoint end, float t) {
    OpPoint pt1w = control.pt * control.weight;
    OpPoint C = start;
    OpPoint A = end - 2 * pt1w + C;
    OpPoint B = 2 * (pt1w - C);
    return (A * t + B) * t + C;
}

inline OpPoint ConicPointAtT(OpPoint start, PointWeight control, OpPoint end, float t) {
    if (0 == t)
        return start;
    if (1 == t)
        return end;
    return ConicNumer(start, control, end, t) / ConicDenom(control.weight, t);
}

// given a pair of t values, return a pair of x values
// !!! implementation is non-optimal; but see if it works at all, and wonder, who uses conics?
inline OpPair ConicXYAtT(OpPoint s, PointWeight c, OpPoint e, OpPair t, XyChoice xy) {
    return { ConicPointAtT(s, c, e, t.s).choice(xy), ConicPointAtT(s, c, e, t.l).choice(xy) };
}

inline PointWeight ConicControl(OpPoint start, PointWeight control, OpPoint end, OpPtT ptT1, OpPtT ptT2) {
    if (0 == ptT1.t && 1 == ptT2.t)
        return control;
    auto subWeight = [start, control, end, ptT1, ptT2](float t) {
        if (t == 0)
            return PointWeight(ptT1.pt, 1);
        if (t == 1)
            return PointWeight(ptT2.pt, 1);
        return PointWeight(ConicNumer(start, control, end, t), ConicDenom(control.weight, t));
    };
    PointWeight a = subWeight(ptT1.t);
    PointWeight c = subWeight(ptT2.t);
    float midT = OpMath::Average(ptT1.t, ptT2.t);
    PointWeight d(ConicNumer(start, control, end, midT), ConicDenom(control.weight, midT));
    PointWeight b(2 * d.pt - (a.pt + c.pt) / 2,  // !!! add math pt average?
            2 * d.weight - OpMath::Average(a.weight, c.weight));  // !!! rewrite with fma?
    // if bz is 0, weight is 0, control point has no effect: any value will do
    float bzNonZero = !b.weight ? 1 : b.weight;
    if (!b.weight)
        b.weight = 1;
    PointWeight result(b.pt / bzNonZero, b.weight / sqrtf(a.weight * c.weight));
    result.pt.pin(ptT1.pt, ptT2.pt);
    return result;
}

inline OpRoots ConicAxisRawHit(OpPoint start, PointWeight control, OpPoint end, Axis axis, 
        float intercept) {
    float a = end.choice(axis);
    float b = control.pt.choice(axis) * control.weight - intercept * control.weight + intercept;
    float c = start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    return OpMath::QuadRootsDouble(a, 2 * b, c - intercept);  // ? double req'd: testConics3759897
}

inline OpQuadCoefficients DerivativeCoefficients(
        OpPoint start, PointWeight control, OpPoint end, XyChoice offset) {
    float P20 = end.choice(offset) - start.choice(offset);
    float P10 = control.pt.choice(offset) - start.choice(offset);
    float wP10 = control.weight * P10;
    float a = control.weight * P20 - P20;
    float b = P20 - 2 * wP10;
    float c = wP10;
    return { a, b, c };
}

inline float ConicTangent(OpPoint start, PointWeight control, OpPoint end, XyChoice offset, 
        float t) {
    OpQuadCoefficients coeff = DerivativeCoefficients(start, control, end, offset);
    return t * (t * coeff.a + coeff.b) + coeff.c;
}

inline OpVector ConicTangent(OpPoint start, PointWeight control, OpPoint end, float t) {
    if ((OpMath::NearlyZeroT(t) && start.isNearly(control.pt))
            || (OpMath::NearlyOneT(t) && end.isNearly(control.pt)))
        return end - start;
    return { ConicTangent(start, control, end, XyChoice::inX, t), 
            ConicTangent(start, control, end, XyChoice::inY, t) };
}

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the Conicratic Bezier
inline size_t AddConics(AddCurve curve, AddWinding windings) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[1];
    float weight = curve.points[3].x;  // !!! a bit of a hack
    PointWeight control(curve.points[2], weight);
    auto [left, right] = std::minmax(start.x, end.x);
    bool monotonicInX = left <= control.pt.x && control.pt.x <= right;
    auto [top, bottom] = std::minmax(start.y, end.y);
    bool monotonicInY = top <= control.pt.y && control.pt.y <= bottom;
    if (monotonicInX && monotonicInY) {
        Add(curve, windings);
        return 1;
    }
    // control point is not inside bounds formed by end points; split Conic into parts
    std::vector<float> tValues { 0, 1 };
    auto addExtrema = [start, control, end, &tValues](XyChoice offset) {
        OpQuadCoefficients dc = DerivativeCoefficients(start, control, end, offset);
        OpRoots roots = OpMath::QuadRootsInteriorT(dc.a, dc.b, dc.c);
        OP_ASSERT(0 == roots.count || 1 == roots.count);   // !!! I wanna see the extreme case...
        if (0 == roots.count)
            return;
        tValues.push_back(roots.roots[0]);
    };
    if (!monotonicInX)
        addExtrema(XyChoice::inX);
    if (!monotonicInY)
        addExtrema(XyChoice::inY);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { ConicPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        struct ConicData {
            OpPoint endPts[2];
            PointWeight control;
        } curveData { { ptTs[index].pt, ptTs[index + 1].pt },
                ConicControl(start, control, end, ptTs[index], ptTs[index + 1]) };
        Add({ curveData.endPts, curve.size, curve.type }, windings );
    }
    return curvesAdded;
}

// callback functions
inline size_t conicPtCount() {
    return 3;
}

inline bool conicIsFinite(Curve c) {
    PointWeight control(c);
    return control.pt.isFinite();
}

inline bool conicIsLine(Curve c) {
    PointWeight control(c);
    LinePts linePts { c.data->start, c.data->end };
    return linePts.ptOnLine(control.pt);
}

inline OpRoots conicAxisRawHit(Curve c, Axis axis, float axisIntercept, MatchEnds ends) {
    PointWeight control(c);
    return ConicAxisRawHit(c.data->start, control, c.data->end, axis, axisIntercept);
}

inline OpPoint conicPtAtT(Curve c, float t) {
    PointWeight control(c);
    return ConicPointAtT(c.data->start, control, c.data->end, t);
}

inline OpPair conicXYAtT(Curve c, OpPair t, XyChoice xyChoice) {
    PointWeight control(c);
    return ConicXYAtT(c.data->start, control, c.data->end, t, xyChoice);
}

inline bool conicsEqual(Curve one, Curve two) {
    PointWeight ctrl1(one);
    PointWeight ctrl2(two);
    return ctrl1.pt == ctrl2.pt && ctrl1.weight == ctrl2.weight;
}

inline OpVector conicTangent(Curve c, float t) {
    PointWeight control(c);
    return ConicTangent(c.data->start, control, c.data->end, t);
}

inline OpVector conicNormal(Curve c, float t) {
    OpVector tan = conicTangent(c, t);
    return { -tan.dy, tan.dx };
}

inline void conicRotate(Curve c, const LinePts& line, float adj, float opp, Curve result) {
    PointWeight control(c);
    OpVector v = control.pt - line.pts[0];
    PointWeight rotated({ v.dy * adj - v.dx * opp, v.dy * opp + v.dx * adj }, control.weight);
    rotated.copyTo(result);
}

inline void conicSetBounds(Curve c, OpRect& bounds) {
    PointWeight control(c);
    bounds.add(control.pt);
}

inline void conicSubDivide(Curve curve, OpPtT ptT1, OpPtT ptT2, Curve result) {
    result.data->start = ptT1.pt;
    result.data->end = ptT2.pt;
    PointWeight control(curve);
    PointWeight subPtW = ConicControl(curve.data->start, control, curve.data->end, ptT1, ptT2);
    subPtW.copyTo(result);
}

inline OpPoint conicHull(Curve c, int index) {
    if (1 == index)
        return PointWeight(c).pt;
    OP_ASSERT(0); // should never be called
    return OpPoint();
}

#if OP_DEBUG_DUMP
inline std::string conicDebugDumpName() { 
    return "conic"; 
}

inline std::string conicDebugDumpExtra(Curve c, DebugLevel l, DebugBase b) {
    PointWeight control(c);
    return debugValue(l, b, " weight", control.weight);
}
#endif

}
