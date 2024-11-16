// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea

#include "PathOps.h"

namespace PathOpsV0Lib {

struct CubicControls {
    CubicControls() {
    }

    CubicControls(Curve c) {
        OP_ASSERT(sizeof(CubicControls) == c.size - CurveUserDataOffset());
        std::memcpy(pts, CurveUserData(c.data), sizeof(CubicControls));
    }

    CubicControls(OpPoint pt1, OpPoint pt2) {
        pts[0] = pt1;
        pts[1] = pt2;
    }

    void copyTo(Curve c) {
        OP_ASSERT(sizeof(CubicControls) == c.size - CurveUserDataOffset());
        std::memcpy(CurveUserData(c.data), this, sizeof(CubicControls));
    }

    OpPoint pts[2];
};

inline OpPoint CubicPointAtT(OpPoint start, CubicControls controls, OpPoint end, float t) {
    if (0 == t)
        return start;
    if (1 == t)
        return end;
    float one_t = 1 - t;
    float one_t2 = one_t * one_t;
    float a = one_t2 * one_t;
    float b = 3 * one_t2 * t;
    float t2 = t * t;
    float c = 3 * one_t * t2;
    float d = t2 * t;
    OpPoint result = a * start + b * controls.pts[0] + c * controls.pts[1] + d * end;
    return result;
}

inline OpPair CubicXYAtT(OpPoint start, CubicControls controls, OpPoint end, OpPair t, XyChoice xy) {
    OpPair one_t = 1 - t;
    OpPair one_t2 = one_t * one_t;
    OpPair a = one_t2 * one_t;
    OpPair b = 3 * one_t2 * t;
    OpPair t2 = t * t;
    OpPair c = 3 * one_t * t2;
    OpPair d = t2 * t;
    return a * start.choice(xy) + b * controls.pts[0].choice(xy) 
            + c * controls.pts[1].choice(xy) + d * end.choice(xy);
}

inline CubicControls CubicControlPt(OpPoint start, CubicControls controls, OpPoint end, 
        OpPtT ptT1, OpPtT ptT2) {
    auto interp = [start, controls, end](float t) {
        OpPoint ab = OpMath::Interp(start, controls.pts[0], t);
        OpPoint bc = OpMath::Interp(controls.pts[0], controls.pts[1], t);
        OpPoint cd = OpMath::Interp(controls.pts[1], end, t);
        OpPoint abc = OpMath::Interp(ab, bc, t);
        OpPoint bcd = OpMath::Interp(bc, cd, t);
        OpPoint abcd = OpMath::Interp(abc, bcd, t);
        return abcd;
    };
//    OpPoint a = interp(ptT1.t);
    OpPoint e = interp((ptT1.t * 2 + ptT2.t) / 3);
    OpPoint f = interp((ptT1.t + ptT2.t * 2) / 3);
//    OpPoint d = interp(ptT2.t);
    OpVector m = e * 27 - ptT1.pt * 8 - ptT2.pt;
    OpVector n = f * 27 - ptT1.pt - ptT2.pt * 8;
    CubicControls results;
    /* b = */ results.pts[0] = (m * 2 - n) / 18;
    /* c = */ results.pts[1] = (n * 2 - m) / 18;
    results.pts[0].pin(ptT1.pt, ptT2.pt);
    results.pts[1].pin(ptT1.pt, ptT2.pt);
    return results;
}

inline OpRoots CubicAxisRawHit(OpPoint start, CubicControls controls, OpPoint end, Axis axis, 
        float axisIntercept, MatchEnds matchEnds) {
    OpCubicFloatType A = end.choice(axis);   // d
    OpCubicFloatType B = controls.pts[1].choice(axis) * 3;  // 3*c
    OpCubicFloatType C = controls.pts[0].choice(axis) * 3;  // 3*b
    OpCubicFloatType D = start.choice(axis);   // a
    A -= D - C + B;     // A =   -a + 3*b - 3*c + d
    B += 3 * D - 2 * C; // B =  3*a - 6*b + 3*c
    C -= 3 * D;         // C = -3*a + 3*b
    return OpMath::CubicRootsReal(A, B, C, D - axisIntercept, matchEnds);
}

inline OpVector CubicTangent(OpPoint start, CubicControls controls, OpPoint end, float t) {
    // !!! document why this needs to be double (include example test requiring it)
    auto tangent = [start, controls, end](XyChoice offset, double t) {
        double one_t = 1 - t;
        double a = start.choice(offset);
        double b = controls.pts[0].choice(offset);
        double c = controls.pts[1].choice(offset);
        double d = end.choice(offset);
        return (float) (3 * ((b - a) * one_t * one_t + 2 * (c - b) * t * one_t + (d - c) * t * t));
    };
    OpPoint threshold = OpMath::Threshold(start, end);
    if (OpMath::NearlyZeroT(t) && start.isNearly(controls.pts[0], threshold)) {
        if (controls.pts[0].isNearly(controls.pts[1], threshold))
            return end - start;
        else
            return controls.pts[1] - start;
    }
    if (OpMath::NearlyOneT(t) && end.isNearly(controls.pts[1], threshold)) {
        if (controls.pts[0].isNearly(controls.pts[1], threshold))
            return end - start;
        else
            return end - controls.pts[0];
    }
    return { tangent(XyChoice::inX, t), tangent(XyChoice::inY, t) };
}

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the cubic Bezier
inline size_t AddCubics(AddCurve curve, AddWinding windings) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[1];
    CubicControls controls { curve.points[2], curve.points[3] };
    // control point is not inside bounds formed by end points; split cubic into parts
    std::vector<float> tValues { 0, 1 };
    auto addExtrema = [&tValues](float a, float b, float c, float d) {
        float A = d - a + 3 * (b - c);
        float B = 2 * (a - b - b + c);
        float C = b - a;
        OpRoots roots = OpMath::QuadRootsInteriorT(A, B, C);  // don't keep roots ~0, ~1
        for (unsigned index = 0; index < roots.count; ++index)
            tValues.push_back(roots.roots[index]);
    };
    addExtrema(start.x, controls.pts[0].x, controls.pts[1].x, end.x);
    addExtrema(start.y, controls.pts[0].y, controls.pts[1].y, end.y);
    // inflections
    OpPoint A = controls.pts[0] - start;
    OpPoint B = controls.pts[1] - 2 * controls.pts[0] + start;
    OpPoint C = end + 3 * (controls.pts[0] - controls.pts[1]) - start;
    OpRoots roots = OpMath::QuadRootsInteriorT(B.x * C.y - B.y * C.x, A.x * C.y - A.y * C.x,
            A.x * B.y - A.y * B.x);  // don't keep roots ~0, ~1
    for (unsigned index = 0; index < roots.count; ++index)
        tValues.push_back(roots.roots[index]);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { CubicPointAtT(start, controls, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        OpPoint curveData[4] { ptTs[index].pt, ptTs[index + 1].pt };
        *(CubicControls*)&curveData[2] = CubicControlPt(start, controls, end, 
                ptTs[index], ptTs[index + 1]);
        if (curveData[0] == curveData[1])
            continue;
        Add({ curveData, curve.size, curve.type }, windings );
    }
    return curvesAdded;
}

// callback functions
inline size_t cubicPtCount() {
    return 4;
}

inline bool cubicIsFinite(Curve c) {
    CubicControls controls(c);
    return controls.pts[0].isFinite() && controls.pts[1].isFinite();
}

inline bool cubicIsLine(Curve c) {
    CubicControls controls(c);
    LinePts linePts = { c.data->start, c.data->end };
    return linePts.ptOnLine(controls.pts[0]) && linePts.ptOnLine(controls.pts[1]);
}

inline OpRoots cubicAxisRawHit(Curve c, Axis axis, float axisIntercept, MatchEnds ends) {
    CubicControls controls(c);
    return CubicAxisRawHit(c.data->start, controls, c.data->end, axis, axisIntercept, ends);
}

inline OpPoint cubicPtAtT(Curve c, float t) {
    CubicControls controls(c);
    return CubicPointAtT(c.data->start, controls, c.data->end, t);
}

inline OpPair cubicXYAtT(Curve c, OpPair t, XyChoice xyChoice) {
    CubicControls controls(c);
    return CubicXYAtT(c.data->start, controls, c.data->end, t, xyChoice);
}

inline bool cubicsEqual(Curve one, Curve two) {
    CubicControls ctrlPt1(one);
    CubicControls ctrlPt2(two);
    return one.data->start == two.data->start  // check if curves are reversed
            ? ctrlPt1.pts[0] == ctrlPt2.pts[0] && ctrlPt1.pts[1] == ctrlPt2.pts[1]
            : ctrlPt1.pts[0] == ctrlPt2.pts[1] && ctrlPt1.pts[1] == ctrlPt2.pts[0];
}

inline OpVector cubicTangent(Curve c, float t) {
    CubicControls controls(c);
    return CubicTangent(c.data->start, controls, c.data->end, t);
}

inline OpVector cubicNormal(Curve c, float t) {
    OpVector tan = cubicTangent(c, t);
    return { -tan.dy, tan.dx };
}

inline void cubicPinCtrl(Curve c) {
    CubicControls controls(c);
    controls.pts[0].pin(c.data->start, c.data->end);
    controls.pts[1].pin(c.data->start, c.data->end);
    controls.copyTo(c);
}

inline void cubicRotate(Curve c, const LinePts& line, float adj, float opp, Curve result) {
    CubicControls controls(c);
    for (int index = 0; index < 2; ++index) {
        OpVector v = controls.pts[index] - line.pts[0];
        controls.pts[index] = { v.dy * adj - v.dx * opp, v.dy * opp + v.dx * adj };
    }
    controls.copyTo(result);
}

inline void cubicSetBounds(Curve c, OpRect& bounds) {
    CubicControls controls(c);
    bounds.add(controls.pts[0]);
    bounds.add(controls.pts[1]);
}

inline void cubicSubDivide(Curve c, OpPtT ptT1, OpPtT ptT2, Curve result) {
    result.data->start = ptT1.pt;
    result.data->end = ptT2.pt;
    CubicControls controls(c);
    CubicControls subControl = CubicControlPt(c.data->start, controls, c.data->end, ptT1, ptT2);
    subControl.copyTo(result);
}

inline OpPoint cubicHull(Curve c, int index) {
    if (1 == index || 2 == index)
        return CubicControls(c).pts[index - 1];
    OP_ASSERT(0); // should never be called
    return OpPoint();
}

inline void cubicReverse(Curve c) {
    CubicControls controls(c);
    std::swap(controls.pts[0], controls.pts[1]);
    controls.copyTo(c);
}

#if OP_DEBUG_DUMP
inline std::string cubicDebugDumpName() { 
    return "cubic"; 
}
#endif

}
