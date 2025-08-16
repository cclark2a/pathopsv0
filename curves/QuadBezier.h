// (c) 2024, Cary Clark cclark2@gmail.com

#include "PathOps.h"
#if OP_DEBUG
#include "DebugOps.h"
#endif

namespace PathOpsV0Lib {

// !!! optimization: quadPointAtT (lower case), called by OpCurve::ptAtT has checked for 0, 1
// so no need to check again
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
    OpVector diff = 2 * midPt - avgPt;
    OpPoint result(diff.dx, diff.dy);
    result.pin(ptT1.pt, ptT2.pt);
    return result;
}

inline OpVector QuadTangent(OpPoint start, OpPoint control, OpPoint end, float t) {
    OpVector threshold = OpMath::Threshold(start, end);
    if ((OpMath::NearlyZeroT(t) && start.isNearly(control, threshold))
            || (OpMath::NearlyOneT(t) && end.isNearly(control, threshold)))
        return end - start;
    float a = t - 1;
    float b = 1 - 2 * t;
    float c = t;
    OpPoint sum = a * start + b * control + c * end;
    return OpVector(sum.x, sum.y);
}

// break out extrema so it can be called by quadRotatedT
inline std::vector<float> AddExtrema(OpPoint start, OpPoint end, OpPoint control,
		bool monotonicInX, bool monotonicInY) {
    std::vector<float> tValues;
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
	return tValues;
}

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the quadratic Bezier
inline size_t AddQuads(Contour* contour, AddCurve curve) {
    OpPoint start = curve.points[0];
    OpPoint control = curve.points[1];
    OpPoint end = curve.points[2];
    auto [left, right] = std::minmax(start.x, end.x);
    bool monotonicInX = left <= control.x && control.x <= right;
    auto [top, bottom] = std::minmax(start.y, end.y);
    bool monotonicInY = top <= control.y && control.y <= bottom;
    if (monotonicInX && monotonicInY) {
        if (start == end)
            return 0;
        // swizzle input to match v0's start/end/ctrl layout
        OpPoint swizzled[3] { start, end, control };
        Add(contour, { curve.context, swizzled, curve.size, curve.type } );
        return 1;
    }
    // control point is not inside bounds formed by end points; split quad into parts
	std::vector<float> tValues = AddExtrema(start, end, control, monotonicInX, monotonicInY);
	tValues.push_back(0);
	tValues.push_back(1);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { QuadPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        if (ptTs[index].pt == ptTs[index + 1].pt)
            continue;
        OpPoint curveData[3] { ptTs[index].pt, ptTs[index + 1].pt,
            QuadControlPt(start, control, end, ptTs[index], ptTs[index + 1]) };
        Add(contour, { curve.context, curveData, curve.size, curve.type } );
    }
    return curvesAdded;
}

inline OpPoint quadControlPt(Curve c) {
    OpPoint result;
    OP_ASSERT(sizeof(OpPoint) <= c.size - CurveUserDataOffset());
    std::memcpy(&result, CurveUserData(c.data), sizeof(OpPoint));
    return result;
}

inline void quadSetControl(Curve c, OpPoint pt) {
    OP_ASSERT(sizeof(OpPoint) <= c.size - CurveUserDataOffset());
    std::memcpy(CurveUserData(c.data), &pt, sizeof(OpPoint));
}

// callback functions
inline int quadHullPtCount() {
    return 1;
}

inline bool quadIsFinite(Curve c) {
    return quadControlPt(c).isFinite();
}

inline bool quadIsLine(Curve c) {
    OpPoint ctrlPt = quadControlPt(c);
    LinePts linePts = { c.data->start, c.data->end };
    return linePts.ptOnLine(ctrlPt);
}

inline OpRoots quadAxisT(Curve curve, Axis axis, float axisIntercept
		OP_DEBUG_PARAMS(const OpRoots& )) {
    float a = curve.data->end.choice(axis);
    float b = quadControlPt(curve).choice(axis);
    float c = curve.data->start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    OpRoots result = OpMath::QuadRootsDouble(a, 2 * b, c - axisIntercept);  // double req'd: testQuads3759897
    result = result.keepValidT();
    return result;
}

inline OpRoots quadRotatedT(Curve curve, Axis axis, float intercept  
		OP_DEBUG_PARAMS(const OpRoots& debugAdded)) {
	OpPoint start = curve.data->start;
	OpPoint end = curve.data->end;
	OpPoint control = quadControlPt(curve);
    bool monotonicInX = OpMath::Between(start.x, control.x, end.x);
    bool monotonicInY = OpMath::Between(start.y, control.y, end.y);
	std::vector<float> tValues = AddExtrema(start, end, control, monotonicInX, monotonicInY);
	if (tValues.empty())
		return quadAxisT(curve, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size() + 2);
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 0; index < tValues.size(); ++index) {
        ptTs[index + 1] = { QuadPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
	OpRoots result;
	unsigned lastIndex = ptTs.size() - 1;
    for (unsigned index = 0; index < lastIndex; ++index) {
        OpPoint curveData[3] { ptTs[index].pt, ptTs[index + 1].pt };
		float startT = ptTs[index].t;
		float endT = ptTs[index + 1].t;
		if (OpMath::Equal(intercept, curveData[0].choice(axis))) {
			result.add(startT);
			continue;
		}
		if (OpMath::Equal(intercept, curveData[1].choice(axis))) {
			result.add(endT);
			continue;
		}
		if (curveData[0].choice(axis) * curveData[1].choice(axis) > 0)
			continue;
        OP_ASSERT(curveData[0] != curveData[1]);
        curveData[2] = QuadControlPt(start, control, end, ptTs[index], ptTs[index + 1]);
		Curve part { curve.context, (CurveData*) curveData, curve.size, curve.type };
		OpRoots partRoot = quadAxisT(part, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
		for (float root : partRoot.roots)
			result.add(startT + root * (endT - startT));
	}
	return result;
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

inline void quadRotate(Curve c, OpPoint origin, OpVector s, Curve result) {
    OpPoint ctrlPt = quadControlPt(c);
    OpVector v = ctrlPt - origin;
#if 1  // disabling this triggers an error in testQuads26021089
	OpPoint rotated(s.cross(v), s.dot(v));
#else
    OpPoint rotated(v.dy * s.dx - v.dx * s.dy, v.dy * s.dy + v.dx * s.dx);
#endif
    quadSetControl(result, rotated);
}

inline void quadSetBounds(Curve c, OpRect& bounds) {
    bounds.add(quadControlPt(c));
}

inline void quadSubDivide(Curve c, float t1, float t2, float threshold, Curve* result) {
	OpPtT ptT1 { result->data->start, t1 };
	OpPtT ptT2 { result->data->end, t2 };
    OpPoint subControl = QuadControlPt(c.data->start, quadControlPt(c), c.data->end, ptT1, ptT2);
    quadSetControl(*result, subControl);
    if (quadIsLine(*result))
        result->type = degenerateLine;
}

inline OpPoint quadHull(Curve c, int index) {
    if (1 == index)
        return quadControlPt(c);
    OP_ASSERT(0); // should never be called
    return OpPoint();
}

inline void quadCallbacks(Context* context, int nativeCurveType) {
    SetCurveCallbacks(context, nativeCurveType, 
            { quadAxisT, quadRotatedT, quadHull, quadIsFinite, 
            quadIsLine, quadSetBounds, quadTangent, quadsEqual, quadPtAtT, nullptr, quadHullPtCount,  
			quadRotate, quadSubDivide, quadXYAtT });
}

#if OP_DEBUG_DUMP
inline std::string quadDebugDumpName() { 
    return "quad"; 
}
#endif

}
