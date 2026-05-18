// (c) 2026, Cary Clark cclark2@gmail.com
// Double version experiment: see if error can be reduced here
#ifndef DConicBezier_DEFINED
#define DConicBezier_DEFINED

#include "PathOps.h"
#include "DebugOps.h"
#include "OpDebugRaster.h"

namespace PathOpsV0Lib {

    struct OpDPoint {
    OpDPoint(double _x, double _y) :
        x(_x),
        y(_y) {
    }

    OpDPoint(OpPoint pt) :
        x(pt.x),
        y(pt.y) {
    }

    OpDPoint() : 
        x(OpNaN), 
        y(OpNaN) {
    }

    double choice(XyChoice xyChoice) {
        return XyChoice::inX == xyChoice ? x : y;
    }

    void pin(const OpDPoint a, const OpDPoint b) {
        x = OpMath::PinUnsorted(a.x, x, b.x);
        y = OpMath::PinUnsorted(a.y, y, b.y);
    }

    double x;
    double y;
};

// used with computed double points
struct DPointWeight {
    DPointWeight() {
    }

    DPointWeight(Curve c) {
        OP_ASSERT(sizeof(DPointWeight) <= c.size - CurveUserDataOffset());
        const char* data = (const char*) CurveUserData(c.data);
        std::memcpy(&pt, data, sizeof pt);
        data += sizeof pt;
        std::memcpy(&weight, data, sizeof weight);
    }

    DPointWeight(OpDPoint p, float w)
        : pt(p)
        , weight(w) {
    }

    void copyTo(Curve c) {
        OP_ASSERT(sizeof(DPointWeight) <= c.size - CurveUserDataOffset());
        char* data = (char*) CurveUserData(c.data);
        std::memcpy(data, &pt, sizeof pt);
        data += sizeof pt;
        std::memcpy(data, &weight, sizeof weight);
    }

    OpPoint sPt() const {
        return { (float) pt.x, (float) pt.y };
    }

    OpDPoint pt;
    float weight;
};

inline double DConicDenom(float weight, float t) {
    double B = 2 * (weight - 1);
    double C = 1;
    double A = -B;
    return (A * t + B) * t + C;
}

inline OpDPoint DConicNumer(OpPoint start, DPointWeight control, OpPoint end, float t) {
    OpDPoint pt1w { control.pt.x * control.weight, control.pt.y * control.weight };
    OpDPoint C = start;
    OpDPoint A { C.x + (end.x - 2 * pt1w.x), C.y + (end.y - 2 * pt1w.y) };
    OpDPoint B { 2 * (pt1w.x - C.x), 2 * (pt1w.y - C.y) };
    return OpDPoint((A.x * t + B.x) * t + C.x, (A.y * t + B.y) * t + C.y);
}

inline OpPoint DConicPointAtT(OpPoint start, DPointWeight control, OpPoint end, float t) {
    if (0 == t)
        return start;
    if (1 == t)
        return end;
    OpDPoint dPt = DConicNumer(start, control, end, t);
    double denom = DConicDenom(control.weight, t);
    return { (float) (dPt.x / denom), (float) (dPt.y / denom) };
}

// given a pair of t values, return a pair of x values
// !!! implementation is non-optimal; but see if it works at all, and wonder, who uses conics?
inline OpPair DConicXYAtT(OpPoint s, DPointWeight c, OpPoint e, OpPair t, XyChoice xy) {
    return { DConicPointAtT(s, c, e, t.s).choice(xy), DConicPointAtT(s, c, e, t.l).choice(xy) };
}

inline DPointWeight DConicControl(OpPoint start, DPointWeight control, OpPoint end, OpPtT ptT1, 
        OpPtT ptT2  OP_DEBUG_DUMP_PARAMS(bool debugSubDivide = false)) {
    if (0 == ptT1.t && 1 == ptT2.t)
        return control;
    auto subWeight = [start, control, end, ptT1, ptT2](float t) {
        if (t == 0)
            return DPointWeight(ptT1.pt, 1);
        if (t == 1)
            return DPointWeight(ptT2.pt, 1);
        return DPointWeight(DConicNumer(start, control, end, t), DConicDenom(control.weight, t));
    };
    DPointWeight a = subWeight(ptT1.t);
    DPointWeight c = subWeight(ptT2.t);
    float midT = OpMath::Average(ptT1.t, ptT2.t);
    DPointWeight d(DConicNumer(start, control, end, midT), DConicDenom(control.weight, midT));
    OpDPoint pwVec { 2 * d.pt.x - (a.pt.x + c.pt.x) / 2, 2 * d.pt.y - (a.pt.y + c.pt.y) / 2 };
    DPointWeight b(OpDPoint(pwVec.x, pwVec.y),  // !!! add math pt average?
            2 * d.weight - OpMath::Average(a.weight, c.weight));  // !!! rewrite with fma?
    // if bz is 0, weight is 0, control point has no effect: any value will do
    float bzNonZero = !b.weight ? 1 : b.weight;
    if (!b.weight)
        b.weight = 1;
    DPointWeight result({ b.pt.x / bzNonZero, b.pt.y / bzNonZero }, b.weight / sqrtf(a.weight * c.weight));
    if (ptT1.pt.isFinite() && ptT2.pt.isFinite()  OP_DEBUGGER_CODE(&& !debugSubDivide))
        result.pt.pin(ptT1.pt, ptT2.pt);
    return result;
}

inline OpQuadCoefficients DerivativeCoefficients(
        OpPoint start, DPointWeight control, OpPoint end, XyChoice offset) {
    float P20 = end.choice(offset) - start.choice(offset);
    float P10 = control.pt.choice(offset) - start.choice(offset);
    float wP10 = control.weight * P10;
    float a = control.weight * P20 - P20;
    float b = P20 - 2 * wP10;
    float c = wP10;
    return { a, b, c };
}

inline float DConicTangent(OpPoint start, DPointWeight control, OpPoint end, XyChoice offset, 
        float t) {
    OpQuadCoefficients coeff = DerivativeCoefficients(start, control, end, offset);
    return t * (t * coeff.a + coeff.b) + coeff.c;
}

inline OpVector DConicTangent(OpPoint start, DPointWeight control, OpPoint end, float t) {
    OpVector threshold = OpMath::Threshold(start, end);
    if ((OpMath::NearlyZeroT(t) && start.isNearly(OpPoint(control.pt.x, control.pt.y), threshold))
            || (OpMath::NearlyOneT(t) && end.isNearly(OpPoint(control.pt.x, control.pt.y), threshold)))
        return end - start;
    return { DConicTangent(start, control, end, XyChoice::inX, t), 
            DConicTangent(start, control, end, XyChoice::inY, t) };
}

inline std::vector<float> AddExtrema(OpPoint start, OpPoint end, DPointWeight control,
		bool monotonicInX, bool monotonicInY) {
    std::vector<float> tValues;
    auto addExtrema = [start, control, end, &tValues](XyChoice offset) {
        OpQuadCoefficients dc = DerivativeCoefficients(start, control, end, offset);
        OpRoots roots = OpMath::QuadRootsInteriorT(dc.a, dc.b, dc.c);
        OP_ASSERT(0 == roots.count() || 1 == roots.count() || RootFail::rootIsNaN == roots.fail);   // !!! I wanna see the extreme case...
        if (0 == roots.count())
            return;
        tValues.push_back(roots.roots[0]);
    };
    if (!monotonicInX)
        addExtrema(XyChoice::inX);
    if (!monotonicInY)
        addExtrema(XyChoice::inY);
    return tValues;
}

#if OP_TEST
struct DebugDConic {
    CurveType curveType;
    size_t curveSize;
    OpPoint curveData[4];
    float extrema[2];
};
#endif

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the DConicratic Bezier
inline size_t AddDConics(Contour* contour, AddCurve curve) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[2];
    float weight = curve.points[3].x;  // !!! a bit of a hack
    DPointWeight control(curve.points[1], weight);
    OpPoint swizzled[4] { start, end, OpPoint(control.pt.x, control.pt.y), { weight, 0 } };
    Curve conic { curve.context, (CurveData*) swizzled, curve.size, curve.type };
#if OP_TEST
    // save original curve and extrema t values as debugging data for visualization
    auto setDebugDConic = [contour, swizzled, &conic](std::vector<float>* tValues) {
        OP_ASSERT(sizeof(swizzled) == sizeof(DebugDConic::curveData));
        DebugDConic debugDConic { conic.type, sizeof(swizzled) };
        memcpy(debugDConic.curveData, swizzled, sizeof(swizzled));
        size_t extremaCount = tValues ? tValues->size() : 0;
        for (size_t index = 0; index < ARRAY_COUNT(DebugDConic::extrema); ++index) {
            debugDConic.extrema[index] = index < extremaCount ? (*tValues)[index] : OpNaN;
        }
        SetDebugCurveData(contour, { (DebugCurve*) &debugDConic, sizeof(debugDConic) });
    };
#endif
    auto [left, right] = std::minmax(start.x, end.x);
    bool monotonicInX = left <= control.pt.x && control.pt.x <= right;
    auto [top, bottom] = std::minmax(start.y, end.y);
    bool monotonicInY = top <= control.pt.y && control.pt.y <= bottom;
    if (monotonicInX && monotonicInY) {
#if OP_TEST
        setDebugDConic(nullptr);
#endif
        if (start == end)
            return 0;
        Add(contour, { curve.context, swizzled, curve.size, curve.type } );
        return 1;
    }
    // control point is not inside bounds formed by end points; split DConic into parts
	std::vector<float> tValues = AddExtrema(start, end, control, monotonicInX, monotonicInY);
#if OP_TEST
    OP_ASSERT(sizeof(tValues[0]) == sizeof(DebugDConic::extrema[0]));
    OP_ASSERT(tValues.size() <= ARRAY_COUNT(DebugDConic::extrema));
    setDebugDConic(&tValues);
#endif
	tValues.push_back(0);
	tValues.push_back(1);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { DConicPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        if (ptTs[index].pt == ptTs[index + 1].pt)
            continue;
        struct DConicData {
            OpPoint endPts[2];
            DPointWeight control;
        } curveData { { ptTs[index].pt, ptTs[index + 1].pt },
                DConicControl(start, control, end, ptTs[index], ptTs[index + 1]) };
            Add(contour, { curve.context, curveData.endPts, curve.size, curve.type } );
    }
    return curvesAdded;
}

// callback functions
inline bool conicIsFinite(Curve c) {
    DPointWeight control(c);
    return control.sPt().isFinite();
}

inline int conicHullPtCount() {
    return 1;
}

inline bool conicIsLine(Curve c, float threshold) {
    DPointWeight control(c);
    LinePts linePts { c.data->start, c.data->end };
    return linePts.ptOnLine(control.sPt(), threshold);
}

inline OpRoots conicAxisT(Curve curve, Axis axis, float intercept  
		OP_DEBUG_PARAMS(const OpRoots& )) {
    DPointWeight control(curve);
    float a = curve.data->end.choice(axis);
    float b = control.sPt().choice(axis) * control.weight - intercept * control.weight + intercept;
    float c = curve.data->start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    OpRoots result = OpMath::QuadRootsDouble(a, 2 * b, c - intercept);  // ? double req'd: testDConics3759897
    result = result.keepValidT();
    return result;
    
}

inline OpRoots conicRotatedT(Curve curve, Axis axis, float intercept
		OP_DEBUG_PARAMS(const OpRoots& debugAdded)) {
	OpPoint start = curve.data->start;
	OpPoint end = curve.data->end;
	DPointWeight control(curve);
    bool monotonicInX = OpMath::Between(start.x, control.pt.x, end.x);
    bool monotonicInY = OpMath::Between(start.y, control.pt.y, end.y);
	std::vector<float> tValues = AddExtrema(start, end, control, monotonicInX, monotonicInY);
	if (tValues.empty())
		return conicAxisT(curve, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size() + 2);
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 0; index < tValues.size(); ++index) {
        ptTs[index + 1] = { DConicPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
	OpRoots result;
	unsigned lastIndex = (unsigned) (ptTs.size() - 1);
    for (unsigned index = 0; index < lastIndex; ++index) {
        struct DConicData {
            OpPoint endPts[2];
            DPointWeight control;
        } conicData { { ptTs[index].pt, ptTs[index + 1].pt }, {} };
		float startT = ptTs[index].t;
		float endT = ptTs[index + 1].t;
		if (OpMath::Equal(intercept, conicData.endPts[0].choice(axis))) {
			result.add(startT);
			continue;
		}
		if (OpMath::Equal(intercept, conicData.endPts[1].choice(axis))) {
			result.add(endT);
			continue;
		}
		if (conicData.endPts[0].choice(axis) * conicData.endPts[1].choice(axis) > 0)
			continue;
        OP_ASSERT(conicData.endPts[0] != conicData.endPts[1]);
        conicData.control = DConicControl(start, control, end, ptTs[index], ptTs[index + 1]);
		Curve part { curve.context, (CurveData*) &conicData, curve.size, curve.type };
		OpRoots partRoot = conicAxisT(part, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
		for (float root : partRoot.roots)
			result.add(startT + root * (endT - startT));
	}
	return result;
}

inline OpPoint conicPtAtT(Curve c, float t) {
    DPointWeight control(c);
    return DConicPointAtT(c.data->start, control, c.data->end, t);
}

inline OpPair conicXYAtT(Curve c, OpPair t, XyChoice xyChoice) {
    DPointWeight control(c);
    return DConicXYAtT(c.data->start, control, c.data->end, t, xyChoice);
}

inline bool conicsEqual(Curve one, Curve two) {
    DPointWeight ctrl1(one);
    DPointWeight ctrl2(two);
    return ctrl1.pt.x == ctrl2.pt.x && ctrl1.pt.y == ctrl2.pt.y && ctrl1.weight == ctrl2.weight;
}

inline void conicPin(Curve c, OpPoint newStart, OpPoint newEnd) {
    DPointWeight control(c);
    control.pt.pin(newStart, newEnd);
    control.copyTo(c);
 }

inline OpVector conicTangent(Curve c, float t) {
    DPointWeight control(c);
    return DConicTangent(c.data->start, control, c.data->end, t);
}

#if 0
inline OpVector conicNormal(Curve c, float t) {
    OpVector tan = conicTangent(c, t);
    return { -tan.dy, tan.dx };
}
#endif

inline void conicRotate(Curve c, OpPoint origin, OpVector scale, Curve result) {
    DPointWeight control(c);
    OpVector v = control.sPt() - origin;
#if 1
    DPointWeight rotated({ scale.cross(v), scale.dot(v) }, control.weight);
#else
    DPointWeight rotated({ v.dy * s.dx - v.dx * s.dy, v.dy * s.dy + v.dx * s.dx }, control.weight);
#endif
    rotated.copyTo(result);
}

inline void conicSetBounds(Curve c, OpRect& bounds) {
    DPointWeight control(c);
    bounds.add(control.sPt());
}

inline void conicSubDivide(Curve curve, float t1, float t2, OpVector threshold, Curve* result) {
	OpPtT ptT1 { result->data->start, t1 };
	OpPtT ptT2 { result->data->end, t2 };
    DPointWeight control(curve);
    DPointWeight subPtW = DConicControl(curve.data->start, control, curve.data->end, ptT1, ptT2);
    subPtW.copyTo(*result);
    if (conicIsLine(*result, threshold.length()))
        result->type = degenerateLine;
}

inline OpPoint conicHull(Curve c, int index) {
    if (1 == index)
        return DPointWeight(c).sPt();
    OP_ASSERT(0); // should never be called
    return OpPoint();
}

#if OP_DEBUG_DUMP
inline void debugDConicSubDivide(Curve curve, float t1, float t2, Curve* result) {
	OpPtT ptT1 { result->data->start, t1 };
	OpPtT ptT2 { result->data->end, t2 };
    DPointWeight control(curve);
    DPointWeight subPtW = DConicControl(curve.data->start, control, curve.data->end, ptT1, ptT2, true);
    subPtW.copyTo(*result);
}

inline std::string conicDebugDumpExtra(Curve c, DebugLevel l, DebugBase b) {
    DPointWeight control(c);
    return debugValue(l, b, " weight", control.weight);
}

#define DUMP_CONIC_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(conicDebugDumpExtra), \
    OP_TAGGED_FUNCTION(debugDConicSubDivide), \

#endif

#if OP_DEBUG_SERIALIZE
inline std::string conicDebugDumpName() { 
    return "conic"; 
}

#define CONIC_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(conicAxisT), \
    OP_TAGGED_FUNCTION(conicRotatedT), \
    OP_TAGGED_FUNCTION(conicHull), \
    OP_TAGGED_FUNCTION(conicIsFinite), \
    OP_TAGGED_FUNCTION(conicIsLine), \
    OP_TAGGED_FUNCTION(conicSetBounds), \
    OP_TAGGED_FUNCTION(conicPin), \
    OP_TAGGED_FUNCTION(conicTangent), \
    OP_TAGGED_FUNCTION(conicsEqual), \
    OP_TAGGED_FUNCTION(conicPtAtT), \
    OP_TAGGED_FUNCTION(conicHullPtCount), \
	OP_TAGGED_FUNCTION(conicRotate), \
    OP_TAGGED_FUNCTION(conicSubDivide), \
    OP_TAGGED_FUNCTION(conicXYAtT), \
    OP_TAGGED_FUNCTION(conicDebugDumpName), \
    
#endif

inline void conicCallbacks(Context* context, int nativeCurveType) {
    SetCurveCallbacks(context, nativeCurveType, { conicAxisT,
			conicRotatedT, conicHull, conicIsFinite, conicIsLine, conicSetBounds, conicPin,
			conicTangent, conicsEqual, conicPtAtT, nullptr, conicHullPtCount, conicRotate, 
			conicSubDivide, conicXYAtT });
#if OP_TEST

	SetDebugCurveCallbacks(context, nativeCurveType, { nullptr
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpName, conicDebugDumpExtra, debugDConicSubDivide)
//            OP_DEBUG_RASTER_PARAMS(debugRasterAdd)
            });
#endif
}

}

#endif
