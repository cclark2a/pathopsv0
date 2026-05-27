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

    double choice(Axis axis) {
        return Axis::vertical == axis ? x : y;
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

// used in curves
struct SPointWeight {
    SPointWeight() {
    }

    SPointWeight(Curve c) {
        OP_ASSERT(sizeof(SPointWeight) <= c.size - CurveUserDataOffset());
        const char* data = (const char*) CurveUserData(c.data);
        std::memcpy(&pt, data, sizeof pt);
        data += sizeof pt;
        std::memcpy(&weight, data, sizeof weight);
    }

    SPointWeight(OpPoint p, float w)
        : pt(p)
        , weight(w) {
    }

    void copyTo(Curve c) {
        OP_ASSERT(sizeof(SPointWeight) <= c.size - CurveUserDataOffset());
        char* data = (char*) CurveUserData(c.data);
        std::memcpy(data, &pt, sizeof pt);
        data += sizeof pt;
        std::memcpy(data, &weight, sizeof weight);
    }

    OpPoint pt;
    float weight;
};

// used with computed double points
struct DPointWeight {
    DPointWeight() {
    }

    DPointWeight(Curve c) {
        SPointWeight sPtW(c);
        pt = sPtW.pt;
        weight = sPtW.weight; 
    }

    DPointWeight(OpDPoint p, float w)
        : pt(p)
        , weight(w) {
    }

    void copyTo(Curve c) {
        SPointWeight sPtW({ (float) pt.x, (float) pt.y }, weight);
        sPtW.copyTo(c);
    }

    OpDPoint pt;
    float weight;
};

struct DQuadCoefficients {
	double a;
	double b;
	double c;
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

inline DQuadCoefficients DerivativeCoefficients(
        OpPoint start, SPointWeight control, OpPoint end, XyChoice offset) {
    double P20 = end.choice(offset) - start.choice(offset);
    double P10 = control.pt.choice(offset) - start.choice(offset);
    double wP10 = control.weight * P10;
    double a = control.weight * P20 - P20;
    double b = P20 - 2 * wP10;
    double c = wP10;
    return { a, b, c };
}

inline float DConicTangent(OpPoint start, SPointWeight control, OpPoint end, XyChoice offset, 
        float t) {
    DQuadCoefficients coeff = DerivativeCoefficients(start, control, end, offset);
    return (float) (t * (t * coeff.a + coeff.b) + coeff.c);
}

inline OpVector DConicTangent(OpPoint start, SPointWeight control, OpPoint end, float t) {
    OpVector threshold = OpMath::Threshold(start, end);
    if ((OpMath::NearlyZeroT(t) && start.isNearly(OpPoint(control.pt.x, control.pt.y), threshold))
            || (OpMath::NearlyOneT(t) && end.isNearly(OpPoint(control.pt.x, control.pt.y), threshold)))
        return end - start;
    return { DConicTangent(start, control, end, XyChoice::inX, t), 
            DConicTangent(start, control, end, XyChoice::inY, t) };
}

inline std::vector<float> AddExtrema(OpPoint start, OpPoint end, SPointWeight control,
		bool monotonicInX, bool monotonicInY) {
    std::vector<float> tValues;
    auto addExtrema = [start, control, end, &tValues](XyChoice offset) {
        DQuadCoefficients dc = DerivativeCoefficients(start, control, end, offset);
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

struct DConicSwizzled {
    OpPoint curveData[2];
    SPointWeight ctrlWeight;
};

#if OP_TEST
struct DebugDConic {
    CurveType curveType;
    size_t curveSize;
    DConicSwizzled curveData;
    float extrema[2];
};
#endif

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the DConicratic Bezier
inline size_t AddDConics(Contour* contour, AddCurve curve) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[2];
    float weight = curve.points[3].x;  // !!! a bit of a hack
    SPointWeight control(curve.points[1], weight);
    DConicSwizzled swizzled { { start, end }, { { control.pt.x, control.pt.y }, weight } };
    Curve conic { curve.context, (CurveData*) &swizzled, sizeof swizzled, curve.type };
#if OP_TEST
    // save original curve and extrema t values as debugging data for visualization
    auto setDebugDConic = [contour, swizzled, &conic](std::vector<float>* tValues) {
        OP_ASSERT(sizeof(swizzled) == sizeof(DebugDConic::curveData));
        DebugDConic debugDConic { conic.type, sizeof(swizzled) };
        memcpy(&debugDConic.curveData, &swizzled, sizeof(swizzled));
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
        Add(contour, conic);
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
    DPointWeight dControl { control.pt, control.weight };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { DConicPointAtT(start, dControl, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        if (ptTs[index].pt == ptTs[index + 1].pt)
            continue;
        DPointWeight dPtW = DConicControl(start, dControl, end, ptTs[index], ptTs[index + 1]);
        struct DConicData {
            OpPoint endPts[2];
            SPointWeight control;
        } curveData { { ptTs[index].pt, ptTs[index + 1].pt } };
        curveData.control = { { (float) dPtW.pt.x, (float) dPtW.pt.y }, dPtW.weight };
        Add(contour, { curve.context, curveData.endPts, sizeof curveData, curve.type } );
    }
    return curvesAdded;
}

// callback functions
inline bool dConicIsFinite(Curve c) {
    SPointWeight control(c);
    return control.pt.isFinite();
}

inline int dConicHullPtCount() {
    return 1;
}

inline bool dConicIsLine(Curve c, float threshold) {
    SPointWeight control(c);
    LinePts linePts { c.data->start, c.data->end };
    return linePts.ptOnLine(control.pt, threshold);
}

inline OpRoots dConicAxisT(Curve curve, Axis axis, float intercept  
		OP_DEBUG_PARAMS(const OpRoots& )) {
    DPointWeight control(curve);
    double a = curve.data->end.choice(axis);
    double b = control.pt.choice(axis) * control.weight - intercept * control.weight + intercept;
    double c = curve.data->start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    OpRoots result = OpMath::QuadRootsDouble(a, 2 * b, c - intercept);  // ? double req'd: testDConics3759897
    result = result.keepValidT();
    return result;
    
}

inline OpRoots dConicRotatedT(Curve curve, Axis axis, float intercept
		OP_DEBUG_PARAMS(const OpRoots& debugAdded)) {
	OpPoint start = curve.data->start;
	OpPoint end = curve.data->end;
    SPointWeight control(curve);
    bool monotonicInX = OpMath::Between(start.x, control.pt.x, end.x);
    bool monotonicInY = OpMath::Between(start.y, control.pt.y, end.y);
	std::vector<float> tValues = AddExtrema(start, end, control, monotonicInX, monotonicInY);
	if (tValues.empty())
		return dConicAxisT(curve, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size() + 2);
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
	DPointWeight dControl(curve);
    for (unsigned index = 0; index < tValues.size(); ++index) {
        ptTs[index + 1] = { DConicPointAtT(start, dControl, end, tValues[index]), tValues[index] }; 
    } 
	OpRoots result;
	unsigned lastIndex = (unsigned) (ptTs.size() - 1);
    for (unsigned index = 0; index < lastIndex; ++index) {
        struct DConicData {
            OpPoint endPts[2];
            SPointWeight control;
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
        DPointWeight dPtW = DConicControl(start, dControl, end, ptTs[index], ptTs[index + 1]);
        conicData.control = { { (float) dPtW.pt.x, (float) dPtW.pt.y }, dPtW.weight };
		Curve part { curve.context, (CurveData*) &conicData, curve.size, curve.type };
		OpRoots partRoot = dConicAxisT(part, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
		for (float root : partRoot.roots)
			result.add(startT + root * (endT - startT));
	}
	return result;
}

inline OpPoint dConicPtAtT(Curve c, float t) {
    DPointWeight control(c);
    return DConicPointAtT(c.data->start, control, c.data->end, t);
}

inline OpPair dConicXYAtT(Curve c, OpPair t, XyChoice xyChoice) {
    DPointWeight control(c);
    return DConicXYAtT(c.data->start, control, c.data->end, t, xyChoice);
}

inline bool dConicsEqual(Curve one, Curve two) {
    SPointWeight ctrl1(one);
    SPointWeight ctrl2(two);
    return ctrl1.pt.x == ctrl2.pt.x && ctrl1.pt.y == ctrl2.pt.y && ctrl1.weight == ctrl2.weight;
}

inline void dConicPin(Curve c, OpPoint newStart, OpPoint newEnd) {
    SPointWeight control(c);
    control.pt.pin(newStart, newEnd);
    control.copyTo(c);
 }

inline OpVector dConicTangent(Curve c, float t) {
    SPointWeight control(c);
    return DConicTangent(c.data->start, control, c.data->end, t);
}

#if 0
inline OpVector conicNormal(Curve c, float t) {
    OpVector tan = conicTangent(c, t);
    return { -tan.dy, tan.dx };
}
#endif

inline void dConicRotate(Curve c, OpPoint origin, OpVector scale, Curve result) {
    SPointWeight control(c);
    OpVector v = control.pt - origin;
    SPointWeight rotated({ scale.cross(v), scale.dot(v) }, control.weight);
    rotated.copyTo(result);
}

inline void dConicSetBounds(Curve c, OpRect& bounds) {
    SPointWeight control(c);
    bounds.add(control.pt);
}

inline void dConicSubDivide(Curve curve, float t1, float t2, OpVector threshold, Curve* result) {
	OpPtT ptT1 { result->data->start, t1 };
	OpPtT ptT2 { result->data->end, t2 };
    DPointWeight control(curve);
    DPointWeight subPtW = DConicControl(curve.data->start, control, curve.data->end, ptT1, ptT2);
    subPtW.copyTo(*result);
    if (dConicIsLine(*result, threshold.length()))
        result->type = degenerateLine;
}

inline OpPoint dConicHull(Curve c, int index) {
    if (1 == index)
        return SPointWeight(c).pt;
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

inline std::string dConicDebugDumpExtra(Curve c, DebugLevel l, DebugBase b) {
    DPointWeight control(c);
    return debugValue(l, b, " weight", control.weight);
}

#define DUMP_DCONIC_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(dConicDebugDumpExtra), \
    OP_TAGGED_FUNCTION(debugDConicSubDivide), \

#endif

#if OP_DEBUG_SERIALIZE
inline std::string dConicDebugDumpName() { 
    return "dconic"; 
}

#define DCONIC_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(dConicAxisT), \
    OP_TAGGED_FUNCTION(dConicRotatedT), \
    OP_TAGGED_FUNCTION(dConicHull), \
    OP_TAGGED_FUNCTION(dConicIsFinite), \
    OP_TAGGED_FUNCTION(dConicIsLine), \
    OP_TAGGED_FUNCTION(dConicSetBounds), \
    OP_TAGGED_FUNCTION(dConicPin), \
    OP_TAGGED_FUNCTION(dConicTangent), \
    OP_TAGGED_FUNCTION(dConicsEqual), \
    OP_TAGGED_FUNCTION(dConicPtAtT), \
    OP_TAGGED_FUNCTION(dConicHullPtCount), \
	OP_TAGGED_FUNCTION(dConicRotate), \
    OP_TAGGED_FUNCTION(dConicSubDivide), \
    OP_TAGGED_FUNCTION(dConicXYAtT), \
    OP_TAGGED_FUNCTION(dConicDebugDumpName), \
    
#endif

inline void dConicCallbacks(Context* context, int nativeCurveType) {
    SetCurveCallbacks(context, nativeCurveType, { dConicAxisT,
			dConicRotatedT, dConicHull, dConicIsFinite, dConicIsLine, dConicSetBounds, dConicPin,
			dConicTangent, dConicsEqual, dConicPtAtT, nullptr, dConicHullPtCount, dConicRotate, 
			dConicSubDivide, dConicXYAtT });
#if OP_TEST
	SetDebugCurveCallbacks(context, nativeCurveType, { debugDConicScale
            OP_DEBUG_DUMP_PARAMS(dConicDebugDumpName, dConicDebugDumpExtra, debugDConicSubDivide)
//            OP_DEBUG_RASTER_PARAMS(debugRasterAdd)
            });
#endif
}

}

#endif
