// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef ConicBezier_DEFINED
#define ConicBezier_DEFINED

#include "PathOps.h"
#include "DebugOps.h"
#include "OpDebugRaster.h"

namespace PathOpsV0Lib {

struct PointWeight {
    PointWeight() {
    }

    PointWeight(Curve c) {
        OP_ASSERT(sizeof(PointWeight) <= c.size - CurveUserDataOffset());
        const char* data = (const char*) CurveUserData(c.data);
        std::memcpy(&pt, data, sizeof pt);
        data += sizeof pt;
        std::memcpy(&weight, data, sizeof weight);
    }

    PointWeight(OpPoint p, float w)
        : pt(p)
        , weight(w) {
    }

    void copyTo(Curve c) {
        OP_ASSERT(sizeof(PointWeight) <= c.size - CurveUserDataOffset());
        char* data = (char*) CurveUserData(c.data);
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
    OpPoint A = C + (end - 2 * pt1w);
    OpVector B = 2 * (pt1w - C);
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

inline PointWeight ConicControl(OpPoint start, PointWeight control, OpPoint end, OpPtT ptT1, 
        OpPtT ptT2  OP_DEBUG_DUMP_PARAMS(bool debugSubDivide = false)) {
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
    OpVector pwVec = 2 * d.pt - (a.pt + c.pt) / 2;
    PointWeight b(OpPoint(pwVec.dx, pwVec.dy),  // !!! add math pt average?
            2 * d.weight - OpMath::Average(a.weight, c.weight));  // !!! rewrite with fma?
    // if bz is 0, weight is 0, control point has no effect: any value will do
    float bzNonZero = !b.weight ? 1 : b.weight;
    if (!b.weight)
        b.weight = 1;
    PointWeight result(b.pt / bzNonZero, b.weight / sqrtf(a.weight * c.weight));
    if (ptT1.pt.isFinite() && ptT2.pt.isFinite()  OP_DEBUGGER_CODE(&& !debugSubDivide))
        result.pt.pin(ptT1.pt, ptT2.pt);
    return result;
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
    OpVector threshold = OpMath::Threshold(start, end);
    if ((OpMath::NearlyZeroT(t) && start.isNearly(control.pt, threshold))
            || (OpMath::NearlyOneT(t) && end.isNearly(control.pt, threshold)))
        return end - start;
    return { ConicTangent(start, control, end, XyChoice::inX, t), 
            ConicTangent(start, control, end, XyChoice::inY, t) };
}

inline std::vector<float> AddExtrema(OpPoint start, OpPoint end, PointWeight control,
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
struct DebugConic {
    CurveType curveType;
    size_t curveSize;
    OpPoint curveData[4];
    float extrema[2];
};
#endif

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the Conicratic Bezier
inline size_t AddConics(Contour* contour, AddCurve curve) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[2];
    float weight = curve.points[3].x;  // !!! a bit of a hack
    PointWeight control(curve.points[1], weight);
    OpPoint swizzled[4] { start, end, control.pt, { weight, 0 } };
    Curve conic { curve.context, (CurveData*) swizzled, curve.size, curve.type };
#if OP_TEST
    // save original curve and extrema t values as debugging data for visualization
    auto setDebugConic = [contour, swizzled, &conic](std::vector<float>* tValues) {
        OP_ASSERT(sizeof(swizzled) == sizeof(DebugConic::curveData));
        DebugConic debugConic { conic.type, sizeof(swizzled) };
        memcpy(debugConic.curveData, swizzled, sizeof(swizzled));
        size_t extremaCount = tValues ? tValues->size() : 0;
        for (size_t index = 0; index < ARRAY_COUNT(DebugConic::extrema); ++index) {
            debugConic.extrema[index] = index < extremaCount ? (*tValues)[index] : OpNaN;
        }
        SetDebugCurveData(contour, { (DebugCurve*) &debugConic, sizeof(debugConic) });
    };
#endif
    auto [left, right] = std::minmax(start.x, end.x);
    bool monotonicInX = left <= control.pt.x && control.pt.x <= right;
    auto [top, bottom] = std::minmax(start.y, end.y);
    bool monotonicInY = top <= control.pt.y && control.pt.y <= bottom;
    if (monotonicInX && monotonicInY) {
#if OP_TEST
        setDebugConic(nullptr);
#endif
        if (start == end)
            return 0;
        Add(contour, conic );
        return 1;
    }
    // control point is not inside bounds formed by end points; split Conic into parts
	std::vector<float> tValues = AddExtrema(start, end, control, monotonicInX, monotonicInY);
#if OP_TEST
    OP_ASSERT(sizeof(tValues[0]) == sizeof(DebugConic::extrema[0]));
    OP_ASSERT(tValues.size() <= ARRAY_COUNT(DebugConic::extrema));
    setDebugConic(&tValues);
#endif
	tValues.push_back(0);
	tValues.push_back(1);
    std::sort(tValues.begin(), tValues.end());
    std::vector<OpPtT> ptTs(tValues.size());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (unsigned index = 1; index < tValues.size() - 1; ++index) {
        ptTs[index] = { ConicPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
    size_t curvesAdded = tValues.size() - 1;
    for (unsigned index = 0; index < curvesAdded; ++index) {
        if (ptTs[index].pt == ptTs[index + 1].pt)
            continue;
        struct ConicData {
            OpPoint endPts[2];
            PointWeight control;
        } curveData { { ptTs[index].pt, ptTs[index + 1].pt },
                ConicControl(start, control, end, ptTs[index], ptTs[index + 1]) };
            Add(contour, { curve.context, curveData.endPts, curve.size, curve.type } );
    }
    return curvesAdded;
}

// callback functions
inline bool conicIsFinite(Curve c) {
    PointWeight control(c);
    return control.pt.isFinite();
}

inline int conicHullPtCount() {
    return 1;
}

inline bool conicIsLine(Curve c, float threshold) {
    PointWeight control(c);
    LinePts linePts { c.data->start, c.data->end };
    return linePts.ptOnLine(control.pt, threshold);
}

inline OpRoots conicAxisT(Curve curve, Axis axis, float intercept  
		OP_DEBUG_PARAMS(const OpRoots& )) {
    PointWeight control(curve);
    float a = curve.data->end.choice(axis);
    float b = control.pt.choice(axis) * control.weight - intercept * control.weight + intercept;
    float c = curve.data->start.choice(axis);
    a += c - 2 * b;    // A = a - 2*b + c
    b -= c;            // B = -(b - c)
    OpRoots result = OpMath::QuadRootsDouble(a, 2 * b, c - intercept);  // ? double req'd: testConics3759897
    result = result.keepValidT();
    return result;
    
}

inline OpRoots conicRotatedT(Curve curve, Axis axis, float intercept
		OP_DEBUG_PARAMS(const OpRoots& debugAdded)) {
	OpPoint start = curve.data->start;
	OpPoint end = curve.data->end;
	PointWeight control(curve);
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
        ptTs[index + 1] = { ConicPointAtT(start, control, end, tValues[index]), tValues[index] }; 
    } 
	OpRoots result;
	unsigned lastIndex = (unsigned) (ptTs.size() - 1);
    for (unsigned index = 0; index < lastIndex; ++index) {
        struct ConicData {
            OpPoint endPts[2];
            PointWeight control;
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
        conicData.control = ConicControl(start, control, end, ptTs[index], ptTs[index + 1]);
		Curve part { curve.context, (CurveData*) &conicData, curve.size, curve.type };
		OpRoots partRoot = conicAxisT(part, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
		for (float root : partRoot.roots)
			result.add(startT + root * (endT - startT));
	}
	return result;
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

inline void conicPin(Curve c, OpPoint newStart, OpPoint newEnd) {
    PointWeight control(c);
    control.pt.pin(newStart, newEnd);
    control.copyTo(c);
 }

inline OpVector conicTangent(Curve c, float t) {
    PointWeight control(c);
    return ConicTangent(c.data->start, control, c.data->end, t);
}

#if 0
inline OpVector conicNormal(Curve c, float t) {
    OpVector tan = conicTangent(c, t);
    return { -tan.dy, tan.dx };
}
#endif

inline void conicRotate(Curve c, OpPoint origin, OpVector scale, Curve result) {
    PointWeight control(c);
    OpVector v = control.pt - origin;
#if 1
    PointWeight rotated({ scale.cross(v), scale.dot(v) }, control.weight);
#else
    PointWeight rotated({ v.dy * s.dx - v.dx * s.dy, v.dy * s.dy + v.dx * s.dx }, control.weight);
#endif
    rotated.copyTo(result);
}

inline void conicSetBounds(Curve c, OpRect& bounds) {
    PointWeight control(c);
    bounds.add(control.pt);
}

inline void conicSubDivide(Curve curve, float t1, float t2, OpVector threshold, Curve* result) {
	OpPtT ptT1 { result->data->start, t1 };
	OpPtT ptT2 { result->data->end, t2 };
    PointWeight control(curve);
    PointWeight subPtW = ConicControl(curve.data->start, control, curve.data->end, ptT1, ptT2);
    subPtW.copyTo(*result);
    if (conicIsLine(*result, threshold.length()))
        result->type = degenerateLine;
}

inline OpPoint conicHull(Curve c, int index) {
    if (1 == index)
        return PointWeight(c).pt;
    OP_ASSERT(0); // should never be called
    return OpPoint();
}

#if OP_DEBUG_DUMP
inline void debugConicSubDivide(Curve curve, float t1, float t2, Curve* result) {
	OpPtT ptT1 { result->data->start, t1 };
	OpPtT ptT2 { result->data->end, t2 };
    PointWeight control(curve);
    PointWeight subPtW = ConicControl(curve.data->start, control, curve.data->end, ptT1, ptT2, true);
    subPtW.copyTo(*result);
}

inline std::string conicDebugDumpExtra(Curve c, DebugLevel l, DebugBase b) {
    PointWeight control(c);
    return debugValue(l, b, " weight", control.weight);
}

#define DUMP_CONIC_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(conicDebugDumpExtra), \
    OP_TAGGED_FUNCTION(debugConicSubDivide), \

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
	SetDebugCurveCallbacks(context, nativeCurveType, { debugConicScale
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpName, conicDebugDumpExtra, debugConicSubDivide)
//            OP_DEBUG_RASTER_PARAMS(debugRasterAdd)
            });
#endif
}

}

#endif
