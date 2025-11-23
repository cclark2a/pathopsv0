// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef CubicBezier_DEFINED
#define CubicBezier_DEFINED

#include "PathOps.h"
#include "DebugOps.h"
#include "OpDebugRaster.h"

namespace PathOpsV0Lib {

struct CubicControls {
    CubicControls() {
    }

    CubicControls(Curve c) {
        OP_ASSERT(sizeof(pts) == c.size - CurveUserDataOffset());
        std::memcpy(pts, CurveUserData(c.data), sizeof(pts));
    }

    CubicControls(OpPoint pt1, OpPoint pt2) {
        pts[0] = pt1;
        pts[1] = pt2;
    }

    void copyTo(Curve c) {
        OP_ASSERT(sizeof(pts) == c.size - CurveUserDataOffset());
        std::memcpy(CurveUserData(c.data), pts, sizeof(pts));
    }

    OpPoint pts[2];
};

inline OpPoint CubicPtAtT(OpPoint start, CubicControls controls, OpPoint end, float t) {
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

inline OpPoint CubicDPtAtT(OpPoint start, CubicControls controls, OpPoint end, float t) {
    if (0 == t)
        return start;
    if (1 == t)
        return end;
    double one_t = 1 - t;
    double one_t2 = one_t * one_t;
    double a = one_t2 * one_t;
    double b = 3 * one_t2 * t;
    double t2 = t * t;
    double c = 3 * one_t * t2;
    double d = t2 * t;
    OpPoint result = 
		{ (float) (a * start.x + b * controls.pts[0].x + c * controls.pts[1].x + d * end.x),
		  (float) (a * start.y + b * controls.pts[0].y + c * controls.pts[1].y + d * end.y) };
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
    OpVector threshold = OpMath::Threshold(start, end);
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

inline CubicControls CubicControlPt(OpPoint start, CubicControls controls, OpPoint end, 
        float t1, float t2) {
	OpPoint ptT1 = CubicPtAtT(start, controls, end, t1);
	OpPoint ptT2 = CubicPtAtT(start, controls, end, t2);
#if 0 && OP_DEBUG_VALIDATE  // compare with pure double to see if it gets any more accurate...
	struct DPoint {
		DPoint(double _x, double _y) :
			x(_x),
			y(_y) {
		}

		double x;
		double y;
	};
	auto DInterp = [](DPoint d1, DPoint d2, double dT, double dS) {
		return DPoint(d1.x * dS + d2.x * dT, d1.y * dS + d2.y * dT);
	};
	auto DInterp2 = [](OpPoint d1, OpPoint d2, double dT, double dS) {
		return DPoint(d1.x * dS + d2.x * dT, d1.y * dS + d2.y * dT);
	};
    auto dInterp = [start, controls, end, DInterp, DInterp2](double dT) {
		double dS = 1. - dT;
        DPoint ab = DInterp2(start, controls.pts[0], dT, dS);
        DPoint bc = DInterp2(controls.pts[0], controls.pts[1], dT, dS);
        DPoint cd = DInterp2(controls.pts[1], end, dT, dS);
        DPoint abc = DInterp(ab, bc, dT, dS);
        DPoint bcd = DInterp(bc, cd, dT, dS);
        DPoint abcd = DInterp(abc, bcd, dT, dS);
        return abcd;
    };
    DPoint dE = dInterp((t1 * 2. + t2) / 3.);
    DPoint dF = dInterp((t1 + t2 * 2.) / 3.);
    DPoint dM(dE.x * 27. - ptT1.x * 8. - ptT2.x, 
	          dE.y * 27. - ptT1.y * 8. - ptT2.y);
    DPoint dN(dF.x * 27. - ptT1.x - ptT2.x * 8., 
	          dF.y * 27. - ptT1.y - ptT2.y * 8.);
	DPoint dCtrl1((dM.x * 2. - dN.x) / 18., (dM.y * 2. - dN.y) / 18.);
    DPoint dCtrl2((dN.x * 2. - dM.x) / 18., (dN.y * 2. - dM.y) / 18.);
#endif
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
    OpPoint e = interp((t1 * 2 + t2) / 3);
    OpPoint f = interp((t1 + t2 * 2) / 3);
//    OpPoint d = interp(ptT2.t);
    OpVector m = e * 27 - ptT1 * 8 - ptT2;
    OpVector n = f * 27 - ptT1 - ptT2 * 8;
	OpVector ctrl1((m * 2 - n) / 18);
	OpVector ctrl2((n * 2 - m) / 18);
    CubicControls results { OpPoint(ctrl1.dx, ctrl1.dy), OpPoint(ctrl2.dx, ctrl2.dy) };
#if 0 && OP_DEBUG_VALIDATE // compare float and double to measure epsilon of float error
	OpPoint err1((float) fabs(dCtrl1.x - ctrl1.x), (float) fabs(dCtrl1.y - ctrl1.y));
	OpPoint err2((float) fabs(dCtrl2.x - ctrl2.x), (float) fabs(dCtrl2.y - ctrl2.y));
	if (fabsf(err1.x) > OpEpsilon * 16 || fabsf(err1.y) > OpEpsilon * 16)
		OpDebugOut("err1: " + STR(err1.x / OpEpsilon) + ", " + STR(err1.y / OpEpsilon) + "\n");
	if (fabsf(err2.x) > OpEpsilon * 16 || fabsf(err2.y) > OpEpsilon * 16)
		OpDebugOut("err2: " + STR(err2.x / OpEpsilon) + ", " + STR(err2.y / OpEpsilon) + "\n");
#endif

#if 0 && OP_DEBUG_VALIDATE
	// compare raw control points with mod points by evaluating pt values from t to see the
	// tweak is distorting the cubic
	constexpr int samples = 16;
	static float gMaxError = 0;
	for (int index = 0; index < samples; ++index) {
		float sample = (t1 * (samples - index) + t2 * index) / samples;
		float local = (float) index / samples;
		OP_ASSERT(t1 <= sample && sample <= t2);
		OP_ASSERT(0 <= local && local <= 1);
		OP_ASSERT(t1 * (1 - local) + t2 * local == sample);
		OpPoint truth = CubicPtAtT(start, controls, end, sample);
		OpPoint raw = CubicPtAtT(ptT1, results, ptT2, local);
		float rawError = (truth - raw).length();
		gMaxError = std::max(gMaxError, rawError);
		if (rawError > OpEpsilon * 8)
			OpDebugOut("raw error too large: " + STR(rawError / OpEpsilon) + "\n");
		OP_ASSERT(rawError <= OpEpsilon * 1024);
	}
#endif
    if (start == ptT1 && start == controls.pts[0])
	    results.pts[0] = start;
    if (end == ptT2 && end == controls.pts[1])
        results.pts[1] = end;
    results.pts[0].pin(ptT1, ptT2);
    results.pts[1].pin(ptT1, ptT2);
    return results;
}

// if monotonic curve is rotated, there can be at most a single extrema
inline OpRoots AddExtrema(OpPoint start, OpPoint end, CubicControls& controls, bool single) {
    OpRoots tValues;
    auto addExtrema = [&tValues, single](float a, float b, float c, float d) {
        float A = d - a + 3 * (b - c);
        float B = 2 * (a - b - b + c);
        float C = b - a;
        OpRoots roots = OpMath::QuadRootsInteriorT(A, B, C);  // don't keep roots ~0, ~1
		if (single && 2 == roots.count()) {
			tValues.add(roots.roots[0]);
			if (fabsf(roots.roots[1] - .5f) < fabsf(roots.roots[0] - .5f))
				tValues.roots[0] = roots.roots[1];
		} else
			for (int index = 0; index < roots.count(); ++index)
				tValues.add(roots.roots[index]);
    };
    addExtrema(start.x, controls.pts[0].x, controls.pts[1].x, end.x);
    addExtrema(start.y, controls.pts[0].y, controls.pts[1].y, end.y);
	return tValues;
}

inline OpRoots AddInflections(OpPoint start, OpPoint end, CubicControls& controls) {
    OpVector A = controls.pts[0] - start;
    OpVector B = controls.pts[1] - 2 * controls.pts[0] + OpVector(start.x, start.y);
    OpVector C = end + 3 * (controls.pts[0] - controls.pts[1]) - start;
    OpRoots result = OpMath::QuadRootsInteriorT(B.dx * C.dy - B.dy * C.dx, A.dx * C.dy - A.dy * C.dx,
            A.dx * B.dy - A.dy * B.dx);  // don't keep roots ~0, ~1
	return result;
}

enum class CubicSubDivide {
    noAngleChecks,
    checkAngles
    OP_DEBUG_DUMP_PARAMS(debuggerSubDivide)
};

inline void cubicCommonSubDivide(Curve c, float t1, float t2, float threshold, Curve* result,
        CubicSubDivide check) {
    CubicControls controls(c);
	OpPoint start = c.data->start;
	OpPoint end = c.data->end;
	OpVector t1Tan = CubicTangent(start, controls, end, t1);
	OpVector t2Tan = CubicTangent(start, controls, end, t2);
	float tanAngle = t1Tan.cross(t2Tan);
    float t1TanLen = t1Tan.length();
    float t2TanLen = t2Tan.length();
	if (fabsf(tanAngle) < threshold * t1TanLen && fabsf(tanAngle) < threshold * t2TanLen) {
		result->type = degenerateLine;  // mark as linear
		return;
	}
    CubicControls subControls = CubicControlPt(start, controls, end, t1, t2);
    subControls.copyTo(*result);
#if OP_DEBUGGER
    if (CubicSubDivide::debuggerSubDivide == check)
        return;
#endif
#if 1  // experiment: restrict sub controls to cross product of original controls w/ end tangent
	// cross product of control lines should have same sign
    auto makeLines = [](Curve& curve, CubicControls& controls) {
        return std::array<OpVector, 3> { 
                controls.pts[0] - curve.data->start, 
	            controls.pts[1] - controls.pts[0],
	            curve.data->end - controls.pts[1] };
    };
	std::array<OpVector, 3> ctrlLines = makeLines(c, controls);
	std::array<OpVector, 3> subLines = makeLines(*result, subControls);
    auto makeCross = [](OpVector line1, OpVector line2) {
        float result = line1.cross(line2);
        return fabs(result) <= OpEpsilon ? 0 : result;
    };
    auto makeCrosses = [makeCross](std::array<OpVector, 3>& lines) {
        std::array<float, 2> result{ makeCross(lines[0], lines[1]), makeCross(lines[1], lines[2]) };
        if (result[0] * result[1] >= 0)
            return result;
        int smaller = fabs(result[0]) > fabs(result[1]);
        float scale = fabs(result[!smaller] / result[smaller]);
        if (fabs(result[smaller]) <= OpEpsilon * std::min(scale, 256.f))
            result[smaller] = 0;
        return result;
    };
    std::array<float, 2> ctrlCrosses = makeCrosses(ctrlLines);
    int smallerCrossIndex = fabs(ctrlCrosses[0]) > fabs(ctrlCrosses[1]);
    float crossAngle = smallerCrossIndex ? ctrlCrosses[0] : ctrlCrosses[1]; 
    std::array<float, 2> subCrosses = makeCrosses(subLines);
    if (subCrosses[0] * subCrosses[1] < 0) {
    #if OP_DEBUG && !OP_DEBUG_FAST_TEST && 0
        float crossProduct = ctrlCrosses[0] * ctrlCrosses[1];
    #endif
        int smallerSubIndex = fabs(subCrosses[0]) > fabs(subCrosses[1]);
        float scale = ctrlCrosses[!smallerCrossIndex] / subCrosses[!smallerSubIndex];
        if (subCrosses[smallerSubIndex] * scale < ctrlCrosses[smallerCrossIndex] * 4)
            subCrosses[smallerSubIndex] = 0;  // don't mark it as a line
        else if (fabsf(subCrosses[smallerSubIndex] / subCrosses[!smallerSubIndex]) < 0.002)
            subCrosses[smallerSubIndex] = 0;
    #if OP_DEBUG && !OP_DEBUG_FAST_TEST && 0
        else  // loop105792 triggers assert but works anyway...
            OP_ASSERT(crossProduct >= 0 || fabs(crossProduct) < OpEpsilon * 256);
    #endif
       
    }
    // check if original data is inflection-free
    // sub divide should bend the same way as original
    if (!OP_DEBUGGER) {
        if (0 == subCrosses[0] && 0 == subCrosses[1]) {
            result->type = degenerateLine;
            return;
        }
        if (subCrosses[0] * subCrosses[1] < 0) {
            result->type = degenerateLine;
            return;
        }
        if (CubicSubDivide::checkAngles == check) {
            if (subCrosses[0] * crossAngle < 0) {
                result->type = degenerateLine;
                return;
            }
            if (subCrosses[1] * crossAngle < 0) {
                result->type = degenerateLine;
                return;
            }
        }
    }
#endif
    subControls.pts[0].pin(result->data->start, result->data->end);
    subControls.pts[1].pin(result->data->start, result->data->end);
    subControls.copyTo(*result);
}

#if OP_DEBUG_IMAGE
struct DebugCubic {
    CurveType curveType;
    size_t curveSize;
    OpPoint curveData[4];
    float extrema[3];
};
#endif

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the cubic Bezier
inline void AddCubics(Contour* contour, AddCurve curve) {
    // swizzle input to match v0's start/end/ctrl1/ctrl2 layout
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[3];
    CubicControls controls { curve.points[1], curve.points[2] };
	OpPoint swizzled[4] { start, end, controls.pts[0], controls.pts[1] };  
    Curve cubic { curve.context, (CurveData*) swizzled, curve.size, curve.type };
    // control point is not inside bounds formed by end points; split cubic into parts
	OpRoots tValues = AddExtrema(start, end, controls, false);
	tValues.add(AddInflections(start, end, controls));
#if OP_DEBUG_IMAGE
    // save original curve and extrema t values as debugging data for visualization
    OP_ASSERT(sizeof(swizzled) == sizeof(DebugCubic::curveData));
    OP_ASSERT(sizeof(tValues.roots[0]) == sizeof(DebugCubic::extrema[0]));
    OP_ASSERT(tValues.count() <= (int) ARRAY_COUNT(DebugCubic::extrema));
    DebugCubic debugCubic { curve.type, sizeof(swizzled) };
    memcpy(debugCubic.curveData, swizzled, sizeof(swizzled));
    for (size_t index = 0; index < ARRAY_COUNT(DebugCubic::extrema); ++index) {
        debugCubic.extrema[index] = index < (size_t) tValues.count() ? tValues.get(index) : OpNaN;
    }
    SetDebugCurveData(contour, { (DebugCurve*) &debugCubic, sizeof(debugCubic) });
#endif
    if (tValues.empty()) {
        if (start != end)
            Add(contour, cubic);
        return;
    }
	tValues.add(0);
	tValues.add(1);
    if (std::any_of(tValues.roots.begin(), tValues.roots.end(), [](float t) {
            return OpMath::IsNaN(t); })) {
        PathOpsV0Lib::SetError(curve.context, PathOpsV0Lib::ContextError::root);
        return;
    }
    tValues.sort();
    tValues.smooth();  // disallow intervals <= epsilon
    std::vector<OpPtT> ptTs(tValues.count());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (int index = 1; index < tValues.count() - 1; ++index) {
        ptTs[index] = { CubicPtAtT(start, controls, end, tValues.get(index)), tValues.get(index) }; 
    } 
    float threshold = OpMath::Threshold(start, end).length();
    for (int index = 0; index < tValues.count() - 1; ++index) {
        OpPoint result[4] { ptTs[index].pt, ptTs[index + 1].pt };
        if (result[0] == result[1])
            continue;
        Curve subDivide { curve.context, (CurveData*) result, cubic.size, cubic.type };
        cubicCommonSubDivide(cubic, tValues.roots[index], tValues.roots[index + 1], 
                threshold, &subDivide, CubicSubDivide::noAngleChecks);
        Add(contour, subDivide);
		// for debugging
		OP_DEBUG_CODE(cubicCommonSubDivide(cubic, tValues.roots[index], tValues.roots[index + 1], 
                threshold, &subDivide, CubicSubDivide::noAngleChecks));
    }
}

// callback functions
inline int cubicHullPtCount() {
    return 2;
}

inline bool cubicIsFinite(Curve c) {
    CubicControls controls(c);
    return controls.pts[0].isFinite() && controls.pts[1].isFinite();
}

inline bool cubicIsLine(Curve c, float threshold) {
    CubicControls controls(c);
    LinePts linePts = { c.data->start, c.data->end };
    return linePts.ptOnLine(controls.pts[0], threshold) 
            && linePts.ptOnLine(controls.pts[1], threshold);
}

// cubic must be monotonic and non-linear (e.g., an arc, not a line)
// axis-aligned intercept can at most cross the cubic at one point
// if match ends is set, return appropriate root immediately
inline OpRoots cubicAxisT(Curve c, Axis axis, float axisIntercept  
		OP_DEBUG_PARAMS(const OpRoots& debugAdded)) {
    CubicControls controls(c);
    OpCubicFloatType A = c.data->end.choice(axis);   // d
    OpCubicFloatType B = controls.pts[1].choice(axis) * 3;  // 3*c
    OpCubicFloatType C = controls.pts[0].choice(axis) * 3;  // 3*b
    OpCubicFloatType D = c.data->start.choice(axis);   // a
    A -= D - C + B;     // A =   -a + 3*b - 3*c + d
    B += 3 * D - 2 * C; // B =  3*a - 6*b + 3*c
    C -= 3 * D;         // C = -3*a + 3*b
	OpRoots test;
	test.add(OpMath::CubicRoot(A, B, C, D - axisIntercept));
	return test;
}

// if curve is rotated, break at extrema before calling intersection finder
inline OpRoots cubicRotatedT(Curve c, Axis axis, float intercept  
		OP_DEBUG_PARAMS(const OpRoots& debugAdded)) {
	OpPoint start = c.data->start;
	OpPoint end = c.data->end;
    CubicControls controls(c);
	OpRoots tValues = AddExtrema(start, end, controls, true);
	if (tValues.empty())
		return cubicAxisT(c, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
    tValues.sort();
    std::vector<OpPtT> ptTs(tValues.count() + 2);
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (int idx = 0; idx < tValues.count(); ++idx) {
        ptTs[idx + 1] = { CubicPtAtT(start, controls, end, tValues.get(idx)), tValues.get(idx) }; 
    } 
	OpRoots result;
	unsigned lastIndex = ptTs.size() - 1;
    for (unsigned index = 0; index < lastIndex; ++index) {
        OpPoint curveData[4] { ptTs[index].pt, ptTs[index + 1].pt };
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
        *(CubicControls*)&curveData[2] = CubicControlPt(start, controls, end, startT, endT);
		Curve part { c.context, (CurveData*) curveData, c.size, c.type };
		OpRoots partRoot = cubicAxisT(part, axis, intercept  OP_DEBUG_PARAMS(debugAdded));
		for (float root : partRoot.roots)
			result.add(startT + root * (endT - startT));
	}
	return result;
}

inline OpPoint cubicPtAtT(Curve c, float t) {
    CubicControls controls(c);
    return CubicPtAtT(c.data->start, controls, c.data->end, t);
}

inline OpPoint cubicDPtAtT(Curve c, float t) {
    CubicControls controls(c);
    return CubicDPtAtT(c.data->start, controls, c.data->end, t);
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

inline void cubicPin(Curve c, OpPoint oldStart, OpPoint oldEnd) {
//	OP_ASSERT(c.data->start != oldStart || c.data->end != oldEnd);
    CubicControls controls(c);
	controls.pts[0] += c.data->start - oldStart;
	controls.pts[1] += c.data->end - oldEnd;
    controls.pts[0].pin(c.data->start, c.data->end);
    controls.pts[1].pin(c.data->start, c.data->end);
    controls.copyTo(c);
}

inline OpVector cubicTangent(Curve c, float t) {
    CubicControls controls(c);
    return CubicTangent(c.data->start, controls, c.data->end, t);
}

#if 0
inline OpVector cubicNormal(Curve c, float t) {
    OpVector tan = cubicTangent(c, t);
    return { -tan.dy, tan.dx };
}
#endif

// end points may be adjusted to align with other curves
// move control points to ensure cubic is monotonic
// original cubic: oldStart c.ctrl1 c.ctrl2 oldEnd
//      new cubic:  c.start   calc1   calc2  c.end
// old tangent: c.ctrl1 - oldStart

inline void cubicRotate(Curve c, OpPoint origin, OpVector scale, Curve result) {
    CubicControls controls(c);
    for (int index = 0; index < 2; ++index) {
        OpVector v = controls.pts[index] - origin;
	#if 1
        controls.pts[index] = { scale.cross(v), scale.dot(v) };
	#else
        controls.pts[index] = { v.dy * s.dx - v.dx * s.dy, v.dy * s.dy + v.dx * s.dx };
	#endif
    }
    controls.copyTo(result);
}

inline void cubicSetBounds(Curve c, OpRect& bounds) {
    CubicControls controls(c);
    bounds.add(controls.pts[0]);
    bounds.add(controls.pts[1]);
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

inline void cubicSubDivide(Curve c, float t1, float t2, float threshold, Curve* result) {
    cubicCommonSubDivide(c, t1, t2, threshold, result, CubicSubDivide::checkAngles);
}

#if OP_DEBUG_DUMP
inline void debugCubicSubDivide(Curve c, float t1, float t2, float threshold, Curve* result) {
    cubicCommonSubDivide(c, t1, t2, threshold, result, CubicSubDivide::debuggerSubDivide);
}

inline std::string cubicDebugDumpName() { 
    return "cubic"; 
}

#define CUBIC_TAGGED_FUNCTIONS \
    OP_TAGGED_FUNCTION(cubicAxisT), \
    OP_TAGGED_FUNCTION(cubicRotatedT), \
    OP_TAGGED_FUNCTION(cubicHull), \
    OP_TAGGED_FUNCTION(cubicIsFinite), \
    OP_TAGGED_FUNCTION(cubicIsLine), \
    OP_TAGGED_FUNCTION(cubicSetBounds), \
    OP_TAGGED_FUNCTION(cubicPin), \
    OP_TAGGED_FUNCTION(cubicTangent), \
    OP_TAGGED_FUNCTION(cubicsEqual), \
    OP_TAGGED_FUNCTION(cubicPtAtT), \
    OP_TAGGED_FUNCTION(cubicDPtAtT), \
    OP_TAGGED_FUNCTION(cubicHullPtCount), \
	OP_TAGGED_FUNCTION(cubicRotate), \
    OP_TAGGED_FUNCTION(cubicSubDivide), \
    OP_TAGGED_FUNCTION(cubicXYAtT), \
    OP_TAGGED_FUNCTION(cubicReverse), \
    OP_TAGGED_FUNCTION(cubicDebugDumpName), \
    OP_TAGGED_FUNCTION(debugCubicSubDivide), \

#endif

inline void cubicCallbacks(Context* context, int nativeCurveType) {
    SetCurveCallbacks(context, nativeCurveType, { cubicAxisT,
			cubicRotatedT, cubicHull, cubicIsFinite, cubicIsLine, cubicSetBounds, cubicPin,
			cubicTangent, cubicsEqual, cubicPtAtT, cubicDPtAtT, cubicHullPtCount, cubicRotate, 
			cubicSubDivide, cubicXYAtT, cubicReverse });
#if OP_DEBUG
	SetDebugCurveCallbacks(context, nativeCurveType, { debugCubicScale
            OP_DEBUG_DUMP_PARAMS(cubicDebugDumpName, nullptr, debugCubicSubDivide)
//            OP_DEBUG_RASTER_PARAMS(debugRasterAdd)
            });
#endif
}

}

#endif
