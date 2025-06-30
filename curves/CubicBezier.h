// (c) 2024, Cary Clark cclark2@gmail.com

#include "PathOps.h"

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
#if 0 &&  OP_DEBUG_VALIDATE  // compare with pure double to see if it gets any more accurate...
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
    DPoint dE = dInterp((ptT1.t * 2. + ptT2.t) / 3.);
    DPoint dF = dInterp((ptT1.t + ptT2.t * 2.) / 3.);
    DPoint dM(dE.x * 27. - ptT1.pt.x * 8. - ptT2.pt.x, 
	          dE.y * 27. - ptT1.pt.y * 8. - ptT2.pt.y);
    DPoint dN(dF.x * 27. - ptT1.pt.x - ptT2.pt.x * 8., 
	          dF.y * 27. - ptT1.pt.y - ptT2.pt.y * 8.);
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
    OpPoint e = interp((ptT1.t * 2 + ptT2.t) / 3);
    OpPoint f = interp((ptT1.t + ptT2.t * 2) / 3);
//    OpPoint d = interp(ptT2.t);
    OpVector m = e * 27 - ptT1.pt * 8 - ptT2.pt;
    OpVector n = f * 27 - ptT1.pt - ptT2.pt * 8;
    CubicControls results;
	OpPoint ctrl1((m * 2 - n) / 18);
	OpPoint ctrl2((n * 2 - m) / 18);
#if 0 && OP_DEBUG_VALIDATE // compare float and double to measure epsilon of float error
	OpPoint err1((float) fabs(dCtrl1.x - ctrl1.x), (float) fabs(dCtrl1.y - ctrl1.y));
	OpPoint err2((float) fabs(dCtrl2.x - ctrl2.x), (float) fabs(dCtrl2.y - ctrl2.y));
	if (fabsf(err1.x) > OpEpsilon * 16 || fabsf(err1.y) > OpEpsilon * 16)
		OpDebugOut("err1: " + STR(err1.x / OpEpsilon) + ", " + STR(err1.y / OpEpsilon) + "\n");
	if (fabsf(err2.x) > OpEpsilon * 16 || fabsf(err2.y) > OpEpsilon * 16)
		OpDebugOut("err2: " + STR(err2.x / OpEpsilon) + ", " + STR(err2.y / OpEpsilon) + "\n");
#endif
	/* b = */ results.pts[0] = start == ptT1.pt && start == controls.pts[0] ? start : ctrl1;
    /* c = */ results.pts[1] = end == ptT2.pt && end == controls.pts[1] ? end : ctrl2;
    results.pts[0].pin(ptT1.pt, ptT2.pt);
    results.pts[1].pin(ptT1.pt, ptT2.pt);
    return results;
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

// if monotonic curve is rotated, there can be at most a single extrema
inline OpRoots AddExtrema(OpPoint start, OpPoint end, CubicControls& controls,
		bool single) {
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
    OpPoint A = controls.pts[0] - start;
    OpPoint B = controls.pts[1] - 2 * controls.pts[0] + start;
    OpPoint C = end + 3 * (controls.pts[0] - controls.pts[1]) - start;
    OpRoots result = OpMath::QuadRootsInteriorT(B.x * C.y - B.y * C.x, A.x * C.y - A.y * C.x,
            A.x * B.y - A.y * B.x);  // don't keep roots ~0, ~1
	return result;
}

// Curves must be subdivided so their endpoints describe the rectangle that contains them
// returns the number of curves generated from the cubic Bezier
inline void AddCubics(Contour* contour, AddCurve curve) {
    OpPoint start = curve.points[0];
    OpPoint end = curve.points[1];
    CubicControls controls { curve.points[2], curve.points[3] };
    // control point is not inside bounds formed by end points; split cubic into parts
	OpRoots tValues = AddExtrema(start, end, controls, false);
	tValues.add(0);
	tValues.add(1);
	OpRoots roots = AddInflections(start, end, controls);
	tValues.add(roots);
    tValues.sort();
    std::vector<OpPtT> ptTs(tValues.count());
    ptTs.front() = { start, 0 };
    ptTs.back() = { end, 1 };
    for (int index = 1; index < tValues.count() - 1; ++index) {
        ptTs[index] = { CubicPtAtT(start, controls, end, tValues.get(index)), tValues.get(index) }; 
    } 
    for (int index = 0; index < tValues.count() - 1; ++index) {
        OpPoint curveData[4] { ptTs[index].pt, ptTs[index + 1].pt };
        if (curveData[0] == curveData[1])
            continue;
        *(CubicControls*)&curveData[2] = CubicControlPt(start, controls, end, 
                ptTs[index], ptTs[index + 1]);
        Add(contour, { curveData, curve.size, curve.type } );
		// for debugging
		OP_DEBUG_CODE(*(CubicControls*)&curveData[2] = CubicControlPt(start, controls, end, 
                ptTs[index], ptTs[index + 1]));
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

inline bool cubicIsLine(Curve c) {
    CubicControls controls(c);
    LinePts linePts = { c.data->start, c.data->end };
    return linePts.ptOnLine(controls.pts[0]) && linePts.ptOnLine(controls.pts[1]);
}

#define OP_DEBUG_CUBIC_VERBOSE 1
#define OP_DEBUG_ROOT_CLASSIC 1
#define OP_DEBUG_ROOT_CY 0
#define OP_DEBUG_ROOT_NOUVEAU 1

#if OP_DEBUG_CUBIC_VERBOSE
	#define VERBOSE_OUT(...) OpDebugOut(__VA_ARGS__)
#else
	#define VERBOSE_OUT(...)
#endif

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
#if OP_DEBUG_ROOT_CLASSIC
	MatchEnds ends = MatchEnds::none;
    OpRoots result = OpMath::CubicRootsReal(A, B, C, D - axisIntercept, ends);
#endif
#if 0 && (OP_DEBUG_ROOT_CY || OP_DEBUG_ROOT_NOUVEAU)
	float crossRoot = OpNaN;
	if ((axisIntercept - A) * (D - axisIntercept) <= 0)  // if intercept is crossed...
		crossRoot = fabs(axisIntercept - A) < fabs(D - axisIntercept);  // ...return 0 or 1
#endif
#if OP_DEBUG_ROOT_CY
	OpRoots test2 = OpMath::CubicRootsY(a, b, c_, d - axisIntercept);  // cy's original
#endif
#if OP_DEBUG_ROOT_NOUVEAU
	OpRoots test;
	test.add(OpMath::CubicRoot(A, B, C, D - axisIntercept));  // cy's distilled
#endif
#if OP_DEBUG_ROOT_CY && OP_DEBUG_ROOT_NOUVEAU && OP_DEBUG_CUBIC_VERBOSE
	if (test2.count() != test.count()) {
		VERBOSE_OUT("distill count : " + STR(test.count()) + " disagrees with cy: " 
				+ STR(test2.count()));
		OpMath::CubicRootsY(a, b, c_, d - axisIntercept);	// !!! for tracing through errors
		OpRoots knownRoots;
		knownRoots.add(OpMath::CubicRoot(a, b, c_, d - axisIntercept));
		OP_ASSERT(0);
	} else {
//		bool disagrees = false;
		test.sort();
		test2.sort();
		for (int index = 0; index < test2.count(); ++index) {
			if (test2.roots[index] != test.roots[index]) {
				VERBOSE_OUT("distill value: " + STR(test.roots[index]) + " disagrees with cy: " 
						+ STR(test2.roots[index]) + "\n");
//				disagrees = true;
#if 1
				OpMath::CubicRootsY(a, b, c_, d - axisIntercept);	// !!! for tracing through errors
				OpRoots knownRoots2;
				knownRoots2.add(OpMath::CubicRoot(a, b, c_, d - axisIntercept));
				OP_ASSERT(0);
#endif
			}
		}
//		OP_ASSERT(!disagrees);
	}
#endif
#if OP_DEBUG_ROOT_CLASSIC
	OpRoots valid = result.keepValidTs();
	float classicError = 0;
#endif
#if OP_DEBUG_ROOT_CLASSIC && OP_DEBUG_ROOT_NOUVEAU && OP_DEBUG_CUBIC_VERBOSE
	bool matchAll = true;
	for (float f : valid.roots) {
		bool matchOne = false;
		for (float f2 : test.roots) {
			matchOne |= f == f2;
		}
		for (float f3 : debugAdded.roots) {
			matchOne |= f == f3;
		}
		if (!matchOne)
			matchAll = false;
	}
	if (!matchAll) {
		std::string s;
		s += "axisIntercept: " + STR(axisIntercept) + "\n";
		s += "classic: ";
		OP_ASSERT(valid.count() == test.count() + debugAdded.count());
		OpMath::CubicRootsY(A, B, C, D - axisIntercept);	// !!! for tracing through errors
		for (float f : valid.roots) {
			OpPoint pt = CubicPtAtT(c.data->start, controls, c.data->end, f);
			float error = fabs(pt.choice(axis) - axisIntercept);
			classicError = std::max(classicError, error);
			s += STR(f)  
#if OP_DEBUG_DUMP
					+ " " + pt.debugDump(DebugLevel::normal, DebugBase::dec) 
#endif
					+ " error:" + STR(error) + " "; 
		}
		s += "\n";
		s += "nouveau: ";
		bool tooBig = false;
		for (float f2 : test.roots) {
			OpPoint pt = CubicPtAtT(c.data->start, controls, c.data->end, f2);
			float error = fabs(pt.choice(axis) - axisIntercept);
			tooBig |= error > 8 * OpEpsilon && error > classicError * 2;
			s += STR(f2) 
#if OP_DEBUG_DUMP
					+ " " + pt.debugDump(DebugLevel::normal, DebugBase::dec)
#endif
					+ " error:" + STR(error) + " "; 
		}
		s += "\n";
		if (tooBig) {
			VERBOSE_OUT(s);
			OpRoots cyRoots = OpMath::CubicRootsY(A, B, C, D - axisIntercept);	// !!! for tracing through errors
			OpRoots knownRoots3;
			knownRoots3.add(OpMath::CubicRoot(A, B, C, D - axisIntercept));
		}
	}
#endif
#if OP_DEBUG_ROOT_CLASSIC && !OP_DEBUG_ROOT_NOUVEAU
	return result;
#else
	OP_ASSERT(OP_DEBUG_ROOT_NOUVEAU);
	return test;
#endif
}

// if curve is rotated, break at extrema before calling nouveau intersection finder
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
        *(CubicControls*)&curveData[2] = CubicControlPt(start, controls, end, 
                ptTs[index], ptTs[index + 1]);
		Curve part { (CurveData*) curveData, c.size, c.type };
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

#if 0
inline OpVector cubicNormal(Curve c, float t) {
    OpVector tan = cubicTangent(c, t);
    return { -tan.dy, tan.dx };
}
#endif

inline void cubicPinCtrl(Curve c) {
    CubicControls controls(c);
    controls.pts[0].pin(c.data->start, c.data->end);
    controls.pts[1].pin(c.data->start, c.data->end);
    controls.copyTo(c);
}

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

inline bool cubicSubDivide(Curve c, float t1, float t2, Curve result) {
	OpPtT ptT1 { result.data->start, t1 };
	OpPtT ptT2 { result.data->end, t2 };
    CubicControls controls(c);
    CubicControls subControl = CubicControlPt(c.data->start, controls, c.data->end, ptT1, ptT2);
	// if partial curve control points are not monotonic, assume cubic has devolved to line
#if 0
// !!! inflections are an indicator that the sub control points have error, but do not by themselves
//     detect that the resulting curve (error free) is linear.  Maybe the present of inflections
//     could trigger a more careful linear test..
	OpRoots hasInflection = AddInflections(result.data->start, result.data->end, subControl);
	if (!hasInflection.empty()) {
#if OP_DEBUG_VALIDATE
		subControl = CubicControlPt(c.data->start, controls, c.data->end, ptT1, ptT2);  // trace
		subControl.copyTo(result);  // check if there is really an inflection
		if (!debugDmpIsLine(result))
			dmp(result);
#endif
		return false;
	}
#endif
    subControl.copyTo(result);
	if (cubicIsLine(result))
		return false;
	return true;
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
