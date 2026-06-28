// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContext.h"
#include "OpEdge.h"
#if OP_DEBUG_DUMP
#include "OpCurveCurve.h"
#endif
#if OP_TEST_RASTER
#include "OpDebugRaster.h"
#endif

OpCurve::OpCurve(PathOpsV0Lib::Curve curve, Rotated r) 
	: c(curve)
	, rotated(r)
	, isLineSet(false)
	, isLineResult(false)
	, isSmall(false)
    , reversed(false) {
	c.data = context().allocateCurveData(c.size);
	if (0 == (int) curve.type)
		c.type = lineType();
	if (curve.data) {
		std::memcpy(c.data, curve.data, c.size);
		start = curve.data->start;
		end = curve.data->end;
		OP_DEBUG_CODE(if (Rotated::debug == r) return);
		if (Rotated::init == r)
			rotated = Rotated::no;
		else
			isLine();
	}
}

OpRoots OpCurve::axisRawHit(Axis axis, float intercept, MatchEnds matchEnds) const {
//	OP_ASSERT(debugIsLine() || MatchEnds::both != matchEnds);  // !!! testQuads3756113 triggers
	OpRoots result;
	if (MatchEnds::start == matchEnds || firstPt().choice(axis) == intercept)
		result.add(0);
	if (MatchEnds::end == matchEnds || lastPt().choice(axis) == intercept)
		result.add(1);
	if (Rotated::no == rotated) {
		 if (!result.roots.empty())
			return result;
		float s = c.data->start.choice(axis) - intercept;
		float e = c.data->end.choice(axis) - intercept;
		if (s * e > 0)  // if both are on the same half-plane delineated by intercept and axis..
			return result;  // ..no intersection 
	}
	PathOpsV0Lib::AxisT func = Rotated::no == rotated
			? context().callback(c.type).axisTFuncPtr
			: context().callback(c.type).rotateTFuncPtr;
	if (!func) {
        if (!result.empty())
            return result;
		const float* ptr = c.data->start.asPtr(axis);
		return OpRoots((intercept - ptr[0]) / (ptr[2] - ptr[0]));
	}
	OpRoots moreRoots = (*func)(c, axis, intercept  OP_DEBUG_PARAMS(result));
	for (float root : moreRoots.roots) {
		result.addEnd(root);
	}
	result.sort();
	if (Rotated::yes == rotated && 2 < result.count()) {
		OpRoots temp(result.roots.front(), result.roots.back());
		result = temp;
	}
#if OP_DEBUG
	if (Rotated::no == rotated && result.count() > 1) {
		OpDebugOut("!!! axisRawHit count > 1: " + context().debugData.testname + "\n");
		OP_DEBUG_DUMP_CODE(((OpContext&) context()).dumpFile("axisRawHit count > 1"));
	}
#endif
	OP_ASSERT((Rotated::no == rotated ? 1 : 2) >= result.count());
	return result;
}

OpRoots OpCurve::axisRayHit(Axis axis, float axisIntercept, float start, float end) const {
	OpRoots roots = axisRawHit(axis, axisIntercept, MatchEnds::none);
	roots.keepValidTs(start, end);
	return roots;
}

float OpCurve::center(Axis axis, float intercept) const {
	OpRoots roots;
	if (OpMath::Between(c.data->start.choice(axis), intercept, c.data->end.choice(axis)))
		roots = axisRayHit(axis, intercept);
	if (1 != roots.count())
		return OpNaN;   // numerics failed
	return roots.roots[0];
}

OpRect OpCurve::closeBounds() const {
	return callerBounds().outset(context().threshold); 
}

#if 0
// cut range minimum should be double the distance between ptT pt and opp pt
CutRangeT OpCurve::cutRange(const OpPtT& ptT, OpPoint oppPt, float loEnd, float hiEnd) const {
	PathOpsV0Lib::CurveConst cutFun = context().callback(c.type).cutFuncPtr;
	float tStep = cutFun ? (*cutFun)(c) : 16.f;
	float cutDt = OpEpsilon * tStep;
	OpVector threshold = context().threshold;
	float minDistanceSq = threshold.lengthSquared() * tStep;
	CutRangeT tRange;
	for (float direction : { -1.f, 1.f }) {
		OpPtT cut;
		float dir = direction;
		do {
			cut.t = std::max(loEnd, std::min(hiEnd, ptT.t + dir * cutDt));
			cut.pt = ptAtT(cut.t);
		} while (((cut.pt - ptT.pt).lengthSquared() < minDistanceSq 
				 || (cut.pt - oppPt).lengthSquared() < minDistanceSq)
				 && loEnd < cut.t && cut.t < hiEnd && (dir *= tStep));
		(-1 == direction ? tRange.lo : tRange.hi) = cut;
	}
	return tRange;
}
#endif

float OpCurve::findValidT(float start, float end, OpPoint opp) {
	if (!isLine()) {
		OpRoots hRoots;
		if (OpMath::Between(c.data->start.y, opp.y, c.data->end.y))
			hRoots = axisRayHit(Axis::horizontal, opp.y, start, end);
		OpRoots vRoots;
		if (OpMath::Between(c.data->start.x, opp.x, c.data->end.x))
			vRoots = axisRayHit(Axis::vertical, opp.x, start, end);
	#if 01 // code coverage says this is unused, but it is required for loop48977
		if (1 != hRoots.count() && 1 != vRoots.count()) {
			if (0 == start && opp.isNearly(firstPt(), context().threshold))
				return 0;
			if (1 == end && opp.isNearly(lastPt(), context().threshold))
				return 1;
			return OpNaN;
		}
		if (1 != hRoots.count()) {
			OP_ASSERT(1 == vRoots.count());  // !!! triggered by thread_loops46134
			return vRoots.roots[0];
		}
	#endif
		if (1 != vRoots.count())
			return hRoots.roots[0];
		OpPoint hPt = ptAtT(hRoots.roots[0]);
		OpPoint vPt = ptAtT(vRoots.roots[0]);
		return (hPt - opp).lengthSquared() < (vPt - opp).lengthSquared() 
				? hRoots.roots[0] : vRoots.roots[0];
	}
	// this won't work for curves with linear control points since t is not necessarily linear
	OpVector lineSize = lastPt() - firstPt();
	float result = fabsf(lineSize.dy) > fabsf(lineSize.dx) ?
		(opp.y - firstPt().y) / lineSize.dy : (opp.x - firstPt().x) / lineSize.dx;
	if (start <= result && result <= end)
		return result;
	return OpNaN;
}

float OpCurve::interceptLimit() const {
	PathOpsV0Lib::CurveConst limFuncPtr = context().callback(c.type).interceptFuncPtr;
	if (!limFuncPtr)
		return 1.f / 256.f;
	return (*limFuncPtr)(c);
}

bool OpCurve::isFinite() const {
	if (!firstPt().isFinite())
		return false;
	if (!lastPt().isFinite())
		return false;
	PathOpsV0Lib::CurveIsFinite funcPtr = context().callback(c.type).curveIsFiniteFuncPtr;
	return funcPtr ? (*funcPtr)(c) : true;
}

OpRoots OpCurve::lineIntersection(OpCurve& curve) {
    OP_ASSERT(debugIsLine());
	MatchReverse matchRev = curve.matchExact(*this);
	// if line and curve share end point, pass hint that root finder can call
	// reduced form that assumes one root is zero or one.
	OpRoots roots;
	if (MatchEnds::both != matchRev.match) {
		LinePts edgePts { c.data->start, c.data->end };
		OP_ASSERT(edgePts.pts[0] != edgePts.pts[1]);
		roots = curve.rayIntersect(edgePts, matchRev.match);
	} else {
		roots.add(0); 
		roots.add(1);
	}
    return roots;
}

OpRoots OpCurve::lineIntersect(const LinePts& line) const {
	MatchEnds matchEnds = MatchEnds::none;
	if (firstPt() == line.pts[0] || firstPt() == line.pts[1])
		matchEnds |= MatchEnds::start;
	if (lastPt() == line.pts[0] || lastPt() == line.pts[1])
		matchEnds |= MatchEnds::end;
	OpRoots result = rawIntersect(line, matchEnds);
	OpRoots valid = result.keepValidTs();
	return valid;
// !!! complexity below may be necessary if curve and line are coincident...
//     likely unnecessary with modern cubic root implementation
#if 0
	if (RootFail::rawIntersectFailed == result.raw.fail)
		return result;
	result.valid = result.raw;
	result.valid.keepValidTs(0, 1);
	if (!result.valid.count())
		return result;
	OpVector lineV = line.pts[1] - line.pts[0];
	XyChoice xy = fabsf(lineV.dx) >= fabsf(lineV.dy) ? XyChoice::inX : XyChoice::inY;
	for (int index = 0; index < result.valid.count(); ++index) {
		OpPoint hit = ptAtT(result.valid.roots[index]);
		// thread_cubics23476 edges 55 & 52 trigger this need for betweenish
		if (OpMath::InUnsorted(line.pts[0].choice(xy), hit.choice(xy), line.pts[1].choice(xy),
				context().threshold.choice(xy))) {
			// curve/curve may need more exact results; try pinning valid hit to line bounds
			if (!lineV.dx || !lineV.dy) {
				// !!! don't like using pairs (may be less performant than returning struct)
				//     but that's what std::minmax returns (which may be very performant!)
				std::pair<float, float> minmaxX = std::minmax(line.pts[0].x, line.pts[1].x);
				std::pair<float, float> minmaxY = std::minmax(line.pts[0].y, line.pts[1].y);
				OpPoint pinned(std::min(std::max(minmaxX.first, hit.x), minmaxX.second),
							   std::min(std::max(minmaxY.first, hit.y), minmaxY.second));
				result.add(pinned, result.valid.roots[index]);
			} else
				result.add(hit, result.valid.roots[index]);
		}
	}
	return result;
#endif
}

// given a t value of the intersection of this curve and a line, find the intersecting line point
// returns true if computed point/t pair is not outside the line and is not near the end of the line
OpPtT OpCurve::lineCurve(OpCurve& line, float inputT, float* lineTPtr, MatchEnds matchEnds,
		float margin) {
	OP_ASSERT(line.isLine());
	OpPoint calcPt = ptAtT(inputT);
	if (MatchEnds::none != matchEnds && calcPt.isNearly(line.firstPt(), context().threshold))
		return OpPtT(SetToNaN::dummy);
	if (MatchEnds::none != matchEnds && calcPt.isNearly(line.lastPt(), context().threshold))
		return OpPtT(SetToNaN::dummy);
#if 1  // !!! experiment: intersect curve with line
	LinePts linePts { line.firstPt(), line.lastPt() };
	float lineT = line.findValidT(-margin, 1 + margin, calcPt);
	if (OpMath::IsNaN(lineT))
		return OpPtT(SetToNaN::dummy);
	OpPoint linePt = line.ptAtT(lineT);
	if (!linePts.ptOnLine(calcPt)) { // if calc point is already on line, there's nothing more to do
		OpVector calcTan = tangent(inputT);
		LinePts tanPts { calcPt, { calcPt.x + calcTan.dx, calcPt.y + calcTan.dy } };
		OpRoots tanSect = line.rawIntersect(tanPts, MatchEnds::none);
		if (1 == tanSect.count()) {
			float tanLineT = tanSect.roots[0];
			OpPoint tanLinePt = line.ptAtT(tanLineT);
			OpVector calcLine = calcPt - linePt;
			OpVector tanLine = tanLinePt - linePt;
			if (tanLine.lengthSquared() < calcLine.lengthSquared()) {
				lineT = tanLineT;
				linePt = tanLinePt;
			}
		}
#if 0
		// !!! next: adjust t value by assuming that for very small values of delta t, delta x/y 
		//           is linear
		// While this works, it doesn't make things better enough to keep it enabled
		OpVector delta = linePt - calcPt;
		OpVector deltaT = delta / calcTan;
		float newT = inputT + deltaT.length();
		OpPoint newPt = ptAtT(newT);
		inputT = newT;
#endif
	}
#else
	float lineT = line.findValidT(0, 1, calcPt);
	if (OpMath::IsNaN(lineT))
		return false;
	if (OpMath::NearlyEndT(lineT))
		return false;
	OpPoint linePt = line.ptAtT(lineT);  // use line instead of curve to keep points on line
#endif
	if (lineTPtr)
		*lineTPtr = lineT;
	OpPtT alignedPtT { linePt, inputT };
	return alignedPtT;
}

PathOpsV0Lib::CurveType OpCurve::lineType() const {
	PathOpsV0Lib::SetLineType funcPtr = context().contextCallbacks.setLineTypeFuncPtr;
    return funcPtr ? (*funcPtr)(c) : 1;
}

#if 0
float OpCurve::matchCommon(float start, float end, OpPoint pt, OpVector ptTan, OpVector slop, 
		OpPoint* matchPt) const {
	if (!nearBounds(pt))
		return OpNaN;
	if (pt == c.data->start && 0 == start)
		return 0;
	if (pt == c.data->end && 1 == end)
		return 1;
	float xRoot = tAtXY(start, end, XyChoice::inX, pt.x);
	float yRoot = tAtXY(start, end, XyChoice::inY, pt.y);
	if (OpMath::EqualT(xRoot, yRoot))
		return xRoot;
	OpPoint xPt = ptAtT(xRoot);
	OpPoint yPt = ptAtT(yRoot);
	float xDistSq = (pt - xPt).lengthSquared();
	float yDistSq = (pt - yPt).lengthSquared();
	// example: testQuads9421393 needs small curve factor for segs (3, 7 to detect intersection)
//			* context().callback(c.type).matchSlopFuncPtr();
	if (!(xDistSq > yDistSq)) {  // reverse test in case y dist is nan
		if (pt.isNearly(xPt, slop)) {
			if (matchPt)
				*matchPt = xPt;
			return xRoot;
		}
	}
	if (matchPt)
		*matchPt = yPt;
	return pt.isNearly(yPt, slop) ? yRoot : OpNaN;
}
#endif

float OpCurve::matchVector(float start, float end, OpPoint pt, OpVector v) const {
	if (!nearBounds(pt))
		return OpNaN;
	if (pt == c.data->start && 0 == start)
		return 0;
	if (pt == c.data->end && 1 == end)
		return 1;
	LinePts linePts { pt, pt + v };
	OpRoots roots = lineIntersect(linePts);
	for (float root : roots.roots) {
		if (start >= root || root >= end)
			continue;
		OpVector threshold = context().threshold;
		OpPoint foundPt = ptAtT(root);
		if (!foundPt.isNearly(pt, threshold))
			continue;
		return root;
	}
	return OpNaN;
}

float OpCurve::matchVector(OpPoint pt, OpVector v) const {
	return matchVector(0, 1, pt, v);
}

#if 0
float OpCurve::matchClosest(OpPoint pt, OpVector ptTan) const {
	return matchCommon(0, 1, pt, ptTan, {OpInfinity, OpInfinity}, nullptr);
}
#endif

MatchReverse OpCurve::matchEnds(const LinePts& opp) const {
	MatchReverse result { MatchEnds::none, false };
	OP_ASSERT(firstPt() != lastPt());
	OP_ASSERT(opp.pts[0] != opp.pts[1]);
	if (firstPt() == opp.pts[0])
		result = { MatchEnds::start, false };
	else if (firstPt() == opp.pts[1])
		result = { MatchEnds::start, true };
	if (lastPt() == opp.pts[1])
		result = { result.match | MatchEnds::end, false };
	else if (lastPt() == opp.pts[0])
		result = { result.match | MatchEnds::end, true };
	return result;
}

MatchReverse OpCurve::matchExact(const OpCurve& opp) const {
	MatchReverse result { MatchEnds::none, false };
	OpPoint sStart = c.data->start;
	OpPoint sEnd = c.data->end;
	OP_ASSERT(sStart != sEnd);
	OpPoint oStart = opp.c.data->start;
	OpPoint oEnd = opp.c.data->end;
	OP_ASSERT(oStart != oEnd);
	if (sStart == oStart)
		result = { MatchEnds::start, false };
	else if (sStart == oEnd)
		result = { MatchEnds::start, true };
	if (sEnd == oEnd)
		result = { result.match | MatchEnds::end, false };
	else if (sEnd == oStart)
		result = { result.match | MatchEnds::end, true };
	return result;
}

bool OpCurve::nearBounds(OpPoint pt) const {
	OpPointBounds bounds { firstPt(), lastPt() };
	return bounds.nearlyContains(pt, context().threshold);
}

NormalDirection OpCurve::normalDirection(Axis axis, float t) const {
	OpVector ray = Axis::horizontal == (Axis) ((int) axis & 1) ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	if (Axis::up <= axis)
		ray = -ray;
	float NdotR = normal(t).normalize().dot(ray);
	if (NdotR > 0)
		return NormalDirection::upRight;
	if (NdotR < 0)
		return NormalDirection::downLeft;
	return NormalDirection::underflow;	 // catches, zero, nan
}

float OpCurve::normalLimit() const {
	PathOpsV0Lib::CurveConst limFuncPtr = context().callback(c.type).normalLimitFuncPtr;
	if (!limFuncPtr)
		return 0.008f; // 0.004  fails on testQuads19022897 edge 151 NxR:-0.00746
	return (*limFuncPtr)(c);
}

// all raw intersects are basically the same
// put any specialization (related to debugging?) in some type specific callout ?
OpRoots OpCurve::rawIntersect(const LinePts& linePt, MatchEnds common) const {
	if (linePt.pts[0].x == linePt.pts[1].x) {
		if (linePt.pts[0].x == firstPt().x)
			common |= MatchEnds::start;
		if (linePt.pts[0].x == lastPt().x)
			common |= MatchEnds::end;
		return axisRawHit(Axis::vertical, linePt.pts[0].x, common);
	}
	if (linePt.pts[0].y == linePt.pts[1].y) {
		if (linePt.pts[0].y == firstPt().y)
			common |= MatchEnds::start;
		if (linePt.pts[0].y == lastPt().y)
			common |= MatchEnds::end;
		return axisRawHit(Axis::horizontal, linePt.pts[0].y, common);
	}
	// do not turn result into a line, since rotated is only used to find intersecting t value
	OpCurve isRotated = toVerticalBase(linePt, common);
	if (!isRotated.isFinite()) {
		writableContext().setError(PathOpsV0Lib::ContextError::toVertical  OP_DEBUG_PARAMS(0));
		return OpRoots();
	}
	// if point bounds of rotated doesn't cross y-axis, this is no intersection
	OpRect rotatedBounds = isRotated.fullBounds();
	if (rotatedBounds.right < 0 || rotatedBounds.left > 0)
		return OpRoots();
	OpRoots result = isRotated.axisRawHit(Axis::vertical, 0, common);
	return result;
}

// !!! this should return OpPoint as well as t so caller doesn't have to recompute
OpRoots OpCurve::rayIntersect(const LinePts& line, MatchEnds common) const {
	OpRoots rawRoots = rawIntersect(line, common);
	rawRoots.keepValidTs();
	if (!rawRoots.count() || rawRoots.fail == RootFail::rawIntersectFailed)
		return rawRoots;
	OpRoots realRoots;
	OpVector lineV = line.pts[1] - line.pts[0];
	XyChoice xy = fabsf(lineV.dx) >= fabsf(lineV.dy) ? XyChoice::inX : XyChoice::inY;
	for (float rawRoot : rawRoots.roots) {
		OpPoint hit = ptAtT(rawRoot);
		// in thread_circles36945 : conic mid touches opposite conic only at end point
		// without this fix, in one direction, intersection misses by 2 epsilon, in the other 1 eps
		if (OpMath::InUnsorted(line.pts[0].choice(xy), hit.choice(xy), line.pts[1].choice(xy),
				context().threshold.choice(xy)))
			realRoots.add(rawRoot);
	}
	return realRoots;
}

float OpCurve::tAtXY(float t1, float t2, XyChoice xy, float goal) const {
	OpPair endCheck = xyAtT( { t1, t2 }, xy );
	if (!OpMath::Between(endCheck.s, goal, endCheck.l))
		return OpNaN;
	float mid = OpMath::Average(t1, t2);
	float step = OpMath::Average(mid, -t1);
	while (step >= OpEpsilon) {
		OpPair test = { mid - step, mid + step };
		OpPair x = xyAtT(test, xy);
		bool ordered = x.s < x.l;
		if (ordered ? goal < x.s : goal > x.s)
			mid = test.s;
		else if (ordered ? goal > x.l : goal < x.l)
			mid = test.l;
		step = step / 2;
	}
	return mid <= OpEpsilon ? 0 : mid >= 1 - OpEpsilon ? 1 : mid;
}

#if 0
float OpCurve::tZeroX(float t1, float t2) const {
	OpPair endCheck = xyAtT( { t1, t2 }, XyChoice::inX);
	if (endCheck.s * endCheck.l > 0)  // if both are non zero and same sign, there's no crossing
		return OpNaN;
	float mid = OpMath::Average(t1, t2);
	float step = OpMath::Average(mid, -t1);
	while (step > OpEpsilon) {
		OpPair test = { mid - step, mid + step };
		OpPair x = xyAtT(test, XyChoice::inX);
		if (x.s * x.l > 0)  // both same sign?
			mid = (x.s * endCheck.s > 0) ? test.l : test.s; // same as t1? use step towards t2
		step = step / 2;
	}
	return mid;
}
#endif

void OpCurve::pinCtrl() {
	PathOpsV0Lib::CurvePin funcPtr = context().callback(c.type).curvePinFuncPtr;
	if (funcPtr)
		(*funcPtr)(c, start, end);
	return;
}

// this can fail (if rotated pts are not finite); can happen when input is finite
// however, callers include sort predicate, which cannot return failure; so don't return failure here
// !!! add match ends from caller so that rotated matching end point can guarantee x == 0
// set curve to rotated so subsequent line intersections can check for extrema if needed
OpCurve OpCurve::toVerticalBase(const LinePts& line, MatchEnds match) const {
	OpVector scale = line.pts[1] - line.pts[0];
//	float opp = line.pts[1].y - line.pts[0].y;
    PathOpsV0Lib::Curve cRotated { c.context, nullptr, c.size, c.type };
	OpCurve isRotated(cRotated, Rotated::yes);
	auto rotatePt = [line, scale](OpPoint pt) {
		OpVector v = pt - line.pts[0];
		return OpPoint(scale.cross(v), scale.dot(v));
	};
	isRotated.setFirstPt(rotatePt(firstPt()));
	if (MatchEnds::start & match)
		isRotated.c.data->start.x = 0;
	isRotated.setLastPt(rotatePt(lastPt()));
	if (MatchEnds::end & match)
		isRotated.c.data->end.x = 0;
	PathOpsV0Lib::Rotate funcPtr = context().callback(c.type).rotateFuncPtr;
	if (funcPtr)
		(*funcPtr)(c, line.pts[0], scale, isRotated.c);
	isRotated.start = isRotated.c.data->start;
	isRotated.end = isRotated.c.data->end;
	return isRotated;
}

OpCurve OpCurve::toVertical(const LinePts& line, MatchEnds match) const {
	OpCurve isRotated = toVerticalBase(line, match);
	if (isRotated.isFinite())
		isRotated.isLine();
	return isRotated;
}

int OpCurve::pointCount() const {
	PathOpsV0Lib::HullPtCount funcPtr = context().callback(c.type).ptCountFuncPtr;
	return 2 + (funcPtr ? (*funcPtr)() : 0);
}

OpPoint OpCurve::ptAtT(float t) const {
	if (0 == t)
		return c.data->start;
	if (1 == t)
		return c.data->end;
	PathOpsV0Lib::PtAtT funcPtr = context().callback(c.type).ptAtTFuncPtr;
	OpPoint result = funcPtr ? (*funcPtr)(c, t) : (1 - t) * c.data->start + t * c.data->end;
	// loop8846, testCubics295953 requires pinning on horizontal when there's no function to call
	if (!funcPtr)
		result.pin(c.data->start, c.data->end);
	return result;
}

OpPoint OpCurve::ptDAtT(float t) const {
	if (0 == t)
		return firstPt();
	if (1 == t)
		return lastPt();
	PathOpsV0Lib::PtAtT funcPtr = context().callback(c.type).ptDAtTFuncPtr;
	OpPoint result = funcPtr ? (*funcPtr)(c, t) : (1 - t) * c.data->start + t * c.data->end;
	// !!! required by release_13: but, should caller's point at T function do the pinning?
	// !!! counterpoint: loop8846 requires pinning on horizontal line (there's no function to call)
//	result.pin(firstPt(), lastPt());
	return result;
}

OpCurve OpCurve::subDivide(float t1, float t2) const {
	if (0 == t1 && 1 == t2)
		return *this;
	OpCurve newResult(c, rotated);
    newResult.setFirstPt(ptAtT(t1));
    newResult.setLastPt(ptAtT(t2));
	PathOpsV0Lib::SubDivide funcPtr = context().callback(c.type).subDivideFuncPtr;
	if (funcPtr) {
		(*funcPtr)(c, t1, t2, context().threshold, &newResult.c);
		if (PathOpsV0Lib::degenerateLine == newResult.c.type)
			newResult.setLine();
	}
	return newResult;
}

// for accuracy, this should only be called with segment's curve, never edge curve
OpVector OpCurve::normal(float t) const {
	OpVector tan = tangent(t);
	return { -tan.dy, tan.dx };
}

OpVector OpCurve::tangent(float t) const {
	PathOpsV0Lib::CurveTangent funcPtr = context().callback(c.type).curveTangentFuncPtr;
	if (!funcPtr)
		return lastPt() - firstPt();
	return (*funcPtr)(c, t);
}

OpPair OpCurve::xyAtT(OpPair t, XyChoice xy) const {
	PathOpsV0Lib::XYAtT funcPtr = context().callback(c.type).xyAtTFuncPtr;
	if (!funcPtr)
		return (1 - t) * firstPt().choice(xy) + t * lastPt().choice(xy);
	return (*funcPtr)(c, t, xy);
}

bool OpCurve::zeroSmall(OpContour& contour, bool zeroStart) {
	auto zero_small = [](float in, float thresh) {
		return fabsf(in) <= thresh ? 0 : in;
	};
	OpVector threshold = context().threshold;
	if (zeroStart) {
		start.x = zero_small(c.data->start.x, threshold.dx);
		start.y = zero_small(c.data->start.y, threshold.dy);
	}
	end.x = zero_small(c.data->end.x, threshold.dx);
	end.y = zero_small(c.data->end.y, threshold.dy);
	if (start != c.data->start || end != c.data->end) {
		pinCtrl();
		OP_DEBUG_CODE(debugZeroedSmall = true);
	}
	isSmall = start.isNearly(end, threshold);
	if (isSmall)
		end = start;
	return isSmall;
}

OpPoint OpCurve::hullPt(int index) const {
//	OP_ASSERT(PathOpsV0Lib::degenerateLine != c.type);
	OP_ASSERT(0 <= index && index < pointCount());
	if (0 == index)
		return firstPt();
	if (pointCount() - 1 == index)
		return lastPt();
	OP_ASSERT(context().callback(c.type).curveHullFuncPtr);
	return context().callback(c.type).curveHullFuncPtr(c, index);
}

void OpCurve::reverse() {
	std::swap(c.data->start, c.data->end);
	std::swap(start, end);
	PathOpsV0Lib::CurveReverse funcPtr = context().callback(c.type).curveReverseFuncPtr;
	if (funcPtr)
		(*funcPtr)(c);
    reversed ^= true;
}

bool OpCurve::isLine() {
	if (!isLineSet) {
		isLineSet = true;
		PathOpsV0Lib::CurveIsLine funcPtr = (int) c.type 
				? context().callback(c.type).curveIsLineFuncPtr : nullptr;
		if ((!funcPtr && (!c.type || c.type == lineType()))
                || (*funcPtr)(c, context().thresholdLength)) {
			setLineType();
			isLineResult = true;
		}
	}
	return isLineResult;
}

// This scales the allowable error from vertical by the magnitude of y.
// This works if the numbers are all very small (denormalized).
// !!! Are there platforms that do not support denormalized numbers? Will this work there?
// !!! If y is large, will this increase the error too much?
bool OpCurve::isVertical() const {
	if (firstPt().y == lastPt().y)
		return false;
	float epsilon = std::max(fabsf(firstPt().y), fabsf(lastPt().y)) * OpEpsilon;
	return fabsf(firstPt().x) <= epsilon && fabsf(lastPt().x) <= epsilon; 
}

#if OP_DEBUG
bool OpCurve::debugIsLine() const {
	if (isLineSet)
		return isLineResult;
	return c.type == lineType();
}
#endif

#if OP_DEBUGGER
OpCurve OpCurve::debugSubDivide(float t1, float t2) const {
	if (0 == t1 && 1 == t2)
		return *this;
	OpCurve newResult(c, rotated);
    newResult.setFirstPt(ptAtT(t1));
    newResult.setLastPt(ptAtT(t2));
	PathOpsV0Lib::DebugSubDivide funcPtr = context().debugCallback(c).debugSubDivideFuncPtr;
	if (funcPtr)
		(*funcPtr)(c, t1, t2, &newResult.c);
	return newResult;
}

#endif

OpPointBounds OpCurve::fullBounds() const {
	OpPointBounds result;
	result.set(start, end);
	if (Rotated::yes == rotated) {
		PathOpsV0Lib::SetBounds funcPtr = context().callback(c.type).setBoundsFuncPtr;
		if (funcPtr)
			(*funcPtr)(c, result);
	}
	return result;
}

static PathOpsV0Lib::LoopAttribute loopAttribute(bool firstPt, bool lastPt, bool reversed) {
	return (PathOpsV0Lib::LoopAttribute) ((int) firstPt | (int) lastPt << 1 | (int) reversed << 2); 
}

PathOpsV0Lib::WindKeep OpCurve::bestLoop(PathOpsV0Lib::CurveOutput curveOutput, 
	PathOpsV0Lib::Winding w, bool firstPt, bool lastPt  OP_DEBUG_PARAMS(int parentID)) {
	context().initOutOnce();
	PathOpsV0Lib::CurveType curveType = c.type;
	if (!curveType)
		curveType = lineType();
	PathOpsV0Lib::Curve curve { c.context, c.data, c.size, context().nativeCurveTypes[curveType] };
	PathOpsV0Lib::LoopAttribute attr = loopAttribute(firstPt, lastPt, reversed);
	return (*curveOutput)({ curve, w, attr });
}

PathOpsV0Lib::WindKeep OpCurve::output(PathOpsV0Lib::Winding w, 
		bool firstPt, bool lastPt  OP_DEBUG_RASTER_PARAMS(OpEdge* edge)) {
    PathOpsV0Lib::CurveOutput curveOutput = context().contextCallbacks.curveOutputFuncPtr;
    if (curveOutput) {
    	context().initOutOnce();
        PathOpsV0Lib::CurveType curveType = c.type;
        if (!curveType)
            curveType = lineType();
        PathOpsV0Lib::Curve curve { c.context, c.data, c.size, context().nativeCurveTypes[curveType] };
        PathOpsV0Lib::LoopAttribute attr = loopAttribute(firstPt, lastPt, reversed);
#if OP_TEST_RASTER
		OP_ASSERT(context().debugRaster);
		context().debugRaster->addOutput({ curve, w, attr }, edge);
#endif
	    return (*curveOutput)({ curve, w, attr });
    }
    return PathOpsV0Lib::WindKeep::Discard;
}

OpPoint OpCurve::whichAlias(EdgeMatch match) const {
	return match == EdgeMatch::end ? end : start;
}

OpPoint OpCurve::whichPt(EdgeMatch match) const {
// match may be 'none' if curve was disabled but found in disabled join pass (testCubics56146)
//	OP_ASSERT(match == EdgeMatch::start || match == EdgeMatch::end);
	return match == EdgeMatch::end ? c.data->end : c.data->start;
}
