// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"
#include "OpContour.h"
#include "OpTightBounds.h"
#if OP_DEBUG
#include "OpDebugRaster.h"
#endif

OpRoots OpCurve::axisRayHit(Axis axis, float axisIntercept, float start, float end) const {
	OpRoots roots = axisRawHit(axis, axisIntercept, MatchEnds::none);
	roots.keepValidTs(start, end);
	return roots;
}

float OpCurve::center(Axis axis, float intercept) const {
	OpRoots roots = axisRayHit(axis, intercept);
	if (1 != roots.count())
		return OpNaN;   // numerics failed
	return roots.roots[0];
}

// cut range minimum should be double the distance between ptT pt and opp pt
CutRangeT OpCurve::cutRange(const OpPtT& ptT, OpPoint oppPt, float loEnd, float hiEnd) const {
	PathOpsV0Lib::CurveConst cutFun = contours->callBack(c.type).cutFuncPtr;
	float tStep = cutFun ? (*cutFun)(c) : 16.f;
	float cutDt = OpEpsilon * tStep;
	OpVector threshold = contours->threshold();
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

float OpCurve::interceptLimit() const {
	PathOpsV0Lib::CurveConst limFuncPtr = contours->callBack(c.type).interceptFuncPtr;
	if (!limFuncPtr)
		return 1.f / 256.f;
	return (*limFuncPtr)(c);
}

OpRootPts OpCurve::lineIntersect(const LinePts& line) const {
	OpRootPts result;
	result.raw = rawIntersect(line, MatchEnds::none);
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
				contours->threshold().choice(xy))) {
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
}

float OpCurve::match(float start, float end, OpPoint pt) const {
	if (!nearBounds(pt))
		return OpNaN;
	float xRoot = tAtXY(start, end, XyChoice::inX, pt.x);
	float yRoot = tAtXY(start, end, XyChoice::inY, pt.y);
	if (OpMath::EqualT(xRoot, yRoot))
		return xRoot;
	OpPoint xPt = ptAtT(xRoot);
	OpPoint yPt = ptAtT(yRoot);
	float xDistSq = (pt - xPt).lengthSquared();
	float yDistSq = (pt - yPt).lengthSquared();
	// example: testQuads9421393 needs small curve factor for segs (3, 7 to detect intersection)
	OpVector slop = contours->threshold();  
//			* contours->callBack(c.type).matchSlopFuncPtr();
	if (!(xDistSq > yDistSq)) {  // reverse test in case y dist is nan
		if (pt.isNearly(xPt, slop))
			return xRoot;
	}
	return pt.isNearly(yPt, slop) ? yRoot : OpNaN;
}

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

bool OpCurve::nearBounds(OpPoint pt) const {
	OpPointBounds bounds { firstPt(), lastPt() };
	return bounds.nearlyContains(pt, contours->threshold());
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
	PathOpsV0Lib::CurveConst limFuncPtr = contours->callBack(c.type).normalLimitFuncPtr;
	if (!limFuncPtr)
		return 0.008f; // 0.004  fails on testQuads19022897 edge 151 NxR:-0.00746
	return (*limFuncPtr)(c);
}

bool OpCurve::normalize() {
	auto zeroSmall = [](float* value, float threshold) {
		if (*value && fabsf(*value) <= threshold) {
			*value = 0;
			return true;
		}
		return false;
	};
	bool recomputeBounds = false;
	OpPtAliases& aliases = contours->aliases;
	OpVector threshold = aliases.threshold;
	recomputeBounds |= zeroSmall(&c.data->start.x, threshold.dx);
	recomputeBounds |= zeroSmall(&c.data->start.y, threshold.dy);
	recomputeBounds |= zeroSmall(&c.data->end.x, threshold.dx);
	recomputeBounds |= zeroSmall(&c.data->end.y, threshold.dy);
	OpPoint smaller = aliases.existing(c.data->start);
	recomputeBounds |= smaller != c.data->start;
	OpPoint larger = aliases.existing(c.data->end);
	recomputeBounds |= larger != c.data->end;
	if (smaller != larger && smaller.isNearly(larger, threshold)) {
		float smallerLen = OpVector(smaller).lengthSquared();
		float largerLen = OpVector(larger).lengthSquared();
		bool swap = (smallerLen > largerLen && !aliases.contains(larger)) 
				|| aliases.contains(smaller);
		if (swap)
			std::swap(larger, smaller);
		if (contours->addAlias(larger, smaller))
			(swap ? c.data->end : c.data->start) = smaller;
		else
			contours->remapPts(larger, smaller);
		recomputeBounds = true;
	}
	if (recomputeBounds)
		pinCtrl();
	return recomputeBounds;
}

// all raw intersects are basically the same
// put any specialization (related to debugging?) in some type specific callout ?
OpRoots OpCurve::rawIntersect(const LinePts& linePt, MatchEnds common) const {
	if (linePt.pts[0].x == linePt.pts[1].x)
		return axisRawHit(Axis::vertical, linePt.pts[0].x, common);
	if (linePt.pts[0].y == linePt.pts[1].y)
		return axisRawHit(Axis::horizontal, linePt.pts[0].y, common);
	OpCurve rotated = toVertical(linePt, common);
	if (!rotated.isFinite()) {
		contours->setError(PathOpsV0Lib::ContextError::toVertical  OP_DEBUG_PARAMS(0));
		return OpRoots();
	}
	// if point bounds of rotated doesn't cross y-axis, this is no intersection
	OpRect rotatedBounds = rotated.ptBounds();
	if (rotatedBounds.right < 0 || rotatedBounds.left > 0)
		return OpRoots();
	OpRoots result = rotated.axisRawHit(Axis::vertical, 0, common);
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
				contours->threshold().choice(xy)))
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
	return mid;
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

#include "OpContour.h"

void OpCurve::pinCtrl() {
	PathOpsV0Lib::CurvePinCtrl funcPtr = contours->callBack(c.type).curvePinCtrlFuncPtr;
	if (funcPtr)
		(*funcPtr)(c);
	return;
}

bool OpCurve::isFinite() const {
	if (!c.data->start.isFinite())
		return false;
	if (!c.data->end.isFinite())
		return false;
	PathOpsV0Lib::CurveIsFinite funcPtr = contours->callBack(c.type).curveIsFiniteFuncPtr;
	return funcPtr ? (*funcPtr)(c) : true;
}

// this can fail (if rotated pts are not finite); can happen when input is finite
// however, callers include sort predicate, which cannot return failure; so don't return failure here
// !!! add match ends from caller so that rotated matching end point can guarantee x == 0
OpCurve OpCurve::toVertical(const LinePts& line, MatchEnds match) const {
	OpVector scale = line.pts[1] - line.pts[0];
//	float opp = line.pts[1].y - line.pts[0].y;
	OpCurve rotated(contours, { nullptr, c.size, c.type } );
	auto rotatePt = [line, scale](OpPoint pt) {
		OpVector v = pt - line.pts[0];
		return OpPoint(scale.cross(v), scale.dot(v));
	};
	rotated.c.data->start = rotatePt(c.data->start);
	if (MatchEnds::start & match)
		rotated.c.data->start.x = 0;
	rotated.c.data->end = rotatePt(c.data->end);
	if (MatchEnds::end & match)
		rotated.c.data->end.x = 0;
	PathOpsV0Lib::Rotate funcPtr = contours->callBack(c.type).rotateFuncPtr;
	if (funcPtr)
		(*funcPtr)(c, line.pts[0], scale, rotated.c);
	return rotated;
}

int OpCurve::pointCount() const {
	PathOpsV0Lib::HullPtCount funcPtr = contours->callBack(c.type).ptCountFuncPtr;
	return 2 + (funcPtr ? (*funcPtr)() : 0);
}

OpPoint OpCurve::ptAtT(float t) const {
	if (0 == t)
		return c.data->start;
	if (1 == t)
		return c.data->end;
	PathOpsV0Lib::PtAtT funcPtr = contours->callBack(c.type).ptAtTFuncPtr;
	if (!funcPtr)
		return (1 - t) * c.data->start + t * c.data->end;
	return (*funcPtr)(c, t);
}

OpCurve OpCurve::subDivide(OpPtT ptT1, OpPtT ptT2) const {
	PathOpsV0Lib::Curve newCurve { c.data, c.size, c.type };
	OpCurve newResult(contours, newCurve);
    newResult.c.data->start = ptT1.pt;
    newResult.c.data->end = ptT2.pt;
	PathOpsV0Lib::SubDivide funcPtr = contours->callBack(c.type).subDivideFuncPtr;
	if (funcPtr)
		(*funcPtr)(c, ptT1.t, ptT2.t, newResult.c);
	return newResult;
}

// for accuracy, this should only be called with segment's curve, never edge curve
OpVector OpCurve::normal(float t) const {
	OpVector tan = tangent(t);
	return { -tan.dy, tan.dx };
}

OpVector OpCurve::tangent(float t) const {
	PathOpsV0Lib::CurveTangent funcPtr = contours->callBack(c.type).curveTangentFuncPtr;
	if (!funcPtr)
		return c.data->end - c.data->start;
	return (*funcPtr)(c, t);
}

OpPair OpCurve::xyAtT(OpPair t, XyChoice xy) const {
	PathOpsV0Lib::XYAtT funcPtr = contours->callBack(c.type).xyAtTFuncPtr;
	if (!funcPtr)
		return (1 - t) * c.data->start.choice(xy) + t * c.data->end.choice(xy);
	return (*funcPtr)(c, t, xy);
}

OpPoint OpCurve::hullPt(int index) const {
	OP_ASSERT((PathOpsV0Lib::CurveType) 0 == c.type || (0 <= index && index < pointCount()));
	if (0 == index)
		return c.data->start;
	if (pointCount() - 1 == index)
		return c.data->end;
	OP_ASSERT(contours->callBack(c.type).curveHullFuncPtr);
	return contours->callBack(c.type).curveHullFuncPtr(c, index);
}

void OpCurve::reverse() {
	std::swap(c.data->start, c.data->end);
	PathOpsV0Lib::CurveReverse funcPtr = contours->callBack(c.type).curveReverseFuncPtr;
	if (funcPtr)
		(*funcPtr)(c);
}

OpCurve::OpCurve(OpContours* cntrs, PathOpsV0Lib::Curve curve) {
	contours = cntrs;
	c.size = curve.size;
	c.data = contours->allocateCurveData(c.size);
	if (curve.data)
		std::memcpy(c.data, curve.data, c.size);
	c.type = curve.type;
	PathOpsV0Lib::CurveIsLine funcPtr = contours->callBack(c.type).curveIsLineFuncPtr;
	if (!funcPtr) {
		isLineSet = true;
		isLineResult = true;
	} else {
		isLineSet = false;
		isLineResult = false;
	}
}

OpRoots OpCurve::axisRawHit(Axis axis, float intercept, MatchEnds matchEnds) const {
	PathOpsV0Lib::AxisT func = contours->callBack(c.type).axisTFuncPtr;
	if (!func) {
		const float* ptr = c.data->start.asPtr(axis);
		return OpRoots((intercept - ptr[0]) / (ptr[2] - ptr[0]));
	}
	return (*func)(c, axis, intercept, matchEnds);
}

bool OpCurve::isLine() {
	if (isLineSet)
		return isLineResult;
	isLineSet = true;
	PathOpsV0Lib::CurveIsLine funcPtr = contours->callBack(c.type).curveIsLineFuncPtr;
	OP_ASSERT(funcPtr);  // !!! can non-line omit this?
	if (!funcPtr || (*funcPtr)(c)) {
		c.type = contours->contextCallBacks.setLineTypeFuncPtr(c);
		return isLineResult = true;
	}
	return false;
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
	return c.type == contours->contextCallBacks.setLineTypeFuncPtr(c);
}
#endif

OpPointBounds OpCurve::ptBounds() const {
	OpPointBounds result;
	result.set(c.data->start, c.data->end);
	PathOpsV0Lib::SetBounds funcPtr = contours->callBack(c.type).setBoundsFuncPtr;
	if (funcPtr)
		(*funcPtr)(c, result);
	return result;
}

void OpCurve::output(bool firstPt, bool lastPt  OP_DEBUG_PARAMS(int parentID)) {
	contours->initOutOnce();
	contours->callBack(c.type).curveOutputFuncPtr(c, firstPt, lastPt, contours->callerOutput);
#if OP_DEBUG && TEST_RASTER
	if (contours->rasterEnabled) {
		contours->sampleOutputs.addCurveXatY(c  OP_DEBUG_PARAMS(parentID));
		contours->sampleOutputs.addCurveYatX(c  OP_DEBUG_PARAMS(parentID));
	}
#endif
}
