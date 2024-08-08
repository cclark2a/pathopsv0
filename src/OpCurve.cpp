// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"
#include "OpTightBounds.h"

OpRoots OpCurve::axisRayHit(Axis axis, float axisIntercept, float start, float end) const {
    OpRoots roots = axisRawHit(axis, axisIntercept, MatchEnds::none);
    roots.keepValidTs(start, end);
    return roots;
}

float OpCurve::center(Axis axis, float intercept) const {
    OpRoots roots = axisRayHit(axis, intercept);
    if (1 != roots.count)
        return OpNaN;   // numerics failed
    return roots.roots[0];
}

OpPtT OpCurve::cut(const OpPtT& ptT, float loBounds, float hiBounds, float direction) const {
    OP_ASSERT(1 == fabsf(direction));
    OP_ASSERT(loBounds <= ptT.t && ptT.t <= hiBounds);
	constexpr float tStep = 16;  // !!! just a guess
	constexpr float cutDt = OpEpsilon * tStep;
	OpVector cutDxy = { OpMath::NextLarger(ptT.pt.x) - ptT.pt.x,
			OpMath::NextLarger(ptT.pt.y) - ptT.pt.y };
	float minDistanceSq = cutDxy.lengthSquared() * tStep;
	OpPtT cut;
	do {
		cut.t = ptT.t + direction * cutDt;
        if (loBounds >= ptT.t || ptT.t >= hiBounds)
            return ptTAtT(OpMath::Average(loBounds, hiBounds));
        if (OpMath::Equalish(loBounds, cut.t) || OpMath::Equalish(cut.t, hiBounds))
            continue;
		cut.pt = ptAtT(cut.t);
        if ((cut.pt - ptT.pt).lengthSquared() >= minDistanceSq)
            break;
	} while ((direction *= tStep));
	return cut;
}

CutRangeT OpCurve::cutRange(const OpPtT& ptT, float loEnd, float hiEnd) const {
	constexpr float tStep = 16;  // !!! just a guess 
	constexpr float cutDt = OpEpsilon * tStep;
	OpVector cutDxy = { OpMath::NextLarger(ptT.pt.x) - ptT.pt.x,
			OpMath::NextLarger(ptT.pt.y) - ptT.pt.y };
	float minDistanceSq = cutDxy.lengthSquared() * tStep;
	CutRangeT tRange;
	for (float direction : { -1, 1 }) {
		OpPtT cut;
		float dir = direction;
		do {
			cut.t = std::max(loEnd, std::min(hiEnd, ptT.t + dir * cutDt));
			cut.pt = ptAtT(cut.t);
		} while ((cut.pt - ptT.pt).lengthSquared() < minDistanceSq 
				 && loEnd < cut.t && cut.t < hiEnd && (dir *= tStep));
		(-1 == direction ? tRange.lo : tRange.hi) = cut;
	}
	return tRange;
}

OpPoint OpCurve::end(float t) const {
    OP_ASSERT(0 == t || 1 == t);
    return t ? lastPt() : firstPt();
}

OpPtT OpCurve::findIntersect(Axis axis, const OpPtT& opPtT) const {
    if (firstPt() == opPtT.pt)
        return { opPtT.pt, 0 };
    if (lastPt() == opPtT.pt)
        return { opPtT.pt, 1 };
    float intercept = *opPtT.pt.asPtr(axis);
    OpRoots roots = axisRayHit(axis, intercept);
    OP_ASSERT(roots.count);
    OpPtT result;
    float best = OpInfinity;
    for (unsigned index = 0; index < roots.count; ++index) {
        OpPoint pt = ptAtT(roots.roots[index]);
        float distance = fabsf(*(&pt.y - +axis) - *(&opPtT.pt.y - +axis));
        if (best > distance) {
            result = { pt, roots.roots[index] };
            best = distance;
        }
    }
    return result;
}

OpRootPts OpCurve::lineIntersect(const LinePts& line) const {
    OpRootPts result;
    result.raw = rawIntersect(line, MatchEnds::none);
    if (RootFail::rawIntersectFailed == result.raw.fail)
        return result;
    result.valid = result.raw;
    result.valid.keepValidTs(0, 1);
    if (!result.valid.count)
        return result;
    OpVector lineV = line.pts[1] - line.pts[0];
    XyChoice xy = fabsf(lineV.dx) >= fabsf(lineV.dy) ? XyChoice::inX : XyChoice::inY;
    for (unsigned index = 0; index < result.valid.count; ++index) {
        OpPoint hit = doublePtAtT(result.valid.roots[index]);
#if 0 && OP_DEBUG_IMAGE
        static int debugDraw = 0;
        if (++debugDraw == 26) {
            std::vector<OpPtT> ptts;
            for (int di = -8; di < 8; ++di) {
                float t = result.valid.roots[index] - OpEpsilon * di;
                if (t > 0 && t < 1) {
                    ptts.push_back( { doublePtAtT(t), t } );
                    ::draw(ptts.back());
                }
            }
             OpDebugOut("");
       }
#endif
        // thread_cubics23476 edges 55 & 52 trigger this need for betweenish
        if (OpMath::Betweenish(line.pts[0].choice(xy), hit.choice(xy), line.pts[1].choice(xy))) {
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
	if (OpMath::Equalish(xRoot, yRoot))
		return xRoot;
    OpPoint xPt = ptAtT(xRoot);
	OpPoint yPt = ptAtT(yRoot);
    float xDistSq = (pt - xPt).lengthSquared();
    float yDistSq = (pt - yPt).lengthSquared();
    float closest = xDistSq < yDistSq ? xRoot : yRoot;
    // !!! can probably optimize this to give up if closest is large -- need to instrument to figure
    //     out how large large needs to be to give up safely
    float lesser = std::max(closest - OpEpsilon, 0.f);
    float greater = std::min(closest + OpEpsilon, 1.f);
    OpPointBounds bounds { ptAtT(lesser), ptAtT(greater) };
    if (bounds.nearlyContains(pt))
        return closest;
    return OpNaN;
}

bool OpCurve::nearBounds(OpPoint pt) const {
    OpPointBounds bounds { firstPt(), lastPt() };
    return bounds.nearlyContains(pt);
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

// all raw intersects are basically the same
// put any specialization (related to debugging?) in some type specific callout ?
OpRoots OpCurve::rawIntersect(const LinePts& linePt, MatchEnds common) const {
    if (linePt.pts[0].x == linePt.pts[1].x)
        return axisRawHit(Axis::vertical, linePt.pts[0].x, common);
    if (linePt.pts[0].y == linePt.pts[1].y)
        return axisRawHit(Axis::horizontal, linePt.pts[0].y, common);
    OpCurve rotated = toVertical(linePt);
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
    if (!rawRoots.count || rawRoots.fail == RootFail::rawIntersectFailed)
        return rawRoots;
    OpRoots realRoots;
    OpVector lineV = line.pts[1] - line.pts[0];
    XyChoice xy = fabsf(lineV.dx) >= fabsf(lineV.dy) ? XyChoice::inX : XyChoice::inY;
    for (unsigned index = 0; index < rawRoots.count; ++index) {
        OpPoint hit = ptAtT(rawRoots.roots[index]);
        // in thread_circles36945 : conic mid touches opposite conic only at end point
        // without this fix, in one direction, intersection misses by 2 epsilon, in the other 1 eps
        if (OpMath::Betweenish(line.pts[0].choice(xy), hit.choice(xy), line.pts[1].choice(xy)))
            realRoots.add(rawRoots.roots[index]);
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

// !!! debugging failure in thread_cubics8753
#if 0
OpCurve OpCurve::toVerticalDouble(const LinePts& line) const {
    OpCurve rotated;
    double adj = (double) line.pts[1].x - line.pts[0].x;
    double opp = (double) line.pts[1].y - line.pts[0].y;
    for (int n = 0; n < pointCount(); ++n) {
        double vdx = (double) pts[n].x - line.pts[0].x;
        double vdy = (double) pts[n].y - line.pts[0].y;
        rotated.pts[n].x = (float) (vdy * adj - vdx * opp);
        rotated.pts[n].y = (float) (vdy * opp + vdx * adj);
    }
    rotated.weight = weight;
    rotated.c.type = c.type;
    return rotated;
}
#endif

#include "OpContour.h"

void OpCurve::pinCtrl() {
    contours->callBack(c.type).curvePinCtrlFuncPtr(c);
    return;
}

bool OpCurve::isFinite() const {
    if (!c.data->start.isFinite())
        return false;
    if (!c.data->end.isFinite())
        return false;
    return contours->callBack(c.type).curveIsFiniteFuncPtr(c);
}

// this can fail (if rotated pts are not finite); can happen when input is finite
// however, callers include sort predicate, which cannot return failure; so don't return failure here
OpCurve OpCurve::toVertical(const LinePts& line) const {
    float adj = line.pts[1].x - line.pts[0].x;
    float opp = line.pts[1].y - line.pts[0].y;
    OpCurve rotated(contours, { nullptr, c.size, c.type } );
    auto rotatePt = [line, adj, opp](OpPoint pt) {
        OpVector v = pt - line.pts[0];
        return OpPoint(v.dy * adj - v.dx * opp, v.dy * opp + v.dx * adj);
    };
    rotated.c.data->start = rotatePt(c.data->start);
    rotated.c.data->end = rotatePt(c.data->end);
    contours->callBack(c.type).rotateFuncPtr(c, line, adj, opp, rotated.c);
    return rotated;
}

int OpCurve::pointCount() const {
    return contours->callBack(c.type).ptCountFuncPtr();
}

// !!! promote types to use double as test cases requiring such are found
OpPoint OpCurve::doublePtAtT(float t) const {
    return contours->callBack(c.type).doublePtAtTFuncPtr(c, t);
}

OpPoint OpCurve::ptAtT(float t) const {
    return contours->callBack(c.type).ptAtTFuncPtr(c, t);
}

OpCurve OpCurve::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    PathOpsV0Lib::Curve newCurve { c.data, c.size, c.type };
    OpCurve newResult(contours, newCurve);
    contours->callBack(c.type).subDivideFuncPtr(c, ptT1, ptT2, newResult.c);
    return newResult;
}

// for accuracy, this should only be called with segment's curve, never edge curve
OpVector OpCurve::normal(float t) const {
    return contours->callBack(c.type).curveNormalFuncPtr(c, t);
}

OpVector OpCurve::tangent(float t) const {
    return contours->callBack(c.type).curveTangentFuncPtr(c, t);
}

OpPair OpCurve::xyAtT(OpPair t, XyChoice xy) const {
    return contours->callBack(c.type).xyAtTFuncPtr(c, t, xy);
}

OpPoint OpCurve::hullPt(int index) const {
    OP_ASSERT((OpType) 0 == c.type || 0 <= index && index < pointCount());
    if (0 == index)
        return c.data->start;
    if (pointCount() - 1 == index)
        return c.data->end;
    return contours->callBack(c.type).curveHullFuncPtr(c, index);
}

void OpCurve::reverse() {
    std::swap(c.data->start, c.data->end);
    contours->callBack(c.type).curveReverseFuncPtr(c);
    return;
}

OpCurve::OpCurve(OpContours* cntrs, PathOpsV0Lib::Curve curve) {
    contours = cntrs;
    c.size = curve.size;
    c.data = contours->allocateCurveData(c.size);
    if (curve.data)
        std::memcpy(c.data, curve.data, c.size);
    c.type = curve.type;
    isLineSet = false;
    isLineResult = false;
}

OpRoots OpCurve::axisRawHit(Axis offset, float intercept, MatchEnds matchEnds) const {
    return contours->callBack(c.type).axisRawHitFuncPtr(c, offset, intercept, matchEnds);
}

bool OpCurve::isLine() {
    if (isLineSet)
        return isLineResult;
    isLineSet = true;
    if (contours->callBack(c.type).curveIsLineFuncPtr(c)) {
        c.type = contours->contextCallBacks.setLineTypeFuncPtr(c);
        return isLineResult = true;
    }
    return false;
}

#if OP_DEBUG
bool OpCurve::debugIsLine() const {
    if (isLineSet)
        return isLineResult;
    return c.type == contours->contextCallBacks.setLineTypeFuncPtr(c);
}
#endif

OpPoint OpCurve::firstPt() const {
    return c.data->start; 
} 

OpPoint OpCurve::lastPt() const {
    return c.data->end; 
} 

void OpCurve::setFirstPt(OpPoint pt) {
    c.data->start = pt;
}

void OpCurve::setLastPt(OpPoint pt) {
    c.data->end = pt;
}

OpPointBounds OpCurve::ptBounds() const {
    OpPointBounds result;
    result.set(c.data->start, c.data->end);
    contours->callBack(c.type).setBoundsFuncPtr(c, result);
    return result;
}

void OpCurve::output(bool firstPt, bool lastPt) {
    contours->callBack(c.type).curveOutputFuncPtr(c, firstPt, lastPt, 
            contours->callerOutput);
}
