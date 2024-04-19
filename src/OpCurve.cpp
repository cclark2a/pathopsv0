// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"
#include "OpTightBounds.h"

bool LinePts::isPoint() const {
    return pts[1] == pts[0];
}

bool OpCurve::isLinear() const {
    OP_ASSERT(type >= OpType::quad);
    OpVector diffs[2];
    diffs[0] = pts[1] - pts[0];
    diffs[1] = (OpType::cubic == type ? pts[3] : pts[2]) - pts[0];
    float cross = diffs[0].cross(diffs[1]);
    bool linear = fabsf(cross) <= OpEpsilon;
    if (!linear)
        return false;
    if (OpType::cubic != type)
        return true;
    diffs[0] = pts[2] - pts[0];
    cross = diffs[0].cross(diffs[1]);
    linear = fabsf(cross) <= OpEpsilon;
#if 0  // this may be necessary for large values
    auto vals = { diffs[0].dx, diffs[0].dy, diffs[1].dx, diffs[1].dy }; 
    auto [min, max] = std::minmax_element( begin(vals), end(vals) );
    auto larger = std::max(fabsf(*min), fabsf(*max));
    linear = fabsf(cross) < OpMath::NextLarger(larger) - larger;
#endif
    return linear;
}

OpLine& OpCurve::asLine() { OP_ASSERT(OpType::line == type); return *static_cast<OpLine*>(this); }
OpQuad& OpCurve::asQuad() { OP_ASSERT(OpType::quad == type); return *static_cast<OpQuad*>(this); }
OpConic& OpCurve::asConic() { OP_ASSERT(OpType::conic == type); return *static_cast<OpConic*>(this); }
OpQuad& OpCurve::asConicQuad() { OP_ASSERT(OpType::conic == type); return *static_cast<OpQuad*>(this); }
OpCubic& OpCurve::asCubic() { OP_ASSERT(OpType::cubic == type); return *static_cast<OpCubic*>(this); }

const OpLine& OpCurve::asLine() const { OP_ASSERT(OpType::line == type); 
        return *static_cast<const OpLine*>(this); }
const OpQuad& OpCurve::asQuad() const { OP_ASSERT(OpType::quad == type); 
        return *static_cast<const OpQuad*>(this); }
const OpConic& OpCurve::asConic() const { OP_ASSERT(OpType::conic == type); 
        return *static_cast<const OpConic*>(this); }
const OpQuad& OpCurve::asConicQuad() const { OP_ASSERT(OpType::conic == type); 
        return *static_cast<const OpQuad*>(this); }
const OpCubic& OpCurve::asCubic() const { OP_ASSERT(OpType::cubic == type); 
        return *static_cast<const OpCubic*>(this); }

OpRoots OpCurve::axisRayHit(Axis axis, float axisIntercept, float start,
            float end) const {
    OpRoots roots;
    switch (type) {
        case OpType::line: roots = asLine().axisRawHit(axis, axisIntercept); break;
        case OpType::quad: roots = asQuad().axisRawHit(axis, axisIntercept); break;
        case OpType::conic: roots = asConic().axisRawHit(axis, axisIntercept); break;
        case OpType::cubic: roots = asCubic().axisRawHit(axis, axisIntercept, 
                MatchEnds::none); break;
        default:
            OP_ASSERT(0); 
    }
    roots.keepValidTs(start, end);
    return roots;
}

// call only with edge generated curves
float OpCurve::center(Axis axis, float intercept) const {
#if USE_SEGMENT_CENTER
    // !!! at present, could be either...
//    OP_ASSERT(OpDebugIntersect::segment == debugIntersect);  
#else
    OP_ASSERT(OpDebugIntersect::edge == debugIntersect);  
#endif
    OpRoots roots;
    switch (type) {
    case OpType::line: { 
        auto ptr = pts[0].asPtr(axis); 
        return (intercept - ptr[0]) / (ptr[2] - ptr[0]); 
    }
    case OpType::quad:
    case OpType::conic:
    case OpType::cubic: 
        roots = axisRayHit(axis, intercept); 
    break;
    default:
        OP_ASSERT(0);
        return OpNaN;
    }
    if (1 != roots.count)
        return OpNaN;   // numerics failed
    return roots.roots[0];
}

#if 0 // unused
int OpCurve::closest(OpPtT* best, float delta, OpPoint pt) const {
    int loop = 0;
    OpPtT test = *best;
    float bestDistSq = (test.pt - pt).lengthSquared();
    do {
        if (++loop >= 256)  // !!! wild guess (likely way too high)
            break;
        float lastT = test.t;
        test.t = std::max(0.f, std::min(1.f, test.t + delta));
        if (lastT == test.t)
            return loop;
        OpPoint lastPt = test.pt;
        test.pt = ptAtT(test.t);
        if (lastPt == test.pt) {
            delta *= 2;
            continue;
        }
        float testDistSq = (test.pt - pt).lengthSquared();
        if (bestDistSq > testDistSq) {
            bestDistSq = testDistSq;
            *best = test;
            delta *= 2;
            continue;
        }
        delta = -delta / 2;
    } while (fabsf(delta) >= OpEpsilon);
    return loop;
}
#endif

// !!! promote types to use double as test cases requiring such are found
OpPoint OpCurve::doublePtAtT(float t) const {
    switch(type) {
        case OpType::line: return asLine().ptAtT(t);    
        case OpType::quad: return asQuad().ptAtT(t);
        case OpType::conic: return asConic().ptAtT(t);
        case OpType::cubic: return asCubic().doublePtAtT(t);
        default:
            OP_ASSERT(0);
    }
    return OpPoint();
}

OpPtT OpCurve::findIntersect(Axis axis, const OpPtT& opPtT) const {
    if (pts[0] == opPtT.pt)
        return { opPtT.pt, 0 };
    if (lastPt() == opPtT.pt)
        return { opPtT.pt, 1 };
    float intercept = *opPtT.pt.asPtr(axis);
    OpRoots roots;
    switch (type) {
    case OpType::line:
    case OpType::quad:
    case OpType::conic:
    case OpType::cubic: 
        roots = axisRayHit(axis, intercept); 
    break;
    default:
        OP_ASSERT(0);
        return OpPtT();
    }
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

bool OpCurve::isFinite() const {
    for (int i = 0; i < pointCount(); ++i)
        if (!pts[i].isFinite())
            return false;
    return OpType::conic != type || OpMath::IsFinite(weight);
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
    OpPointBounds bounds { pts[0], lastPt() };
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

void OpCurve::pinCtrl() {
    switch (type) {
        case OpType::line: return;
        case OpType::quad: return asQuad().pinCtrl();
        case OpType::conic: return asConic().pinCtrl();
        case OpType::cubic: return asCubic().pinCtrls();
        default:
            OP_ASSERT(0);
    }
}

OpRoots OpCurve::rawIntersect(const LinePts& linePt, MatchEnds common) const {
    switch (type) {
        case OpType::line: return asLine().rawIntersect(linePt);
        case OpType::quad: return asQuad().rawIntersect(linePt);
        case OpType::conic: return asConic().rawIntersect(linePt);
        case OpType::cubic: return asCubic().rawIntersect(linePt, common);
        default:
            OP_ASSERT(0);
    }
    return OpRoots();
}

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

// for accuracy, this should only be called with segment's curve, never edge curve
OpVector OpCurve::normal(float t) const {
    switch (type) {
        case OpType::line: return asLine().normal(t);
        case OpType::quad: return asQuad().normal(t);
        case OpType::conic: return asConic().normal(t);
        case OpType::cubic: return asCubic().normal(t);
        default:
            OP_ASSERT(0);
    }
    return OpVector();
}

OpPoint OpCurve::ptAtT(float t) const {
    switch(type) {
        case OpType::line: return asLine().ptAtT(t);    
        case OpType::quad: return asQuad().ptAtT(t);
        case OpType::conic: return asConic().ptAtT(t);
        case OpType::cubic: return asCubic().ptAtT(t);
        default:
            OP_ASSERT(0);
    }
    return OpPoint();
}

void OpCurve::reverse() {
    std::swap(pts[0], pts[pointCount() - 1]);
    if (OpType::cubic == type)
        std::swap(pts[1], pts[2]);
}

#if 0
// control points are set separately
const OpCurve& OpCurve::set(OpPoint start, OpPoint end, unsigned ptCount, 
            OpType opType, float w) {
	pts[0] = start;
	pts[ptCount - 1] = end;
	weight = w;
	type = opType;
	OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::edge);
	return *this;
}
#endif

OpCurve OpCurve::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    OpCurve result;
    result.type = type;
    switch (type) {
        case OpType::line: 
            result.pts[0] = ptT1.pt; 
            result.pts[1] = ptT2.pt;
            result.weight = 1;
            break;
        case OpType::quad: 
            return asQuad().subDivide(ptT1, ptT2);
        case OpType::conic: 
            return asConic().subDivide(ptT1, ptT2);
        case OpType::cubic: 
            return asCubic().subDivide(ptT1, ptT2);
        default:
            OP_ASSERT(0);
    }
    return result;
}

OpVector OpCurve::tangent(float t) const {
    switch (type) {
    case OpType::line: return asLine().tangent();
    case OpType::quad: return asQuad().tangent(t);
    case OpType::conic: return asConic().tangent(t);
    case OpType::cubic: return asCubic().tangent(t);
    default:
        OP_ASSERT(0);
    }
    return OpVector();
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
    rotated.type = type;
    OP_DEBUG_CODE(rotated.debugIntersect = debugIntersect);
    return rotated;
}
#endif

// this can fail (if rotated pts are not finite); can happen when input is finite
// however, callers include sort predicate, which cannot return failure; so don't return failure here
OpCurve OpCurve::toVertical(const LinePts& line) const {
    OpCurve rotated;
    float adj = line.pts[1].x - line.pts[0].x;
    float opp = line.pts[1].y - line.pts[0].y;
    int count = pointCount() + (int) centerPt;
    for (int n = 0; n < count; ++n) {
        OpVector v = pts[n] - line.pts[0];
        rotated.pts[n].x = v.dy * adj - v.dx * opp;
        rotated.pts[n].y = v.dy * opp + v.dx * adj;
    }
    rotated.weight = weight;
    rotated.centerPt = centerPt;
    rotated.type = type;
    OP_DEBUG_CODE(rotated.debugIntersect = debugIntersect);
    return rotated;
}

OpPair OpCurve::xyAtT(OpPair t, XyChoice xy) const {
    switch (type) {
    case OpType::line: return asLine().xyAtT(t, xy);
    case OpType::quad: return asQuad().xyAtT(t, xy);
    case OpType::conic: return asConic().xyAtT(t, xy);
    case OpType::cubic: return asCubic().xyAtT(t, xy);
    default:
        OP_ASSERT(0);
    }
    return OpPair();
}

