// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurve.h"

bool LinePts::isPoint() const {
    return pts[1] == pts[0];
}

bool OpCurve::isLinear() const {
    OP_ASSERT(type >= OpType::quad);
    OpVector diffs[2];
    diffs[0] = pts[1] - pts[0];
    diffs[1] = (OpType::cubic == type ? pts[3] : pts[2]) - pts[0];
    bool linear = fabsf(diffs[0].dx * diffs[1].dy - diffs[1].dx * diffs[0].dy) <= OpEpsilon;
    if (!linear)
        return false;
    if (OpType::cubic != type)
        return true;
    diffs[0] = pts[2] - pts[0];
    return fabsf(diffs[0].dx * diffs[1].dy - diffs[1].dx * diffs[0].dy) <= OpEpsilon;
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
        case OpType::cubic: roots = asCubic().axisRawHit(axis, axisIntercept); break;
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

OpRootPts OpCurve::lineIntersect(const LinePts& line, float start, float end) const {
    OpRootPts result;
    result.raw = rawIntersect(line);
    if (RootFail::rawIntersectFailed == result.raw.fail)
        return result;
    result.valid = result.raw;
    result.valid.keepValidTs(start, end);
    if (!result.valid.count)
        return result;
    OpVector lineV = line.pts[1] - line.pts[0];
    XyChoice xy = fabsf(lineV.dx) >= fabsf(lineV.dy) ? XyChoice::inX : XyChoice::inY;
    for (unsigned index = 0; index < result.valid.count; ++index) {
        OpPoint hit = ptAtT(result.valid.roots[index]);
        if (OpMath::Betweenish(line.pts[0].choice(xy), hit.choice(xy), line.pts[1].choice(xy)))
            result.add(hit, result.valid.roots[index]);

    }
    return result;
}

NormalDirection OpCurve::normalDirection(Axis axis, float t) const {
	bool overflow;
	OpVector ray = Axis::horizontal == (Axis) ((int) axis & 1) ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
    if (Axis::up <= axis)
        ray = -ray;
    float NdotR = normal(t).normalize(&overflow).dot(ray);
	if (overflow)
		return NormalDirection::overflow;
	if (NdotR > 0)
		return NormalDirection::upRight;
	if (NdotR < 0)
		return NormalDirection::downLeft;
	return NormalDirection::underflow;	 // catches, zero, nan
}

OpRoots OpCurve::rawIntersect(const LinePts& linePt) const {
    switch (type) {
        case OpType::line: return asLine().rawIntersect(linePt);
        case OpType::quad: return asQuad().rawIntersect(linePt);
        case OpType::conic: return asConic().rawIntersect(linePt);
        case OpType::cubic: return asCubic().rawIntersect(linePt);
        default:
            OP_ASSERT(0);
    }
    return OpRoots();
}

OpRoots OpCurve::rayIntersect(const LinePts& line) const {
    OpRoots rawRoots = rawIntersect(line);
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

const OpCurve& OpCurve::set(OpPoint start, const OpPoint ctrlPts[2], OpPoint end, unsigned ptCount, 
            OpType opType, float w) {
	pts[0] = start;
	unsigned index = 0;
	while (++index < ptCount - 1)
		pts[index] = ctrlPts[index - 1];
	if (1 == ptCount)
		--index;
	pts[index] = end;
	OP_ASSERT(++index == ptCount);
	weight = w;
	type = opType;
	OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::edge);
	return *this;
}

OpCurve OpCurve::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    OpCurve result;
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
    float mid = (t1 + t2) * .5;
    float step = (mid - t1) * .5;
    while (step > OpEpsilon) {
        OpPair test = { mid - step, mid + step };
        OpPair x = xyAtT(test, xy);
        bool ordered = x.s < x.l;
        if (ordered ? goal < x.s : goal > x.s)
            mid = test.s;
        else if (ordered ? goal > x.l : goal < x.l)
            mid = test.l;
        step = step * .5;
    }
    return mid;
}

float OpCurve::tZeroX(float t1, float t2) const {
    OpPair endCheck = xyAtT( { t1, t2 }, XyChoice::inX);
    if (endCheck.s * endCheck.l > 0)  // if both are non zero and same sign, there's no crossing
        return OpNaN;
    float mid = (t1 + t2) * .5;
    float step = (mid - t1) * .5;
    while (step > OpEpsilon) {
        OpPair test = { mid - step, mid + step };
        OpPair x = xyAtT(test, XyChoice::inX);
        if (x.s * x.l > 0)  // both same sign?
            mid = (x.s * endCheck.s > 0) ? test.l : test.s; // same as t1? use step towards t2
        step = step * .5;
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

