#include "OpCurve.h"

bool LinePts::isPoint() const {
    return pts[1] == pts[0];
}

bool OpCurve::isLinear() const {
    OP_ASSERT(type >= OpType::quad);
    OpVector diffs[3];
    diffs[0] = pts[1] - pts[0];
    diffs[1] = pts[2] - pts[0];
    bool linear = fabsf(diffs[0].dx * diffs[1].dy - diffs[1].dx * diffs[0].dy) <= OpEpsilon;
    if (!linear)
        return false;
    if (OpType::cubic != type)
        return true;
    diffs[2] = pts[3] - pts[0];
    if (0 == diffs[0].dx && 0 == diffs[0].dy)
        diffs[0] = diffs[1];
    return fabsf(diffs[0].dx * diffs[2].dy - diffs[2].dx * diffs[0].dy) <= OpEpsilon;
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
    OP_ASSERT(OpDebugIntersect::edge == debugIntersect);  
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
    if (!rawRoots.count)
        return rawRoots;
    OpRoots realRoots;
    XyChoice xy = fabsf(line.pts[1].x - line.pts[0].x) >= fabsf(line.pts[1].y - line.pts[0].y) ?
            XyChoice::inX : XyChoice::inY;
    for (unsigned index = 0; index < rawRoots.count; ++index) {
        OpPoint hit = ptAtT(rawRoots.roots[index]);
        if (OpMath::Between(line.pts[0].choice(xy), hit.choice(xy), line.pts[1].choice(xy)))
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
#if OP_DEBUG
	debugIntersect = OpDebugIntersect::edge;
#endif
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

// this can fail (if rotated pts are not finite); can happen when input is finite
// however, callers include sort predicate, which cannot return failure; so don't return failure here
OpCurve OpCurve::toVertical(const LinePts& line) const {
#define TRY_DOUBLE 0 // mistaken in thinking this was required to fix a bug. Disable until needed
#if TRY_DOUBLE
    double adj = (double) line[1].x - line[0].x;
    double opp = (double) line[1].y - line[0].y;
    for (int n = 0; n < pointCount(); ++n) {
        double vdx = (double) pts[n].x - line[0].x;
        double vdy = (double) pts[n].y - line[0].y;
        rotated.pts[n].x = (float) (vdy * adj - vdx * opp);
        rotated.pts[n].y = (float) (vdy * opp + vdx * adj);
    }
#else
    OpCurve rotated;
    float adj = line.pts[1].x - line.pts[0].x;
    float opp = line.pts[1].y - line.pts[0].y;
    for (int n = 0; n < pointCount(); ++n) {
        OpVector v = pts[n] - line.pts[0];
        rotated.pts[n].x = v.dy * adj - v.dx * opp;
        rotated.pts[n].y = v.dy * opp + v.dx * adj;
    }
#endif
    rotated.weight = weight;
    rotated.type = type;
    OP_DEBUG_CODE(rotated.debugIntersect = debugIntersect);
    return rotated;
}
