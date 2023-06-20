#include "OpCurve.h"

OpLine& OpCurve::asLine() { assert(lineType == type); return *static_cast<OpLine*>(this); }
OpQuad& OpCurve::asQuad() { assert(quadType == type); return *static_cast<OpQuad*>(this); }
OpConic& OpCurve::asConic() { assert(conicType == type); return *static_cast<OpConic*>(this); }
OpQuad& OpCurve::asConicQuad() { assert(conicType == type); return *static_cast<OpQuad*>(this); }
OpCubic& OpCurve::asCubic() { assert(cubicType == type); return *static_cast<OpCubic*>(this); }

const OpLine& OpCurve::asLine() const { assert(lineType == type); 
        return *static_cast<const OpLine*>(this); }
const OpQuad& OpCurve::asQuad() const { assert(quadType == type); 
        return *static_cast<const OpQuad*>(this); }
const OpConic& OpCurve::asConic() const { assert(conicType == type); 
        return *static_cast<const OpConic*>(this); }
const OpQuad& OpCurve::asConicQuad() const { assert(conicType == type); 
        return *static_cast<const OpQuad*>(this); }
const OpCubic& OpCurve::asCubic() const { assert(cubicType == type); 
        return *static_cast<const OpCubic*>(this); }

OpRoots OpCurve::axisRayHit(Axis axis, float axisIntercept, float start,
            float end) const {
    OpRoots roots;
    switch (type) {
        case pointType: break;
        case lineType: roots = asLine().axisRawHit(axis, axisIntercept); break;
        case quadType: roots = asQuad().axisRawHit(axis, axisIntercept); break;
        case conicType: roots = asConic().axisRawHit(axis, axisIntercept); break;
        case cubicType: roots = asCubic().axisRawHit(axis, axisIntercept); break;
        default:
            assert(0); 
    }
    roots.keepValidTs(start, end);
    return roots;
}

// call only with edge generated curves
float OpCurve::center(Axis axis, float intercept) const {
    assert(!OpDebugPathOpsEnable::inPathOps || OpDebugIntersect::edge == debugIntersect);  
    OpRoots roots;
    switch (type) {
    case pointType: return 0;
    case lineType: { 
        auto ptr = pts[0].asPtr(axis); 
        return (intercept - ptr[0]) / (ptr[2] - ptr[0]); 
    }
    case quadType:
    case conicType:
    case cubicType: 
        roots = axisRayHit(axis, intercept); 
    break;
    default:
        assert(0);
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
    case pointType: 
        return { pts[0], 0 };
    case lineType:
    case quadType:
    case conicType:
    case cubicType: 
        roots = axisRayHit(axis, intercept); 
    break;
    default:
        assert(0);
        return OpPtT();
    }
    assert(roots.count);
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
    return conicType != type || OpMath::IsFinite(weight);
}

OpRoots OpCurve::rawIntersect(const LinePts& linePt) const {
    switch (type) {
        case pointType: return OpRoots();
        case lineType: return asLine().rawIntersect(linePt);
        case quadType: return asQuad().rawIntersect(linePt);
        case conicType: return asConic().rawIntersect(linePt);
        case cubicType: return asCubic().rawIntersect(linePt);
        default:
            assert(0);
    }
    return OpRoots();
}

OpRoots OpCurve::rayIntersect(const LinePts& linePt) const {
    switch (type) {
        case pointType: return OpRoots();
        case lineType: return asLine().rayIntersect(linePt);
        case quadType: return asQuad().rayIntersect(linePt);
        case conicType: return asConic().rayIntersect(linePt);
        case cubicType: return asCubic().rayIntersect(linePt);
        default:
            assert(0);
    }
    return OpRoots();
}

// for accuracy, this should only be called with segment's curve, never edge curve
OpVector OpCurve::normal(float t) const {
    switch (type) {
        case pointType: return OpVector();
        case lineType: return asLine().normal(t);
        case quadType: return asQuad().normal(t);
        case conicType: return asConic().normal(t);
        case cubicType: return asCubic().normal(t);
        default:
            assert(0);
    }
    return OpVector();
}

OpPoint OpCurve::ptAtT(float t) const {
    switch(type) {
        case pointType: return pts[0];
        case lineType: return asLine().ptAtT(t);    
        case quadType: return asQuad().ptAtT(t);
        case conicType: return asConic().ptAtT(t);
        case cubicType: return asCubic().ptAtT(t);
        default:
            assert(0);
    }
    return OpPoint();
}

CurvePts OpCurve::subDivide(OpPtT ptT1, OpPtT ptT2) const {
    CurvePts result;
    switch (type) {
        case lineType: 
            result.pts[1] = ptT2.pt;
            [[fallthrough]];
        case pointType: 
            result.pts[0] = ptT1.pt; 
            result.weight = 1;
            break;
        case quadType: 
            return asQuad().subDivide(ptT1, ptT2);
        case conicType: 
            return asConic().subDivide(ptT1, ptT2);
        case cubicType: 
            return asCubic().subDivide(ptT1, ptT2);
        default:
            assert(0);
    }
    return result;
}

OpVector OpCurve::tangent(float t) const {
    switch (type) {
    case pointType: return OpVector();
    case lineType: return asLine().tangent();
    case quadType: return asQuad().tangent(t);
    case conicType: return asConic().tangent(t);
    case cubicType: return asCubic().tangent(t);
    default:
        assert(0);
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
