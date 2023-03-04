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

int OpCurve::axisRayHit(Axis axis, float axisIntercept, rootCellar& cepts, float start,
            float end) const {
    int roots;
    switch (type) {
    case pointType: return 0;
    case lineType: roots = asLine().axisRawHit(axis, axisIntercept, cepts); break;
    case quadType: roots = asQuad().axisRawHit(axis, axisIntercept, cepts); break;
    case conicType: roots = asConic().axisRawHit(axis, axisIntercept, cepts); break;
    case cubicType: roots = asCubic().axisRawHit(axis, axisIntercept, cepts); break;
    default:
        assert(0); 
        return 0;
    }
    roots = OpMath::KeepValidTs(cepts, roots, start, end);
    return roots;
}

// call only with edge generated curves
float OpCurve::center(Axis axis, float intercept) const {
    assert(!OpDebugPathOpsEnable::inPathOps || OpDebugIntersect::edge == debugIntersect);  
    rootCellar cepts;
    int count;
    switch (type) {
    case pointType: return 0;
    case lineType: { 
        auto ptr = pts[0].asPtr(axis); 
        return (intercept - ptr[0]) / (ptr[2] - ptr[0]); 
    }
    case quadType:
    case conicType:
    case cubicType: 
        count = axisRayHit(axis, intercept, cepts); 
    break;
    default:
        assert(0);
        return OpNaN;
    }
    if (1 != count)
        return OpNaN;   // numerics failed
    return cepts[0];
}

OpPtT OpCurve::findIntersect(Axis axis, const OpPtT& opPtT) const {
    int count;
    float intercept = *opPtT.pt.asPtr(axis);
    rootCellar cepts;
    switch (type) {
    case pointType: 
        return { pts[0], 0 };
    case lineType:
    case quadType:
    case conicType:
    case cubicType: 
        count = axisRayHit(axis, intercept, cepts); 
    break;
    default:
        assert(0);
        return OpPtT();
    }
    assert(count);
    OpPtT result;
    float best = OpInfinity;
    for (int index = 0; index < count; ++index) {
        OpPoint pt = ptAtT(cepts[index]);
        float distance = fabsf(*(&pt.y - +axis) - *(&opPtT.pt.y - +axis));
        if (best > distance) {
            result = { pt, cepts[index] };
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

int OpCurve::rawIntersect(const std::array<OpPoint, 2> linePt, rootCellar& cepts) const {
    switch (type) {
    case pointType: return 0;
    case lineType: return asLine().rawIntersect(linePt, cepts);
    case quadType: return asQuad().rawIntersect(linePt, cepts);
    case conicType: return asConic().rawIntersect(linePt, cepts);
    case cubicType: return asCubic().rawIntersect(linePt, cepts);
    default:
        assert(0);
    }
    return 0;
}

int OpCurve::rayIntersect(const std::array<OpPoint, 2> linePt, rootCellar& cepts) const {
    switch (type) {
    case pointType: return 0;
    case lineType: return asLine().rayIntersect(linePt, cepts);
    case quadType: return asQuad().rayIntersect(linePt, cepts);
    case conicType: return asConic().rayIntersect(linePt, cepts);
    case cubicType: return asCubic().rayIntersect(linePt, cepts);
    default:
        assert(0);
    }
    return 0;
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
            return OpPoint();
    }
}

void OpCurve::subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dest, float* w) const {
    *w = 1;
    switch (type) {
    case pointType: 
        dest[0] = ptT1.pt; 
        return;
    case lineType: 
        dest[0] = ptT1.pt; 
        dest[1] = ptT2.pt; 
        return;
    case quadType: 
        return asQuad().subDivide(ptT1, ptT2, dest);
    case conicType: 
        return asConic().subDivide(ptT1, ptT2, dest, w);
    case cubicType: 
        return asCubic().subDivide(ptT1, ptT2, dest);
    default:
        assert(0);
    }
}

OpVector OpCurve::tangent(float t) const {
    switch (type) {
    case pointType: return OpVector();
    case lineType: return asLine().tangent(t);
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
void OpCurve::toVertical(const std::array<OpPoint, 2> line, OpCurve& rotated) const {
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
    float adj = line[1].x - line[0].x;
    float opp = line[1].y - line[0].y;
    for (int n = 0; n < pointCount(); ++n) {
        OpVector v = pts[n] - line[0];
        rotated.pts[n].x = v.dy * adj - v.dx * opp;
        rotated.pts[n].y = v.dy * opp + v.dx * adj;
    }
#endif
    rotated.weight = weight;
    rotated.type = type;
    OP_DEBUG_CODE(rotated.debugIntersect = debugIntersect);
}
