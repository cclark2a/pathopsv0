#ifndef OpCurve_DEFINED
#define OpCurve_DEFINED

#include "OpMath.h"

struct OpLine;
struct OpQuad;
struct OpConic;
struct OpCubic;

enum OpType {
    noType = -1,
    pointType,
    lineType,
    quadType,
    conicType,
    cubicType
};

struct OpCurve {
    OpCurve() 
        : weight(1)
        , type(noType) {
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(const OpPoint p[], OpType t) 
        : weight(1)
        , type(t) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(const OpPoint p[], float w, OpType t)
        : weight(w)
        , type(t) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(OpPoint p0, OpPoint p1)
        : weight(1)
        , type(lineType) {
        pts[0] = p0;
        pts[1] = p1;
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(const OpPoint p[], float w)
        : weight(w)
        , type(conicType) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpLine& asLine();
    OpQuad& asQuad();
    OpConic& asConic();
    OpQuad& asConicQuad();
    OpCubic& asCubic();
    const OpLine& asLine() const;
    const OpQuad& asQuad() const;
    const OpConic& asConic() const;
    const OpQuad& asConicQuad() const;
    const OpCubic& asCubic() const;

    int axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts, float start = 0,
            float end = 1) const;
    float center(Axis offset, float axisIntercept) const;
    OpPtT findIntersect(Axis offset, const OpPtT& ) const;
    bool isFinite() const;

    OpPoint lastPt() const {
        return pts[pointCount() - 1];
    }

    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    int rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    int rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;

    int pointCount() const {
        return static_cast<int>(type) + (type < conicType);
    }

    void subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dest, float* weight) const;
    OpVector tangent(float t) const;

    // rotates curve in a space where line's (pt[0], pt[1]) moves to ((0, 0), (0, line[1].y - line[0].y))
    // curve scale is not preserved
    void toVertical(const std::array<OpPoint, 2> line, OpCurve& rotated) const;

#if OP_DEBUG_DUMP
    std::string debugDump() const;
    std::string debugDumpHex() const;
    void dump() const { return OpDebugOut(debugDumpHex()); }
#endif
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif

    OpPoint pts[4];
    float weight;
    OpType type;
#if OP_DEBUG
    OpDebugIntersect debugIntersect;
#endif
};

struct OpLine : OpCurve {
    OpLine() {
        type = lineType;
    }

    OpLine(const OpPoint p[])
        : OpCurve(p, lineType) {
    }

    OpLine(OpPoint p0, OpPoint p1) 
        : OpCurve(p0, p1) {
    }

    OpLine(const OpCurve& c)
        : OpCurve(c.pts[0], c.pts[c.pointCount() - 1]) {
    }

    int axisRawHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
//    int axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
    float interp(XyChoice offset, float t) const;
    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    int rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    int rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    OpVector tangent(float t) const;
};

struct OpQuadCoefficients {
    float a;
    float b;
    float c;
};

struct OpQuad : OpCurve {
    OpQuad() {
        type = quadType;
    }

    OpQuad(const OpPoint p[])
        : OpCurve(p, quadType) {
    }

    int axisRawHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
 //   int axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
    OpQuadCoefficients coefficients(Axis offset) const;
    int extrema(XyChoice offset, rootCellar& t) const;
    bool monotonic(XyChoice offset) const;
    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    int rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    int rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    void subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dest) const;
    OpVector tangent(float t) const;
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif
};

struct OpConic : OpCurve {
    OpConic() {
        type = conicType;
    }

    OpConic(const OpPoint p[], float w)
        : OpCurve(p, w) {
    }

    int axisRawHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
//    int axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
    OpQuadCoefficients coefficients(Axis offset, float intercept) const;
    float denominator(float t) const;
    OpQuadCoefficients derivative_coefficients(XyChoice offset) const;
    int extrema(XyChoice offset, rootCellar& t) const;
    bool monotonic(XyChoice offset) const;
    OpVector normal(float t) const;
    OpPoint numerator(float t) const;
    OpPoint ptAtT(float t) const;
    int rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    int rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    void subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dest, float* weight) const;
    float tangent(XyChoice offset, float t) const;
    OpVector tangent(float t) const;
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif
};

struct OpCubicCoefficients {
    OpCubicFloatType a;
    OpCubicFloatType b;
    OpCubicFloatType c;
    OpCubicFloatType d;
};

struct OpCubic : OpCurve {
    OpCubic() {
        type = cubicType;
    }

    OpCubic(const OpPoint p[])
        : OpCurve(p, cubicType) {
    }

    int axisRawHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
//    int axisRayHit(Axis offset, float axisIntercept, rootCellar& cepts) const;
//    int axisRayHit(Axis offset, float axisIntercept, OpPtT start, OpPtT end,
//            rootCellar& cepts) const;
    OpCubicCoefficients coefficients(Axis offset) const;
    OpPoint doublePtAtT(float t) const;
    int extrema(XyChoice offset, rootCellar& t) const;
    int inflections(rootCellar& t) const;
    OpPoint interp(float t) const;
    bool monotonic(XyChoice offset) const;
    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    int rawIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    int rayIntersect(const std::array<OpPoint, 2> line, rootCellar& cepts) const;
    void subDivide(OpPtT ptT1, OpPtT ptT2, std::array<OpPoint, 4>& dest) const;
    float tangent(XyChoice offset, double t) const;
    OpVector tangent(float t) const;
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif
};

#endif
