// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurve_DEFINED
#define OpCurve_DEFINED

#include "OpMath.h"

struct OpLine;
struct OpQuad;
struct OpConic;
struct OpCubic;

// int value is (mostly) point count - 1 (conic is exception)
enum class OpType {
    no,
    line,
    quad,
    conic,
    cubic,
};

// arranged so down/left is -1, up/right is +1
enum class NormalDirection {
	downLeft = -1,
	underflow,
	upRight,
	overflow,
};

struct LinePts {
    bool isPoint() const;

    std::array<OpPoint, 2> pts;
};

struct OpCurve {
    OpCurve() 
        : weight(1)
        , type(OpType::no) {
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
        , type(OpType::line) {
        pts[0] = p0;
        pts[1] = p1;
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(const OpPoint p[], float w)
        : weight(w)
        , type(OpType::conic) {
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

    OpRoots axisRayHit(Axis offset, float axisIntercept, float start = 0, float end = 1) const;
    float center(Axis offset, float axisIntercept) const;
    OpPtT findIntersect(Axis offset, const OpPtT& ) const;
    bool isFinite() const;
    bool isLinear() const;

    OpPoint lastPt() const {
        return pts[pointCount() - 1];
    }

    OpVector normal(float t) const;
    NormalDirection normalDirection(Axis axis, float t) const;
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& line) const;
    OpRoots rayIntersect(const LinePts& line) const;

    int pointCount() const {
        return static_cast<int>(type) + (type < OpType::conic);
    }

    const OpCurve& set(OpPoint start, const OpPoint ctrlPts[2], OpPoint end, unsigned ptCount, 
            OpType opType, float w);
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;

    // rotates curve in a space where line's (pt[0], pt[1]) moves to ((0, 0), (0, line[1].y - line[0].y))
    // curve scale is not preserved
    OpCurve toVertical(const LinePts& line) const;

#if OP_DEBUG_DUMP
    std::string debugDump() const;
    std::string debugDumpHex() const;
    void dump() const;
    void dumpDetail() const;
    void dumpHex() const;
#endif
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif

    OpPoint pts[4];
    float weight;
    OpType type;
#if OP_DEBUG
    OpDebugIntersect debugIntersect;
    OpPoint debugOriginalCtrls[2];   // if cubic ctrl pts are pinned, store originals here
#endif
};

struct OpLine : OpCurve {
    OpLine() {
        type = OpType::line;
    }

    OpLine(const OpPoint p[])
        : OpCurve(p, OpType::line) {
    }

    OpLine(OpPoint p0, OpPoint p1) 
        : OpCurve(p0, p1) {
    }

    OpLine(const OpCurve& c)
        : OpCurve(c.pts[0], c.pts[c.pointCount() - 1]) {
    }

    OpRoots axisRawHit(Axis offset, float axisIntercept) const;
//    OpRoots axisRayHit(Axis offset, float axisIntercept) const;
    float interp(XyChoice offset, float t) const;
    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& line) const;
    OpVector tangent() const;
};

struct OpQuadCoefficients {
    float a;
    float b;
    float c;
};

struct OpQuad : OpCurve {
    OpQuad() {
        type = OpType::quad;
    }

    OpQuad(const OpPoint p[])
        : OpCurve(p, OpType::quad) {
    }

    OpRoots axisRawHit(Axis offset, float axisIntercept) const;
 //   OpRoots axisRayHit(Axis offset, float axisIntercept) const;
    OpQuadCoefficients coefficients(Axis offset) const;
    OpRoots extrema(XyChoice offset) const;
    bool monotonic(XyChoice offset) const;
    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& line) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif
};

struct OpConic : OpCurve {
    OpConic() {
        type = OpType::conic;
    }

    OpConic(const OpPoint p[], float w)
        : OpCurve(p, w) {
    }

    OpRoots axisRawHit(Axis offset, float axisIntercept) const;
//    OpRoots axisRayHit(Axis offset, float axisIntercept) const;
    OpQuadCoefficients coefficients(Axis offset, float intercept) const;
    float denominator(float t) const;
    OpQuadCoefficients derivative_coefficients(XyChoice offset) const;
    OpRoots extrema(XyChoice offset) const;
    bool monotonic(XyChoice offset) const;
    OpVector normal(float t) const;
    OpPoint numerator(float t) const;
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& line) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
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
        type = OpType::cubic;
    }

    OpCubic(const OpPoint p[])
        : OpCurve(p, OpType::cubic) {
    }

    OpRoots axisRawHit(Axis offset, float axisIntercept) const;
//    OpRoots axisRayHit(Axis offset, float axisIntercept) const;
//    OpRoots axisRayHit(Axis offset, float axisIntercept, OpPtT start, OpPtT end) const;
    OpCubicCoefficients coefficients(Axis offset) const;
    OpPoint doublePtAtT(float t) const;
    OpRoots extrema(XyChoice ) const;
    OpRoots inflections() const;
    OpPoint interp(float t) const;
    bool monotonic(XyChoice ) const;
    OpVector normal(float t) const;
    void pinCtrls(XyChoice );
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& line) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    float tangent(XyChoice , double t) const;
    OpVector tangent(float t) const;
#if OP_DEBUG
    OpVector debugTangent(float t) const;
#endif
};

#if OP_DEBUG_IMAGE  
// here because OpPoint is not declared in OpDebugImage.h or OpDebugDouble.h
struct OpDebugRay {
	OpDebugRay(Axis a, float v)
		: axis(a)
		, value(v)
		, useAxis(true) {
	}
	OpDebugRay(const LinePts& );
	LinePts pts;
	Axis axis;
	float value;
	bool useAxis;
};

#endif

#endif
