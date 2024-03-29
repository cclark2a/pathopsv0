// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurve_DEFINED
#define OpCurve_DEFINED

#include "OpMath.h"

#define RAW_INTERSECT_LIMIT 0.00005f  // errors this large or larger mean the crossing was not found

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
    cubic
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
#if OP_DEBUG_DUMP
    DUMP_DECLARATIONS
#endif

    std::array<OpPoint, 2> pts;
};

// used to pass pairs of values where SIMD allows computing two at once
struct OpPair {
    friend OpPair operator+(OpPair a, OpPair b) {
        return { a.s + b.s, a.l + b.l };
    }

    friend OpPair operator-(float a, OpPair b) {
        return { a - b.s, a - b.l };
    }

    friend OpPair operator*(OpPair a, OpPair b) {
        return { a.s * b.s, a.l * b.l };
    }

    friend OpPair operator*(float a, OpPair b) {
        return { a * b.s, a * b.l };
    }

    friend OpPair operator*(OpPair a, float b) {
        return { a.s * b, a.l * b };
    }

    float s;  // smaller
    float l;  // larger
};

#define USE_SEGMENT_CENTER 1

struct OpCurve {
    OpCurve() 
        : weight(1)
        , type(OpType::no)
        , centerPt(false) {
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(const OpPoint p[], OpType t) 
        : weight(1)
        , type(t)
        , centerPt(false) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(const OpPoint p[], float w, OpType t)
        : weight(w)
        , type(t)
        , centerPt(false) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
        OP_DEBUG_CODE(debugIntersect = OpDebugIntersect::segment);
    }

    OpCurve(OpPoint p0, OpPoint p1)
        : weight(1)
        , type(OpType::line)
        , centerPt(false) {
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
    int closest(OpPtT* best, float delta, OpPoint pt) const;
    OpPoint doublePtAtT(float t) const;
    OpPtT findIntersect(Axis offset, const OpPtT& ) const;
    bool isFinite() const;
    bool isLinear() const;
    OpPoint lastPt() const {
        return pts[pointCount() - 1]; }
    OpRootPts lineIntersect(const LinePts& line) const;
    OpVector normal(float t) const;
    NormalDirection normalDirection(Axis axis, float t) const;
    bool output(OpOutPath& path, bool firstPt, bool lastPt);  // provided by graphics implementation
    OpPoint ptAtT(float t) const;
    int pointCount() const {
        return static_cast<int>(type) + (type < OpType::conic); }
    OpRoots rawIntersect(const LinePts& line, MatchEnds ) const;  // requires sect to be on curve
    OpRoots rayIntersect(const LinePts& line, MatchEnds ) const;
    void reverse();
    const OpCurve& set(OpPoint start, OpPoint end, unsigned ptCount, OpType opType, float w);
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;
    float tAtXY(float t1, float t2, XyChoice , float goal) const;
    // rotates curve in a space where line's (pt[0], pt[1]) moves to ((0, 0), (0, line[1].y - line[0].y))
    // curve scale is not preserved
    OpCurve toVertical(const LinePts& line) const;
    float tZeroX(float t1, float t2) const;
    OpPair xyAtT(OpPair t, XyChoice xy) const;
#if OP_DEBUG_DUMP
    DUMP_DECLARATIONS
#endif

    OpPoint pts[5];  // extra point carries cubic center for vertical rotation (used by curve sect)
    float weight;
    OpType type;
    bool centerPt;  // true if center point follows curve
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
    OpRoots axisTanHit(Axis offset, float axisIntercept) const;
    float interp(XyChoice offset, float t) const;
    OpVector normal(float t) const;
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& ) const;
    OpVector tangent() const;
    OpRoots tangentIntersect(const LinePts& line) const;  // treats both lines as rays
    OpPair xyAtT(OpPair t, XyChoice ) const;
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
    OpRoots rawIntersect(const LinePts& ) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;
    OpPair xyAtT(OpPair t, XyChoice ) const;
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
    OpRoots rawIntersect(const LinePts& ) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    float tangent(XyChoice offset, float t) const;
    OpVector tangent(float t) const;
    float tAtXY(float t1, float t2, XyChoice , float xy) const;
    OpPair xyAtT(OpPair t, XyChoice ) const;
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

    OpRoots axisRawHit(Axis offset, float axisIntercept, MatchEnds ) const;
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
    OpRoots rawIntersect(const LinePts& , MatchEnds ) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    float tangent(XyChoice , double t) const;
    OpVector tangent(float t) const;
    OpPair xyAtT(OpPair t, XyChoice ) const;
};

#if OP_DEBUG_IMAGE  
// here because OpPoint is not declared in OpDebugImage.h or OpDebugDouble.h
struct OpDebugRay {
	OpDebugRay(Axis a, float v)
		: axis(a)
		, value(v)
		, useAxis(true) {
	}
	OpDebugRay(const LinePts& pts) {
        construct(pts);
    }
	OpDebugRay(const OpLine& line) {
        LinePts p { line.pts[0], line.pts[1] };
        construct(p);
    }
    void construct(const LinePts& pts);
	LinePts pts;
	Axis axis;
	float value;
	bool useAxis;
};

#endif

#endif
