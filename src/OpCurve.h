// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurve_DEFINED
#define OpCurve_DEFINED

#include "PathOpsTypes.h"
#include "OpTypes.h"

#define RAW_INTERSECT_LIMIT 0.00005f  // errors this large or larger mean the crossing was not found

struct OpLine;
struct OpQuad;
struct OpConic;
struct OpCubic;

// arranged so down/left is -1, up/right is +1
enum class NormalDirection {
	downLeft = -1,
	underflow,
	upRight
};

inline NormalDirection operator!(NormalDirection a) {
    return (int) a & 1 ? (NormalDirection) - (int) a : a;
}

struct CutRangeT {
	OpPtT lo;
	OpPtT hi;
};

struct OpCurve {
    OpCurve() 
        : c{nullptr, 0, OpType::no}
        , contours(nullptr)
#if !OP_TEST_NEW_INTERFACE
        , weightImpl(1) 
#endif
    {
    }

#if !OP_TEST_NEW_INTERFACE
    OpCurve(const OpPoint p[], OpType t) 
        : c{nullptr, 0, t}
        , contours(nullptr)
        , weightImpl(1) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
    }

    OpCurve(const OpPoint p[], float w, OpType t)
        : c{nullptr, 0, t}
        , contours(nullptr)
        , weightImpl(w) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
    }

    OpCurve(OpPoint p0, OpPoint p1)
        : c{nullptr, 0, OpType::line}
        , contours(nullptr)
        , weightImpl(1) {
        pts[0] = p0;
        pts[1] = p1;
    }

    OpCurve(const OpPoint p[], float w)
        : c{nullptr, 0, OpType::conic}
        , contours(nullptr)
        , weightImpl(w) {
        memcpy(pts, p, pointCount() * sizeof(OpPoint));
    }
#endif

    OpCurve(OpContours* , PathOpsV0Lib::Curve );

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
    OpRoots axisRawHit(Axis offset, float axisIntercept, MatchEnds ) const;
    float center(Axis offset, float axisIntercept) const;
//    int closest(OpPtT* best, float delta, OpPoint pt) const;
    OpPtT cut(const OpPtT& ptT, float loBounds, float hiBounds, float direction) const;
    CutRangeT cutRange(const OpPtT& ptT, float loEnd, float hiEnd) const;
    OpPoint doublePtAtT(float t) const;
    OpPoint end(float t) const;
    OpPtT findIntersect(Axis offset, const OpPtT& ) const;
    OpPoint firstPt() const;
    OpPoint hullPt(int index) const;
    bool isFinite() const;
    bool isLine() const; 
    bool isLinear() const;
    OpPoint lastPt() const;
    OpRootPts lineIntersect(const LinePts& line) const;
    // Returns t of point on curve if any; returns NaN if no match. Used by line/curve intersection.
    float match(float start, float end, OpPoint ) const;
    bool nearBounds(OpPoint ) const;
    OpVector normal(float t) const;
    NormalDirection normalDirection(Axis axis, float t) const;
#if OP_TEST_NEW_INTERFACE
    void output(bool firstPt, bool lastPt);  // provided by graphics implementation
#else
    bool output(OpOutPath& path, bool firstPt, bool lastPt);  // provided by graphics implementation
#endif
    void pinCtrl();
    OpPoint ptAtT(float t) const;
    OpPtT ptTAtT(float t) const {
        return { ptAtT(t), t }; }
    OpPointBounds ptBounds() const;
    int pointCount() const;
    OpRoots rawIntersect(const LinePts& line, MatchEnds ) const;  // requires sect to be on curve
    OpRoots rayIntersect(const LinePts& line, MatchEnds ) const;
    void reverse();
    const OpCurve& set(OpPoint start, OpPoint end, unsigned ptCount, OpType opType, float w);
    void setFirstPt(OpPoint );
    void setLastPt(OpPoint );
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;
    float tAtXY(float t1, float t2, XyChoice , float goal) const;
    // rotates curve so that line's (pt[0], pt[1]) moves to ((0, 0), (0, line[1].y - line[0].y))
    // curve scale is not preserved
    OpCurve toVertical(const LinePts& line) const;
    float tZeroX(float t1, float t2) const;  // binary search on t-range finds vert crossing zero
    OpPair xyAtT(OpPair t, XyChoice xy) const;
#if OP_DEBUG_DUMP
#if !OP_TEST_NEW_INTERFACE
    void dumpSetPts(const char*& );
#endif
    DUMP_DECLARATIONS
#endif
    // create storage in contour; helper function casts it to CurveData
    PathOpsV0Lib::Curve c;
    OpContours* contours;  // required by new interface for caller function pointer access
#if !OP_TEST_NEW_INTERFACE
    OpPoint pts[5];  // extra point carries cubic center for vertical rotation (used by curve sect)
    float weightImpl;   // !!! new interface doesn't have this (only required for double debugging though)
#endif
};

#if !OP_TEST_NEW_INTERFACE
struct OpLine : OpCurve {
    OpLine() {
        c.type = OpType::line;
    }

    OpLine(const OpPoint p[])
        : OpCurve(p, OpType::line) {
    }

    OpLine(OpPoint p0, OpPoint p1) 
        : OpCurve(p0, p1) {
    }

    OpLine(const OpCurve& curve)
        : OpCurve(curve.pts[0], curve.pts[curve.pointCount() - 1]) {
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

struct OpQuad : OpCurve {
    OpQuad() {
        c.type = OpType::quad;
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
    void pinCtrl();
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& ) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;
    OpPair xyAtT(OpPair t, XyChoice ) const;
};

struct OpConic : OpCurve {
    OpConic() {
        c.type = OpType::conic;
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
    void pinCtrl() { asQuad().pinCtrl(); }
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& ) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    float tangent(XyChoice offset, float t) const;
    OpVector tangent(float t) const;
    float tAtXY(float t1, float t2, XyChoice , float xy) const;
    OpPair xyAtT(OpPair t, XyChoice ) const;
};

struct OpCubic : OpCurve {
    OpCubic() {
        c.type = OpType::cubic;
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
    void pinCtrls();
    OpPoint ptAtT(float t) const;
    OpRoots rawIntersect(const LinePts& , MatchEnds ) const;
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    float tangent(XyChoice , double t) const;
    OpVector tangent(float t) const;
    OpPair xyAtT(OpPair t, XyChoice ) const;
};
#endif

struct CurveDataStorage {
	CurveDataStorage()
		: next(nullptr)
		, used(0) {
        OP_DEBUG_CODE(memset(storage, 0, sizeof(storage)));
	}
    PathOpsV0Lib::CurveData* curveData(size_t size) {
        PathOpsV0Lib::CurveData* result = (PathOpsV0Lib::CurveData*) &storage[used];
        used += size;
        return result;
    }
#if OP_DEBUG_DUMP
    std::string debugDump(DebugLevel l, DebugBase b) const;
    std::string debugDump(PathOpsV0Lib::CurveData* ) const;
    PathOpsV0Lib::CurveData* dumpSet(const char*& str);
	static void DumpSet(const char*& str, CurveDataStorage** previousPtr);
#endif

	CurveDataStorage* next;
	size_t used;
	char storage[sizeof(OpPoint) * 256];
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

#if !OP_TEST_NEW_INTERFACE
	OpDebugRay(const OpLine& line) {
        LinePts p { line.pts[0], line.pts[1] };
        construct(p);
    }
#endif

    void construct(const LinePts& pts);
	LinePts pts;
	Axis axis;
	float value;
	bool useAxis;
};

#endif

#endif
