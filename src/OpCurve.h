// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurve_DEFINED
#define OpCurve_DEFINED

#include "PathOpsTypes.h"

#define RAW_INTERSECT_LIMIT 0.00005f  // errors this large or larger mean the crossing was not found

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
        : c{ nullptr, 0, (PathOpsV0Lib::CurveType) 0 }
        , contours(nullptr)
        , isLineSet(false)
        , isLineResult(false) {
    }

    OpCurve(OpContours* , PathOpsV0Lib::Curve );
    OpRoots axisRayHit(Axis offset, float axisIntercept, float start = 0, float end = 1) const;
    OpRoots axisRawHit(Axis offset, float axisIntercept, MatchEnds ) const;
    float center(Axis offset, float axisIntercept) const;
    OpPtT cut(const OpPtT& ptT, float loBounds, float hiBounds, float direction) const;
    CutRangeT cutRange(const OpPtT& ptT, float loEnd, float hiEnd) const;
    OpPoint doublePtAtT(float t) const;
    OpPoint end(float t) const;
    OpPtT findIntersect(Axis offset, const OpPtT& ) const;
    OpPoint firstPt() const  {
        return c.data->start; } 
    OpPoint hullPt(int index) const;
    bool isFinite() const;
    bool isLine(); 
    bool isVertical() const;
    OpPoint lastPt() const {
        return c.data->end; }
    LinePts linePts() const {
        LinePts result { firstPt(), lastPt() }; return result; }
    OpRootPts lineIntersect(const LinePts& line) const;
    // Returns t of point on curve if any; returns NaN if no match. Used by line/curve intersection.
    float match(float start, float end, OpPoint ) const;
    MatchReverse matchEnds(const LinePts& ) const;
    bool nearBounds(OpPoint ) const;
    OpVector normal(float t) const;
    NormalDirection normalDirection(Axis axis, float t) const;
    bool normalize();
    void output(bool firstPt, bool lastPt);
    void pinCtrl();
    OpPoint ptAtT(float t) const;
    OpPtT ptTAtT(float t) const {
        return { ptAtT(t), t }; }
    OpPointBounds ptBounds() const;
    int pointCount() const;
    OpRoots rawIntersect(const LinePts& line, MatchEnds ) const;  // requires sect to be on curve
    OpRoots rayIntersect(const LinePts& line, MatchEnds ) const;
    void reverse();
    void setFirstPt(OpPoint pt) {
        c.data->start = pt; }
    void setLastPt(OpPoint pt) {
        c.data->end = pt; }
    OpCurve subDivide(OpPtT ptT1, OpPtT ptT2) const;
    OpVector tangent(float t) const;
    float tAtXY(float t1, float t2, XyChoice , float goal) const;
    // rotates curve so that line's (pt[0], pt[1]) moves to ((0, 0), (0, line[1].y - line[0].y))
    // curve scale is not preserved
    OpCurve toVertical(const LinePts& line, MatchEnds ) const;
    float tZeroX(float t1, float t2) const;  // binary search on t-range finds vert crossing zero
    OpPair xyAtT(OpPair t, XyChoice xy) const;
#if OP_DEBUG
    bool debugIsLine() const;
#endif
    DUMP_DECLARATIONS

    // create storage in contour; helper function casts it to CurveData
    PathOpsV0Lib::Curve c;
    OpContours* contours;  // required by new interface for caller function pointer access
    bool isLineSet;
    bool isLineResult;
};

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

    void construct(const LinePts& pts);
	LinePts pts;
	Axis axis;
	float value;
	bool useAxis;
};
#endif

#endif
