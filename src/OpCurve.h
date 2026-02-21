// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurve_DEFINED
#define OpCurve_DEFINED

#include "PathOpsTypes.h"
#if OP_DEBUG
#include "DebugOpsTypes.h"
#endif

struct OpContext;
struct OpContour;

#if OP_TEST_RASTER
struct OpEdge;
#endif
enum class EdgeMatch : int8_t;

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
	DUMP_DECLARATIONS

	OpPtT lo;
	OpPtT hi;
};

// init; during initialization, threshold is not yet set (so degenerate lines cannot be found)
#define Rotated_Enums \
	OP_ENUM_MEMBER(no), \
	OP_ENUM_MEMBER(yes), \
	OP_ENUM_MEMBER(init), \
	OP_ENUM_MEMBER(debug)

enum class Rotated : int8_t {
	Rotated_Enums
};

struct OpCurve {
	OpCurve() 
		: c{ nullptr, nullptr, 0, PathOpsV0Lib::degenerateLine }
		, rotated(Rotated::no)
		, isLineSet(false)
		, isLineResult(false)
		, isSmall(false)
        , reversed(false) {
	}

	OpCurve(PathOpsV0Lib::Curve , Rotated );
#if OP_DEBUG_IMAGE
	OpCurve(PathOpsV0Lib::AddCurve , Rotated );
#endif
	// void adjust(OpPoint start, OpPoint end);
	OpRect aliasBounds() const {
		return OpRect(start, end); }
	OpRoots axisRayHit(Axis offset, float axisIntercept, float start = 0, float end = 1) const;
	OpRoots axisRawHit(Axis offset, float axisIntercept, MatchEnds) const;
	PathOpsV0Lib::WindKeep bestLoop(PathOpsV0Lib::Winding , 
			bool firstPt, bool lastPt  OP_DEBUG_PARAMS(int parentID));
	OpRect callerBounds() const {
		return OpRect(c.data->start, c.data->end); }
	float center(Axis offset, float axisIntercept) const;
	OpRect closeBounds() const;
    OpContext& context() {
        return *(OpContext*) c.context; }
    const OpContext& context() const {
        return *(const OpContext*) c.context; }
//	OpPtT cut(const OpPtT& ptT, float loBounds, float hiBounds, float direction) const;
//	CutRangeT cutRange(const OpPtT& ptT, OpPoint oppPt, float loEnd, float hiEnd) const;
//	OpPoint end(float t) const;
    float findValidT(float start, float end, OpPoint opp);
//	OpPtT findIntersect(Axis offset, const OpPtT& ) const;
	OpPoint firstPt() const  {
		return start; } 
	float height() const {
		return fabsf(start.y - end.y); }
	OpPoint hullPt(int index) const;
	float interceptLimit() const;
	bool isFinite() const;
	bool isLine(); 
	bool isVertical() const;
	OpPoint lastPt() const {
		return end; }
	float left() const {
		return std::min(start.x, end.x); }
	LinePts linePts() const {
		LinePts result { firstPt(), lastPt() }; return result; }
	OpPtT lineCurve(OpCurve& line, float t, float* lineT, MatchEnds , float margin);
    OpRoots lineIntersection(OpCurve& );
	OpRoots lineIntersect(const LinePts& line) const;
	PathOpsV0Lib::CurveType lineType() const;
	// Returns t of point on curve if any; returns NaN if no match. Used by line/curve intersection.
	float match(float start, float end, OpPoint ) const;
	MatchReverse matchEnds(const LinePts& ) const;
	bool nearBounds(OpPoint ) const;
	OpVector normal(float t) const;
	NormalDirection normalDirection(Axis axis, float t) const;
	float normalLimit() const;
	bool normalize();
	PathOpsV0Lib::WindKeep output(PathOpsV0Lib::Winding , bool firstPt, bool lastPt  
			OP_DEBUG_RASTER_PARAMS(OpEdge* ));
	void pinCtrl();
	OpPoint ptAtT(float t) const;
	OpPoint ptDAtT(float t) const;
	OpPtT ptTAtT(float t) const {
		return { ptAtT(t), t }; }
	OpPointBounds fullBounds() const;  // if curve is rotated, may need to consider control points
	int pointCount() const;
	OpRoots rawIntersect(const LinePts& line, MatchEnds ) const;  // requires sect to be on curve
	OpRoots rayIntersect(const LinePts& line, MatchEnds ) const;
	void reverse();
	void setAliases(OpContour& );
	void setFirstPt(OpPoint pt) {
		start = c.data->start = pt; }
	void setLastPt(OpPoint pt) {
		end = c.data->end = pt; }
	void setLine() {
		isLineSet = false; isLine(); }
	void setLineType() {
		c.type = lineType(); }
	float top() const {
		return std::min(start.y, end.y); }
	OpPoint whichPt(EdgeMatch ) const;
	float width() const {
		return fabsf(start.x - end.x); }

	OpCurve subDivide(float t1, float t2) const;
	OpVector tangent(float t) const;
	float tAtXY(float t1, float t2, XyChoice , float goal) const;
	// rotates curve so that line's (pt[0], pt[1]) moves to ((0, 0), (0, line[1].y - line[0].y))
	// curve scale is not preserved
	OpCurve toVertical(const LinePts& line, MatchEnds ) const;
	OpCurve toVerticalBase(const LinePts& line, MatchEnds ) const;
//	float tZeroX(float t1, float t2) const;  // binary search on t-range finds vert crossing zero
    OpContext& writableContext() const { 
        return *(OpContext*) c.context; }
	OpPair xyAtT(OpPair t, XyChoice xy) const;
	void zeroSmall(OpContour& );
#if OP_DEBUG
	bool debugIsLine() const;
#endif
#if OP_TEST_RASTER
	void debugScale(double scaleX, double scaleY, double offsetX, double offsetY);
#endif
#if OP_DEBUGGER
	OpCurve debugSubDivide(float t1, float t2) const;
#endif
	DUMP_DECLARATIONS

	// create storage in contour; helper function casts it to CurveData
	PathOpsV0Lib::Curve c;
	OpPoint start;	// either original, or alias
	OpPoint end;
	Rotated rotated;
	bool isLineSet;
	bool isLineResult;
	bool isSmall;
    bool reversed;
	OP_DEBUG_CODE(bool debugZeroedSmall = false);
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
#if OP_DEBUG_SERIALIZE
	std::string debugDump(DebugLevel l, DebugBase b) const;
	std::string debugDump(PathOpsV0Lib::CurveData* ) const;
#endif
#if OP_DEBUG_DUMP
	PathOpsV0Lib::CurveData* dumpSet(const char*& str);
	static void DumpSet(const char*& str, CurveDataStorage** previousPtr);
#endif

	CurveDataStorage* next;
	size_t used;
	uint8_t storage[sizeof(OpPoint) * 256];
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
