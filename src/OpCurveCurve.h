// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurveCurve_DEFINED
#define OpCurveCurve_DEFINED

#include "OpContour.h"
#include "OpEdge.h"
#include "OpIntersection.h"

/*
	- reject segments only by bounds intersection (bounds may be rotated)
	- use line/line intersection only (no line/curve intersection)
	- accelerate by guessing better centers for curve splitting
	- split based on curviness, overall bounds

	to be coded/tested
	- can transferring t from line intersection to curve work well enough?
	- how is partial/complete coincidence detected
	- can successive centers suggest future guesses or detect intersection condition?

	further concerns (e.g. thread_circles380007)
	- a curve/curve may be missed if the ends of one pair segments meet at a zero axis, and a 
	  third segment crosses near that axis. The third segment's edges may move away from each
	  of the original pair when subdivided. Keep track of the closest points found when subdividing
	  (store them in the segment) to see if this case can be detected.
*/

enum class SectFound {
	no,
	add,
	fail,
	intersects,
	overflow,
	split,
};

enum class CurveRef {
	edge,
	opp
};

inline CurveRef operator!(CurveRef a) {
    return static_cast<CurveRef>(!static_cast<int>(a));
}

struct TGap {
	TGap(const OpPtT& l, const OpPtT& h)
		: lo(l)
		, hi(h) {
	}

	OpPtT lo;
	OpPtT hi;
};

struct CcCurves {
	void clear();
	OpPtT closest(OpPoint pt) const;
	static OpPtT Dist(const OpSegment* , const OpPtT& segPtT, const OpSegment* opp);
	void endDist(const OpSegment* seg, const OpSegment* opp);
	std::vector<TGap> findGaps() const;
	int groupCount() const;
	void markToDelete(float tStart, float tEnd);
	int overlaps() const;
	float perimeter() const;
	void snipAndGo(const OpSegment* ,  const OpPtT& cut);
	// void snipOne(const OpSegment* ,  const OpPtT& lo, const OpPtT& hi);
	void snipRange(const OpSegment* , const OpPtT& lo, const OpPtT& hi);
	OpPtT splitPt(float oMidDist, const OpEdge& edge) const;
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	std::vector<OpEdge*> c;
};

struct FoundLimits {
//	void setEnd(const OpSegment* opp, const OpCurve& curve, float t);
#if OP_DEBUG_DUMP
	std::string debugDump(DebugLevel l, DebugBase b, int indent) const;
	DUMP_DECLARATIONS
#endif

	const OpEdge* parentEdge;
	const OpEdge* parentOpp;  // may be null
	OpPtT seg;
	OpPtT opp;
	bool fromFoundT;  // if set, don't add segment intersections
#if OP_DEBUG
	OpDebugMaker maker;
	SectReason eReason;
	SectReason oReason;
#endif
};

#endif

struct OpCurveCurve {
	static constexpr int maxDepth = 64;  // !!! no idea what this should be

	OpCurveCurve(OpSegment* seg, OpSegment* opp);
	void addIntersection(OpEdge* edge, OpEdge* opp);
	SectFound addSect();
	void addUnsectable(const OpPtT& edgeStart, const OpPtT& edgeEnd,
			const OpPtT& oppStart, const OpPtT& oppEnd);
	bool checkDist(int edgeOverlaps, int oppOverlaps);
	bool checkForGaps();
	bool checkLimits(size_t oldCount);
	bool checkSect();
	bool checkSplit(float lo, float hi, CurveRef , OpPtT& checkPtT) const;
	static OpPtT Cut(const OpPtT& , const OpSegment* , float direction);
	static CutRangeT CutRange(const OpPtT& , const OpSegment* , float loEnd, float hiEnd);
	SectFound divideAndConquer();
	bool endsOverlap() const;
	void findUnsectable();
	bool ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	bool ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	static bool LineMissed(OpEdge& edge, OpEdge& opp);
	void recordSect(OpEdge* edge, OpEdge* opp, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason));
	bool rotatedIntersect(OpEdge& edge, OpEdge& opp, bool sharesPoint);
	void setHullSects(OpEdge& edge, OpEdge& opp, CurveRef );
	void setHulls(CurveRef curveRef);
	void setIntersections();
	bool setOverlaps();
	void splitDownTheMiddle(const OpEdge& edge, const OpPtT& edgeMid, CcCurves& splits);
	void splitHulls(CurveRef , CcCurves& splits);  // hull finds split point
#if OP_DEBUG
	~OpCurveCurve() { 
		contours->debugCurveCurve = nullptr; }
#endif
#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
	void drawClosest(const OpPoint& originalPt) const;
	void dumpClosest(const OpPoint& pt) const;
#endif
#if OP_DEBUG_VERBOSE
	void debugSaveState();
	void dumpDepth(int level);
	void dumpDepth();
#endif

	OpContours* contours;
	OpSegment* seg;
	OpSegment* opp;
	CcCurves edgeCurves;
	CcCurves oppCurves;
	std::vector<FoundLimits> limits;
	OpPtT snipEdge;
	OpPtT snipOpp;
	MatchReverse matchRev;
	int depth;
	bool addedPoint;
	bool rotateFailed;
	bool sectResult;
	bool smallTFound;  // if true, hull sort should prefer large t values
	bool largeTFound;  // also used to resolve t gaps 
	bool foundGap;
	bool splitMid;
#if OP_DEBUG_DUMP
	static int debugCall;
	int debugLocalCall;  // (copy so it is visible in debugger)
#endif
#if OP_DEBUG_VERBOSE
	std::vector<int> dvDepthIndex;
	std::vector<OpEdge*> dvAll;
#endif
};

