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

	- to limit recursion when edge
	- for every edge allocated:
	  compute its opp dist
	  track if opp dist is between opp dists for edge on either side
	  create a new run if new edge's opp dist is an inflection point
	  maintain runs of t values with opp dist extremes
*/

enum class SectFound {
	no,
	add,
	fail,
	intersects,
	maxOverlaps,
	noOverlapDeep,
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

// distance from edge to opp at this edge t, and number of edges between this and next
struct EdgeRun {
	void set(OpEdge* edge, const OpSegment* oppSeg, EdgeMatch );
	float setOppDist(const OpSegment* segment);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	OpEdge* edge;
	OpEdge* oppEdge;
	OpPtT edgePtT;
	OpPtT oppPtT;
	float oppDist;
	bool fromFoundT;
	bool byZero;
#if OP_DEBUG
	int debugBetween;  // incremented if edge t is between, and oppDist is between
#endif
};

struct CcCurves {
	void addEdgeRun(OpEdge* , const OpSegment* oppSeg, EdgeMatch );
	bool checkMid(size_t index); // true if mid pt dist between this and next run dist is smaller
	void clear();
	OpPtT closest(OpPoint pt) const;
	static OpPtT Dist(const OpSegment* , const OpPtT& segPtT, const OpSegment* opp);
//	void endDist(const OpSegment* seg, const OpSegment* opp);
	std::vector<TGap> findGaps() const;
	int groupCount() const;
	void initialEdgeRun(OpEdge* edge, const OpSegment* oppSeg);
	void markToDelete(float tStart, float tEnd);
	int overlaps() const;
	float perimeter() const;
	void snipAndGo(const OpSegment* ,  const OpPtT& cut, const OpSegment* oppSeg);
	// void snipOne(const OpSegment* ,  const OpPtT& lo, const OpPtT& hi);
	void snipRange(const OpSegment* , const OpPtT& lo, const OpPtT& hi, const OpSegment* oppSeg);
	OpPtT splitPt(float oMidDist, const OpEdge& edge) const;
#if OP_DEBUG
	bool debugHasEdgeRun(float t) const;
#endif
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	std::vector<OpEdge*> c;
	std::vector<EdgeRun> runs;
#if OP_DEBUG
	std::vector<EdgeRun> debugRuns;  // runs that fit inside other runs
#endif
};

struct FoundLimits {
//	void setEnd(const OpSegment* opp, const OpCurve& curve, float t);
#if OP_DEBUG_DUMP
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
	void addEdgeRun(OpEdge* , CurveRef , EdgeMatch );
	bool addUnsectable(const OpPtT& edgeStart, const OpPtT& edgeEnd,
			const OpPtT& oppStart, const OpPtT& oppEnd);
	bool alreadyInLimits(const OpEdge* edge, const OpEdge* oEdge, float t);
	bool checkDist(int edgeOverlaps, int oppOverlaps);
	bool checkForGaps();
	bool checkSect();
	bool checkSplit(float lo, float hi, CurveRef , OpPtT& checkPtT) const;
	static OpPtT Cut(const OpPtT& , const OpSegment* , float direction);
	static CutRangeT CutRange(const OpPtT& , const OpSegment* , float loEnd, float hiEnd);
	SectFound divideAndConquer();
	bool endsOverlap() const;
	void findUnsectable();
	bool ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	bool ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
//	static bool LineMissed(OpEdge& edge, OpEdge& opp);
	void recordSect(OpEdge* edge, OpEdge* opp, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason));
	bool rotatedIntersect(OpEdge& edge, OpEdge& opp, bool sharesPoint);
	SectFound runsToLimits();
	void setHullSects(OpEdge& edge, OpEdge& opp, CurveRef );
	void setHulls(CurveRef curveRef);
	void setIntersections();
	bool setOverlaps();
	bool setSnipFromLimits(size_t oldCount);
	bool splitDownTheMiddle(const OpEdge& edge, const OpPtT& edgeMid, CurveRef , CcCurves& splits);
	bool splitHulls(CurveRef , CcCurves& splits);  // hull finds split point
	size_t uniqueLimits();
#if OP_DEBUG
	~OpCurveCurve() { 
		contours->debugCurveCurve = nullptr; }
#endif
#if OP_DEBUG_DUMP
	OpCurveCurve(OpContours* c) { contours = c; }
	void drawClosest(const OpPoint& originalPt) const;
	bool dumpBreak() const;
	void dumpClosest(const OpPoint& pt) const;
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_VERBOSE
	void debugSaveState();
	void dumpDepth(int level);
	void dumpDepth();
#endif
	static constexpr int maxSplits = 8;   // !!! no idea what this should be 

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
	int uniqueLimits_impl;  // cached count; set negative if invalid (call 
	bool addedPoint;
	bool rotateFailed;
	bool sectResult;
	bool smallTFound;  // if true, hull sort should prefer large t values
	bool largeTFound;  // also used to resolve t gaps 
	bool foundGap;
	bool splitMid;
	bool splitHullFail;  // set true if mid t is nearly equal to an end 
#if OP_DEBUG_DUMP
	static int debugCall;
	int debugLocalCall;  // (copy so it is visible in debugger)
#endif
#if OP_DEBUG_VERBOSE
	std::vector<int> dvDepthIndex;
	std::vector<OpEdge*> dvAll;
#endif
};

