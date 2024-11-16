// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurveCurve_DEFINED
#define OpCurveCurve_DEFINED

#include "OpContour.h"
#include "OpCurve.h"
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

struct CcCurves;

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

// distance from edge to opp at this edge t, and number of edges between this and next
struct EdgeRun {
	void set(OpEdge* , const OpSegment* opp, EdgeMatch );
	bool inDeleted(CcCurves* , CcCurves* oppCurves) const;
	float setOppDist(const OpSegment* segment);
	DUMP_DECLARATIONS

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
	bool deletedT(float t) const;
	static OpPtT Dist(const OpSegment* , const OpPtT& segPtT, const OpSegment* opp);
	std::vector<CutRangeT> findGaps() const;
	int groupCount() const;
	void initialEdgeRun(OpEdge* edge, const OpSegment* oppSeg);
	void markToDelete(float tStart, float tEnd);
	int overlaps() const;
	float perimeter() const;
	void snipAndGo(const OpSegment* , const OpPtT& cut, OpPoint oppPt, const OpSegment* oppSeg);
	// void snipOne(const OpSegment* , const OpPtT& lo, const OpPtT& hi);
	void snipRange(const OpSegment* , const OpPtT& lo, const OpPtT& hi, const OpSegment* oppSeg);
	DUMP_DECLARATIONS

	std::vector<OpEdge*> c;
	std::vector<EdgeRun> runs;
	std::vector<CutRangeT> deleted;
	CcCurves* oppCurves;
};

struct FoundLimits {
//	void setEnd(const OpSegment* opp, const OpCurve& curve, float t);
	DUMP_DECLARATIONS

	const OpEdge* parentEdge;
	const OpEdge* parentOpp;  // may be null
	OpPtT seg;
	OpPtT opp;
	bool fromFoundT;  // if set, don't add segment intersections
	bool oppOutOfOrder;  // if set, opp t is not ordered (skip this limit)  !!! detect error earlier
#if OP_DEBUG_MAKER
	OpDebugMaker maker;
#endif
};

#endif

struct OpCurveCurve {
	OpCurveCurve(OpSegment* seg, OpSegment* opp);
	void addIntersection(OpEdge* edge, OpEdge* opp);
	void addEdgeRun(OpEdge* , CurveRef , EdgeMatch );
	bool addUnsectable(const OpPtT& edgeStart, const OpPtT& edgeEnd,
			const OpPtT& oppStart, const OpPtT& oppEnd);
	bool alreadyInLimits(const OpEdge* edge, const OpEdge* oEdge, float t);
	bool betweenLimits(const OpEdge* edge, const OpEdge* oEdge, float lo, float hi);
	bool checkForGaps();
	bool checkSect();
	bool checkSplit(float lo, float hi, CurveRef , OpPtT& checkPtT) const;
	void checkUnsplitables();
	SectFound divideAndConquer();
	bool endsOverlap() const;
	void findUnsectable();
	bool ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	bool ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	void recordSect(OpEdge* edge, OpEdge* opp, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_ARGS());
	bool reduceDistFlipped();  // replace edges with dist runs that change sign
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
	bool debugShowImage(bool atDepth = false);
#endif
#if OP_DEBUG_DUMP
	OpCurveCurve(OpContours* c) { contours = c; }
	void drawClosest(const OpPoint& originalPt) const;
	void dumpClosest(const OpPoint& pt) const;
#include "OpDebugDeclarations.h"
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
	float maxSignSwap;
	int depth;
	int uniqueLimits_impl;  // cached count; set negative if invalid (call 
	int unsplitables;
	int maxSplits;
	int maxDepth;
	bool addedPoint;
	bool rotateFailed;
	bool sectResult;
	bool smallTFound;  // if true, hull sort should prefer large t values
	bool largeTFound;  // also used to resolve t gaps 
	bool lastDepthReduced;
	bool foundGap;
	bool splitMid;
	bool splitHullFail;  // set true if mid t is nearly equal to an end 
#if OP_DEBUG_DUMP
	static int debugCall;
	int debugLocalCall;  // (copy so it is visible in debugger)
#endif
#if OP_DEBUG_VERBOSE
	std::vector<size_t> dvDepthIndex;
	std::vector<OpEdge*> dvAll;
#endif
};

