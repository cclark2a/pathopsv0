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
	fail,
	intersects,
	overflow,
	split,
};

enum class CurveRef {
	edge,
	opp
};

struct CcSect {
	CcSect() 
		: edge(nullptr)
		OP_DEBUG_PARAMS(debugSetMaker("", 0))
		OP_DEBUG_PARAMS(debugReason(SectReason::unset)) {
	}
	CcSect(OpEdge* e, OpPtT ePtT  OP_LINE_FILE_DEF(SectReason sectReason)) 
		: edge(e)
		, ptT(ePtT)
		OP_DEBUG_PARAMS(debugSetMaker(fileName, lineNo))
		OP_DEBUG_PARAMS(debugReason(sectReason)) {
	}
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	OpEdge* edge;
	OpPtT ptT;
#if OP_DEBUG
	OpDebugMaker debugSetMaker;
	SectReason debugReason;	// reason intersection was found
#endif
};

struct CcSects {
	CcSects() {}
	CcSects(OpEdge* edge, OpPtT ePtT, OpEdge* opp, OpPtT oPtT
				OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason)) 
		: e(edge, ePtT  OP_LINE_FILE_CALLER(eReason))
		, o(opp, oPtT  OP_LINE_FILE_CALLER(oReason)) {
	}
#if OP_DEBUG_DUMP
	std::string debugDump(DebugLevel l, DebugBase b, int indent) const;
	DUMP_DECLARATIONS
#endif

	CcSect e;
	CcSect o;
};

struct CcClose {
	CcClose()
		: bestDist(OpInfinity) {
	}
	void save(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT);
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	CcSects lo;
	CcSects best;
	CcSects hi;
	float bestDist;
};

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
	std::vector<TGap> findGaps() const;
	void markToDelete(float tStart, float tEnd);
	bool overlaps() const;
	void snipAndGo(const OpSegment* ,  const OpPtT& cut);
	void snipOne(const OpSegment* ,  const OpPtT& lo, const OpPtT& hi);
	void snipRange(const OpSegment* , const OpPtT& lo, const OpPtT& hi);

	std::vector<OpEdge*> c;
};

struct FoundPtTs {
	OpPtT seg;
	OpPtT opp;

	void setEnd(const OpSegment* opp, const OpCurve& curve, float t);

#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif
};

struct FoundLimits {
	FoundPtTs lo;
	FoundPtTs hi;

#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif
};

#endif

struct OpCurveCurve {
	static constexpr int ccMaxSplits = 6;   // !!! no idea what this should be 
	static constexpr int maxSplits = 8;   // !!! no idea what this should be 
	static constexpr int maxDepth = 24;  // !!! no idea what this should be

	OpCurveCurve(OpSegment* seg, OpSegment* opp);
	void addIntersection(OpEdge& edge, OpEdge& opp);
	SectFound addSect();
	void addUnsectable(OpSegment* seg, const OpPtT& edgeStart, const OpPtT& edgeEnd,
		OpSegment* opp, const OpPtT& oppStart, const OpPtT& oppEnd);
	bool checkForGaps();
	bool checkSect();
	bool checkSplit(float lo, float hi, CurveRef , OpPtT& checkPtT) const;
	void closest();
	static OpPtT Cut(const OpPtT& , const OpSegment* , float direction);
	static CutRangeT CutRange(const OpPtT& , const OpSegment* , 
			float loEnd, float hiEnd);
	SectFound divideAndConquer();
	bool endsOverlap() const;
	SectFound findUnsectable();
	void ifCloseSave(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	bool ifExactly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	bool ifNearly(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT);
	static bool LineMissed(OpEdge& edge, OpEdge& opp);
	FoundPtTs nearbyRun(OpSegment* seg, OpSegment* opp, const OpPtT& basePtT, float endSegT);
	void recordSect(OpEdge& edge, const OpPtT& edgePtT, OpEdge& opp, const OpPtT& oppPtT
			OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason));
	bool rotatedIntersect(OpEdge& edge, OpEdge& opp);
	bool setHullSects(OpEdge& edge, OpEdge& opp, CurveRef );
	bool setOverlaps();
	void splitDownTheMiddle(OpContours* contours, OpEdge& edge, const OpPtT& edgeMid, 
			CcCurves* splits);
	bool splitHulls(CurveRef );  // hull finds split point; returns true if snipped
	void tryClose();
#if OP_DEBUG
	~OpCurveCurve() { 
			originalEdge->contours()->debugCurveCurve = nullptr; }
#endif
#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
	void drawClosest(const OpPoint& originalPt) const;
	void dumpClosest(const OpPoint& pt) const;
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
#if OP_DEBUG_VERBOSE
	void debugSaveState();
	void dumpDepth(int level);
	void dumpDepth();
#endif

	const OpEdge* originalEdge;
	const OpEdge* originalOpp;
	CcCurves edgeCurves;
	CcCurves oppCurves;
	std::vector<CcSects> ccSects;
	CcClose closeEdge;
	OpPtT snipEdge;
	OpPtT snipOpp;
	MatchReverse matchRev;
	int depth;
	bool rotateFailed;
	bool sectResult;
	bool smallTFound;  // if true, hull sort should prefer large t values
	bool largeTFound;  // also used to resolve t gaps 
	bool foundGap;
	bool splitMid;
	bool tryCloseBy;
#if OP_DEBUG_DUMP
	static int debugCcCall;
	int debugLocalCcCall;  // (copy so it is visible in debugger)
#endif
#if OP_DEBUG_VERBOSE
	std::vector<int> dvDepthIndex;
	std::vector<OpEdge*> dvAll;
#endif
};

