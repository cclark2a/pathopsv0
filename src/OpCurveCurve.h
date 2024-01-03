// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurveCurve_DEFINED
#define OpCurveCurve_DEFINED

#define CC_EXPERIMENT 1  /* rewrites curve curve intersection to:
	- reject segments only by bounds intersection (bounds may be rotated)
	- use line/line intersection only (no line/curve intersection)
	- accelerate by guessing better centers for curve splitting
	- split based on curviness, overall bounds

	to be coded/tested
	- can transferring t from line intersection to curve work well enough?
	- how is partial/complete coincidence detected
	- can successive centers suggest future guesses or detect intersection condition?
*/

#include "OpContour.h"
#include "OpEdge.h"
#include "OpIntersection.h"

enum class SectFound {
	no,
	fail,
	intersects,
	overflow,
	split
};

enum class DoSplit {
	marked,
	all
};

enum class CurveRef {
	edge,
	opp
};

#if CC_EXPERIMENT
enum class CenterSet {
	splitNo,
	splitKeep,
	newEdge,
	defer,
	edgeCurvy,
	oppCurvy,
	edgeLineLine,
	oppLineLine,
};

enum class LineLine {
	alreadySplit,
	noIntersection,
	setCenter,
};

struct CcCenter {
	CcCenter(OpEdge* e, CurveRef w, int d, CenterSet cs)
		: edge(e)
		, which(w)
		, centerSet(cs)
		, depth(d) {
		center = edge->center;
		split = edge->doSplit;
	}
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	OpEdge* edge;
	OpPtT center;
	CurveRef which;
	EdgeSplit split;
	CenterSet centerSet;
	int depth;
};

struct CcSect {
	CcSect(OpEdge& e, OpPtT ePtT  OP_DEBUG_PARAMS(IntersectMaker eMkr)) 
		: edge(e)
		, ptT(ePtT)
		OP_DEBUG_PARAMS(maker(eMkr)) {
	}
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	OpEdge& edge;
	OpPtT ptT;
#if OP_DEBUG
	IntersectMaker maker;
#endif
};

struct CcSects {
	CcSects(OpEdge& edge, OpPtT ePtT, OpEdge& opp, OpPtT oPtT
				OP_DEBUG_PARAMS(IntersectMaker eMkr, IntersectMaker oMkr)) 
		: e(edge, ePtT  OP_DEBUG_PARAMS(eMkr))
		, o(opp, oPtT  OP_DEBUG_PARAMS(oMkr)) {
	}
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	CcSect e;
	CcSect o;
};
#endif

struct OpCurveCurve {
	static constexpr int maxSplits = 8;   // !!! no idea what this should be 
	static constexpr int maxDepth = 24;  // !!! no idea what this should be

	OpCurveCurve(OpEdge* edge, OpEdge* opp);
	SectFound addUnsectable(); // if curve doesn't devolve into line segments
	SectFound curvesIntersect(CurveRef );
	SectFound divideAndConquer();
#if CC_EXPERIMENT
	SectFound divideExperiment();
#endif
	void findEdgesTRanges(CurveRef );
	void linearIntersect(std::vector<OpEdge*>& lines, std::vector<OpEdge*>& linesOrCurves);
	void release();
	bool split(CurveRef , DoSplit );
#if CC_EXPERIMENT
	void checkSplit(float loT, float hiT, CurveRef , OpPtT& checkPtT) const;
	void snipAndGo(std::vector<OpEdge*>& curves, const OpSegment* segment, const OpPtT& cutPtT);
	void splitSect(std::vector<OpEdge*>& curves);  // split and discard edge near intersection
	void splitHulls(CurveRef , int depth);  // hull finds split point
#endif
	bool tooFew(CurveRef );

#if OP_DEBUG
	void debugDone(OpContours* c) { 
			c->debugCurveCurve = nullptr; }
#endif
#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif

	OpEdge* originalEdge;
	OpEdge* originalOpp;
	std::vector<OpEdge*> edgeCurves;
	std::vector<OpEdge*> edgeLines;
	std::vector<OpEdge*> oppCurves;
	std::vector<OpEdge*> oppLines;
	std::vector<OpEdge*> edgeRuns;
	std::vector<OpEdge*> oppRuns;
#if CC_EXPERIMENT
	std::vector<CcSects> ccSects;
	OpPtT snipEdge;
	OpPtT snipOpp;
#endif
	bool sectResult;
#if OP_DEBUG_VERBOSE
	std::vector<int> dvDepthIndex;
	std::vector<OpEdge*> dvAll;
#endif
};

#endif
