// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurveCurve_DEFINED
#define OpCurveCurve_DEFINED

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

struct CutRangeT {
	OpPtT lo;
	OpPtT hi;
};

struct OpCurveCurve {
	static constexpr int maxSplits = 8;   // !!! no idea what this should be 
	static constexpr int maxDepth = 24;  // !!! no idea what this should be

	OpCurveCurve(OpEdge* edge, OpEdge* opp);
	SectFound addUnsectable(); // if curve doesn't devolve into line segments
	void closest();
	SectFound curvesIntersect(CurveRef );
	SectFound divideAndConquer();
#if CC_EXPERIMENT
	SectFound divideExperiment();
#endif
	void findEdgesTRanges(CurveRef );
	void ifCloseSave(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT);
	void linearIntersect(std::vector<OpEdge*>& lines, std::vector<OpEdge*>& linesOrCurves);
	void release();
	bool split(CurveRef , DoSplit );
#if CC_EXPERIMENT
	void addUnsectable2(OpSegment* seg, const OpPtT& edgeStart, const OpPtT& edgeEnd,
		OpSegment* opp, OpPtT oppStart, OpPtT oppEnd);
	void checkForGaps();
	bool checkSplit(float lo, float hi, CurveRef , OpPtT& checkPtT) const;
	static OpPtT Cut(const OpPtT& , const OpSegment* , float direction);
	static CutRangeT CutRange(const OpPtT& , const OpSegment* , 
			float loEnd, float hiEnd);
	void findUnsectable();
	static void SetHullSects(OpEdge& edge, OpEdge& opp);
	void recordSect(OpEdge& edge, OpPtT edgePtT, OpEdge& opp, OpPtT oppPtT
			OP_LINE_FILE_DEF(SectReason eReason, SectReason oReason));
	void snipOne(std::vector<OpEdge*>& curves, const OpSegment* , OpContours* ,
			const OpPtT& lo, const OpPtT& hi);
	void snipAndGo(std::vector<OpEdge*>& curves, const OpSegment* , OpContours* , const OpPtT& cut);
	void splitDownTheMiddle(OpContours* contours, OpEdge& edge, const OpPtT& edgeMid, 
			std::vector<OpEdge*>* splits);
	void splitSect(std::vector<OpEdge*>& curves);  // split and discard edge near intersection
	bool splitHulls(CurveRef );  // hull finds split point; returns true if snipped
#endif
	bool tooFew(CurveRef );

#if OP_DEBUG
	void debugDone(OpContours* c) { 
			c->debugCurveCurve = nullptr; }
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
	void dumpDepth(int level);
	void dumpDepth();
#endif

	const OpEdge* originalEdge;
	const OpEdge* originalOpp;
	std::vector<OpEdge*> edgeCurves;
	std::vector<OpEdge*> edgeLines;
	std::vector<OpEdge*> oppCurves;
	std::vector<OpEdge*> oppLines;
	std::vector<OpEdge*> edgeRuns;
	std::vector<OpEdge*> oppRuns;
#if CC_EXPERIMENT
	std::vector<CcSects> ccSects;
	CcClose closeEdge;
	OpPtT snipEdge;
	OpPtT snipOpp;
#endif
	int depth;
	bool sectResult;
	bool smallTFound;  // if true, hull sort should prefer large t values
	bool largeTFound;  // also used to resolve t gaps 
#if OP_DEBUG_DUMP
	static int debugExperiment;
	int debugLocal;  // (copy so it is visible in debugger)
#endif
#if OP_DEBUG_VERBOSE
	std::vector<int> dvDepthIndex;
	std::vector<OpEdge*> dvAll;
#endif
};

#endif
