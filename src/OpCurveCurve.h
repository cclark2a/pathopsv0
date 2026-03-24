// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpCurveCurve_DEFINED
#define OpCurveCurve_DEFINED

#include "OpContext.h"
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
struct OpCurveCurve;

enum class EdgeOverlaps {
    no,
    overlaps,
};

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

enum class LimitFrom : int8_t {
    OP_DEBUG_ENUM()
    no = false,
    yes = true
};

// distance from edge to opp at this edge t, and number of edges between this and next
struct EdgeRun {
	bool inDeleted(CcCurves* , CcCurves* oppCurves) const;  // true if edgePt t in edge curves, etc.
	void set(OpEdge* edge, OpSegment* opp, EdgeMatch , float distFactor  OP_LINE_FILE_ARGS());
	bool set(OpPtT& s, OpPtT& o, OpSegment* seg, float distFactor  OP_LINE_FILE_ARGS());
	void setOppDist(const OpSegment* segment, float distFactor);
	DUMP_DECLARATIONS

	OpPtT edgePtT;	// should be sorted by t in cc curves' runs (e.g., cc curves check mid)
	OpPtT oppPtT;
	float oppDist  OP_DEBUG_INIT_FLOAT();
	LimitFrom fromFoundT  OP_DEBUG_INIT(LimitFrom);
	bool byZero  OP_DEBUG_INIT_BOOL();
	// !!! may need to add line limit (see found limit) -- wait for test case
#if 0 && OP_DEBUG
	int debugBetween = INT_MAX;  // incremented if edge t is between, and oppDist is between
#endif
	OP_LINE_FILE_DECLARE(debugSetMaker)
};

// distance from edge to opposite is left unclamped for hull intersections to detect crossings
enum class ClampDist : int8_t {
    OP_DEBUG_ENUM()
    no = false,
    yes = true
};

struct Interval {
    OpPtT lo;
    OpPtT hi;
};

enum class DiffIntersect {
	intersect,
	ignore,
	replace
};

struct LoHi {
	float lo;
	float hi;
	DiffIntersect diffSect;
};

struct CutRangeT {
	DUMP_DECLARATIONS

	OpPtT lo;
	OpPtT hi;
};

struct CcCurves {
	EdgeRun* addEdgeRun(OpEdge* edge, EdgeMatch , ClampDist  OP_LINE_FILE_ARGS());
	EdgeRun* addEdgeRun(EdgeRun& , EdgeMatch , ClampDist);
    void baseInit(OpCurveCurve* , CcCurves* );
    bool checkMid(float midT, float startDist, float endDist);
    bool checkMidEdge(OpEdge* );
	bool checkMidRun(size_t index);  // mid pt distance to next run smaller
	void checkSigns();
	void clear();
    void complementRun(OpEdge* opp);
    std::vector<Interval> continuous(const OpPtT& lower, const OpPtT& upper) const;  // true if edges link lower to upper
	bool deletedT(float t) const;  // true if t is in deleted -- deleted need not be sorted
	std::vector<CutRangeT> findGaps() const;
    void init(OpCurveCurve* , CcCurves* oppCurves, float scaledMax, OpEdge* parent, OpSegment* opp);
	void initialEdgeRun(OpEdge* edge);
    int insertPos(std::vector<EdgeRun>& , EdgeRun& );
	bool keepDiff(const LoHi& ) const;
	bool lopSided(size_t priorCount, float maxBias) const;
	void markToDelete(float tStart, float tEnd);
	int overlaps() const;
	float perimeter() const;
    void shareDistance();
	void snipAndGo(const CutRangeT& );
    OpEdge* twoHulls(OpPtT& lower, OpPtT& upper);
	DUMP_DECLARATIONS
    OP_DEBUG_CODE(void debugAdd(EdgeRun& ));
    OP_DEBUG_CODE(void debugAdd(CcCurves& ));
    OP_DEBUG_CODE(void debugCheck(const OpEdge* , EdgeMatch) const);
	OP_DEBUG_VALIDATE_CODE(void debugValidate() const);  // assert if not sorted

    OpCurveCurve* cc;
	std::vector<OpEdge*> c;  // !!! is this sorted?
	std::vector<EdgeRun> runs;  // sorted culled runs, keeping extreme distances
	std::vector<CutRangeT> deleted;  // pairs of t ranges
    OpSegment* seg  OP_DEBUG_INIT_PTR(OpSegment);
    OpSegment* opp  OP_DEBUG_INIT_PTR(OpSegment);
	CcCurves* oppCurves  OP_DEBUG_INIT_PTR(CcCurves);
    float scaledMax  OP_DEBUG_INIT_FLOAT();

#if OP_DEBUG || OP_DEBUGGER
    std::vector<EdgeRun> debugRuns;  // raw data
#endif
};

enum class Unordered : int8_t {
    OP_DEBUG_ENUM()
    no = false,
    yes = true
};

enum class LimitUsed : int8_t {
    OP_DEBUG_ENUM()
    no = false,
    yes = true
};

enum class LimitMatch : int8_t {
    OP_DEBUG_ENUM()
    no = false,
    yes = true
};

enum class LimitSwapped : int8_t {
	OP_DEBUG_ENUM()
    no = false,
    yes = true
};

enum class LimitBettered : int8_t {
	OP_DEBUG_ENUM()
    no = false,
    yes = true
};

enum class LimitLine : int8_t {
	OP_DEBUG_ENUM()
    no = false,
    yes = true
};

struct FoundLimit {
#if OP_DEBUG_DUMP
	FoundLimit() {}
#endif
	FoundLimit(OpEdge* edge, OpEdge* oEdge, const OpPtT& edgePtT, const OpPtT& oppPtT
			OP_LINE_FILE_ARGS());
	DUMP_DECLARATIONS

	const OpEdge* parentEdge  OP_DEBUG_INIT_PTR(OpEdge);
	const OpEdge* parentOpp  OP_DEBUG_INIT_PTR(OpEdge);  // may be null
	OpPtT segPtT;
	OpPtT oppPtT;
	LimitFrom fromFoundT  OP_DEBUG_INIT(LimitFrom);  // if set, don't add segment intersections
	Unordered oppOutOfOrder  OP_DEBUG_INIT(Unordered);  // if set, opp t not ordered (skip this)  !!! detect error earlier
    LimitUsed used  OP_DEBUG_INIT(LimitUsed);
    LimitMatch match  OP_DEBUG_INIT(LimitMatch);
	LimitSwapped swapped  OP_DEBUG_INIT(LimitSwapped);
	LimitBettered bettered  OP_DEBUG_INIT(LimitBettered);
	LimitLine edgeLine  OP_DEBUG_INIT(LimitLine);  // curve degenerated to a line
	LimitLine oppLine  OP_DEBUG_INIT(LimitLine); 
	OP_LINE_FILE_DECLARE(debugMaker)
};

struct SnipPtTs {
	DUMP_DECLARATIONS

    OpPtT segPtT;
    OpPtT oppPtT;
	CutRangeT segCut;
	CutRangeT oppCut;
};

struct FoundLimits {
	FoundLimits(OpCurveCurve* c_c) {
		cc = c_c; }
	void addSnip(const OpPtT& s, const OpPtT& o);
	void addSnipCommon(SnipPtTs& snipLo, SnipPtTs& snipHi);
	bool addSnipRange(size_t oldCount);
	bool alreadyIn(const OpPtT& edgePtT, const OpPtT& oppPtT) const;
	void cutPair(SnipPtTs& lo, SnipPtTs& hi) const;
	bool empty() const { return i.empty(); }
	void markOutOfOrder();
	void setEnds(std::vector<OpIntersection*>& matchingSects);
	void setEdge(const OpEdge* );
	void setOpp(const OpEdge* );
	void setUnique();
	size_t size() const { return i.size(); }
	void sort() {
		std::sort(i.begin(), i.end(), [](const FoundLimit& a, const FoundLimit& b) {
			return a.segPtT.t < b.segPtT.t; }); }
	DUMP_DECLARATIONS

	std::vector<FoundLimit> i;
	std::vector<SnipPtTs> snips;
	OpCurveCurve* cc;
	int unique = -1;  // cached count; set negative if invalid
	bool smSegT = false;
    bool lgSegT = false;
    bool smOppT = false;
    bool lgOppT = false;
};

#if OP_DEBUG_VERBOSE
struct DebugDepth {
	DUMP_DECLARATIONS

    size_t all;
    int depth;
};

struct DebugRunSize {
	DUMP_DECLARATIONS

    size_t edgeRuns;
    size_t oppRuns;
};
#endif

#if OP_DEBUG
enum class CcBreak {
	atEnd,
	atDepth
};
#endif

enum class ForCurveLineSect {
	dummy
};

struct OpCurveCurve {
#if OP_DEBUG_DUMP
	OpCurveCurve() 
		: limits(this) {}
#endif
	OpCurveCurve(OpSegment* seg, OpSegment* opp, std::vector<OpIntersection*>& matchingSects, 
			ForCurveLineSect );
	OpCurveCurve(OpSegment* seg, OpSegment* opp, std::vector<OpIntersection*>& matchingSects);
	void addIntersection(OpEdge* edge, OpEdge* opp);
    bool addLineCurveIntersection(OpEdge& edge, OpEdge& opp, CurveRef );
	EdgeRun* addEdgeRun(OpEdge* , CurveRef , EdgeMatch  OP_LINE_FILE_ARGS());
	bool addUnsectable(FoundLimit& limit, FoundLimit& limitEnd);
    OpEdge* allocateEdge(OpSegment* , const OpEdge* , const OpPtT& start, const OpPtT& end,
            NewEdge, EdgeOverlaps  OP_LINE_FILE_DEF(int parentID));
	bool alreadyInLimits(const OpEdge* edge, const OpEdge* oEdge, 
			const OpPtT& edgePtT, const OpPtT& oppPtT);
	bool betweenLimits(OpSegment* , float lo, float hi);
	OpEdge* boundedEdge(OpSegment* s, const OpPointBounds&, OpPtT* singleton  OP_LINE_FILE_ARGS());
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
	void sectPair(OpIntersection* sect, OpIntersection* oSect, OpPoint limitPt);
	bool setHullSects(OpEdge& edge, OpEdge& opp, CurveRef );
	bool setHulls(CurveRef curveRef);
	bool setOverlaps();
//	bool setSnipFromLimits(size_t oldCount);
    bool smallTFound(CurveRef );
	bool splitDownTheMiddle(const OpEdge& edge, CurveRef , CcCurves& splits);
	bool splitHulls(CurveRef , CcCurves& splits);  // hull finds split point
	size_t uniqueLimits();
#if OP_DEBUG
	bool debugBreak(CcBreak );
#endif
#if OP_DEBUG_DUMP
	OpCurveCurve(OpContext* c) 
		: limits(this) { 
		context = c; }
	void drawClosest(const OpPoint& originalPt) const;
	void dumpClosest(const OpPoint& pt) const;
#endif
#if OP_DEBUG_SERIALIZE
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_VERBOSE
	void debugSaveState();
	std::string debugDumpDepth(int level);
	void dumpDepth(int level);
	void dumpDepth();
#endif

	OpContext* context  OP_DEBUG_INIT_PTR(OpContext);
	OpSegment* seg  OP_DEBUG_INIT_PTR(OpSegment);
	OpSegment* opp  OP_DEBUG_INIT_PTR(OpSegment);
	OpEdge* parentEdge  OP_DEBUG_INIT_PTR(OpEdge);
	OpEdge* parentOpp  OP_DEBUG_INIT_PTR(OpEdge);
	CcCurves edgeCurves;
	CcCurves oppCurves;
	FoundLimits limits;
    OpVector maxSplit;  // limit of subdivision when reducing via distance flip
	OpVector maxBoundedEdge; // threshold factor comparing edge line/line or line/curve intersection
	OpVector maxUnsectable;  // threshold factor to move sect pairs to common points
//	MatchReverse matchRev;
    size_t endMatches  OP_DEBUG_INIT_SIZE();
	float maxAngleMatch  OP_DEBUG_INIT_FLOAT();  // found limit tangent angle required to detect dups
	float maxAngleSweep  OP_DEBUG_INIT_FLOAT();  // found limit tangent angle required to detect dups
	float maxSignSwap  OP_DEBUG_INIT_FLOAT();
	float maxSplitBias  OP_DEBUG_INIT_FLOAT();  // if bias does no meaningful reduction, split down the middle instead
    float maxDist  OP_DEBUG_INIT_FLOAT();  // threshold factor comparing edge run distances between seg and opp
	float maxEdgeTSlop  OP_DEBUG_INIT_FLOAT();  // slop allowed for edge t range to intersect opposite for line/curve or line/line
    int depth  OP_DEBUG_INIT_INT();
	int unsplitables  OP_DEBUG_INIT_INT();
	int maxCheckSplit  OP_DEBUG_INIT_INT();  // iteration count to check if point is inside deleted bounds
	int maxDeep  OP_DEBUG_INIT_INT();  // curves, when divided, always overlap, recurse further to look for sects
	int maxShallow  OP_DEBUG_INIT_INT();  // curves, when divided, with no overlap, recurse less to look for sects
	int maxSplits  OP_DEBUG_INIT_INT();  // if active splits of either curve exceed this level, give up
//	int maxBoundedT;
    bool reversed  OP_DEBUG_INIT_BOOL();
	bool boundedEdgeFailed  OP_DEBUG_INIT_BOOL();
	bool overlap  OP_DEBUG_INIT_BOOL();
	bool rotateFailed  OP_DEBUG_INIT_BOOL();
	bool sectResult  OP_DEBUG_INIT_BOOL();
	bool lastDepthReduced  OP_DEBUG_INIT_BOOL();
	bool foundGap  OP_DEBUG_INIT_BOOL();
	bool splitMid  OP_DEBUG_INIT_BOOL();
	bool splitHullFail  OP_DEBUG_INIT_BOOL();  // set true if mid t is nearly equal to an end 
#if OP_DEBUG_DUMP
	bool debugDumpOn = false;
#endif
};

#if OP_DEBUG_DUMP
struct DumpCurveCurve {
	DUMP_DECLARATIONS

	std::vector<OpCurveCurve> cc;
	std::vector<DebugRunSize> runIndex;
	std::vector<EdgeRun> runs;  // make copy because some originals are temporary
	int nthCall = 0;  // used to break on the nth call (not serialized)
};

extern DumpCurveCurve dumpCurveCurve;
#endif

#endif
