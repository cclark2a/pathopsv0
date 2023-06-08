#ifndef OpIntersection_DEFINED
#define OpIntersection_DEFINED

#include "OpMath.h"

struct OpEdge;
struct OpSegment;

enum class SelfIntersect {
	none,
	self,
	missing,
	split,
};

#if OP_DEBUG
// !!! this seems absurdly long
enum class IntersectMaker {
	addCoincidentCheck,
	addCoincidentCheckOpp,
	addCurveCoinStart,
	addCurveCoinEnd,
	addCurveCoinOppStart,
	addCurveCoinOppEnd,
	addMatchingEnd,
	addMatchingEndOpp,
	addMatchingEndOStart,
	addMatchingEndOStartOpp,
	addMatchingStart,
	addMatchingStartOpp,
	addMatchingStartOEnd,
	addMatchingStartOEndOpp,
	addMix,
	addMixOpp,
	addPair_aPtT,
	addPair_bPtT,
	addPair_oppStart,
	addPair_oppEnd,
	curveCenter,
	curveCenterOpp,
	edgeIntersections,
	edgeIntersectionsOpp,
	edgeLineCurve,
	edgeLineCurveOpp,
	edgeT,
	edgeTOpp,
	oppT,
	oppTOpp,
	findExtrema,
	findIntersections_start,
	findIntersections_startOppReversed,
	findIntersections_startOpp,
	findIntersections_end,
	findIntersections_endOppReversed,
	findIntersections_endOpp,
	missingCoincidence,
	missingCoincidenceOpp,
	segmentLineCurve,
	segmentLineCurveOpp,
	splitAtWinding,
	// testing only
	opTestEdgeZero1,
	opTestEdgeZero2,
	opTestEdgeZero3,
	opTestEdgeZero4,
};

#define SECT_MAKER(maker) IntersectMaker::maker, __LINE__, std::string(__FILE__)

enum class SectReason {
	coinPtsMatch,
	curveCurveCoincidence,
	degenerateCenter,
	divideAndConquer_oneT,
	divideAndConquer_noEdgeToSplit,
	divideAndConquer_noOppToSplit,
	inflection,
	lineCurve,
	missingCoincidence,
	resolveCoin_windingChange,
	resolveCoin_oWindingChange,
	sharedEdgePoint,
	sharedEndPoint,
	xExtrema,
	yExtrema,
	// testing only
	test,
};
#endif

// Places where a pair of segments cross are recorded as intersections.
// Pairs of intersections, along with segments' ends, extremas, and inflections,
// are used to create edges. Edges may then be subdivided so that each edge has
// a unique winding contribution, it is necessary to record those 
// additional subdivisions as intersections, so the intersection array can be used
// to find adjacent edges later. But it isn't necessary
// to record extremas or inflections in the intersection array; it is done as
// a matter of convenience when initially constructing the edge list.

// The intersection struct, once allocated, is never relocated. This allows pairs of
// intersection to point at each other at time of creation.

struct OpIntersection {
	OpIntersection() {}

	void pair(OpIntersection* opp_) {
		opp = opp_;
		opp_->opp = this;
	}

	void set(const OpPtT& t, OpSegment* seg, SelfIntersect self_ , int cID
			OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
			const OpIntersection* parent, const OpEdge* edge, const OpEdge* oEdge, 
			const OpSegment* dSeg, const OpSegment* dOpp)) {
		segment = seg;
		opp = SelfIntersect::none == self_ ? nullptr : this;
		ptT = t;
		coincidenceID = cID;	// 0 if no coincidence; negative if coincident pairs are reversed
		self = self_;
		aliased = false;
//		unsortable = false;
#if OP_DEBUG
		debugSetID();		// debug for now
		debugParent = parent;
		debugEdge = edge;
		debugOpp = oEdge;
		debugSeg = dSeg;
		debugSegOpp = dOpp;
		debugCoincidenceID = 0;
		debugAliasID = 0;
		debugMaker = maker;
		debugMakerLine = line;
		debugMakerFile = file;
		debugReason = reason;
		debugCollapsed = false;
		debugErased = false;
#endif
	}

	void zeroCoincidenceID() {
		assert(coincidenceID);
		assert(opp->coincidenceID);
#if OP_DEBUG
		debugCoincidenceID = coincidenceID;
		opp->debugCoincidenceID = opp->coincidenceID;
#endif
		coincidenceID = 0;
		opp->coincidenceID = 0;
	}

#if OP_DEBUG
	void debugSetID();
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	OpIntersection(std::string);
	void debugCompare(std::string) const;
	std::string debugDump(bool fromDumpFull, bool fromDumpDetail) const;
	std::string debugDumpDetail(bool fromDumpIntersections) const;
	std::string debugDumpBrief() const;
	void dumpPt() const;
	DEBUG_COMMON_DECLARATIONS();
	DUMP_COMMON_DECLARATIONS();
	DUMP_IMPL_DECLARATIONS();
#endif

	OpSegment* segment;
	OpIntersection* opp;
	OpPtT ptT;
	int coincidenceID;
	SelfIntersect self;	// inflection, or extrema -- doesn't reference another intersection
	bool aliased;     // true if point value was changed to match an intersection with the same t
#if OP_DEBUG
	int id;
	const OpIntersection* debugParent;  // intersection pt was copied from
	const OpEdge* debugEdge;	// pair of edges that intersected
	const OpEdge* debugOpp;
	const OpSegment* debugSeg;	// or: pair of segments that intersected
	const OpSegment* debugSegOpp;
	OpPoint debugOriginal;	// point value prior to aliasing
	int debugCoincidenceID;	// this one does not get erased
	int debugAliasID;
	IntersectMaker debugMaker;	// where intersection was made
	int debugMakerLine;
	std::string debugMakerFile;
	SectReason debugReason;	// reason intersection was found
	bool debugCollapsed;
	mutable bool debugErased;
#endif
};

// allocating storage separately allows intersections to be immobile and have reliable pointers
struct OpSectStorage {
	OpSectStorage()
		: next(nullptr)
		, used(0) {
	}
	OpSectStorage* next;
	OpIntersection storage[256];
	int used;
};

#endif
