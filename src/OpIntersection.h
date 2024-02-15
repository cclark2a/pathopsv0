// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpIntersection_DEFINED
#define OpIntersection_DEFINED

#include "OpTightBounds.h"

struct OpEdge;
struct OpSegment;

#if OP_DEBUG
// !!! this seems absurdly long
enum class IntersectMaker {
	addCoincidentCheck,
	addCoincidentCheckOpp,
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
	edgeCCExact,
	edgeCCHull,
	edgeCCHullPair,
	edgeCCNearly,
	edgeCurveCurve,
	edgeLineCurve,
	edgeLineCurveOpp,
	edgeT,
	edgeTOpp,
	oppT,
	oppTOpp,
	findIntersections_start,
	findIntersections_startOppReversed,
	findIntersections_startOpp,
	findIntersections_end,
	findIntersections_endOppReversed,
	findIntersections_endOpp,
	missingCoincidence,
	missingCoincidenceOpp,
	oppCCExact,
	oppCCHull,
	oppCCHullPair,
	oppCCNearly,
	oppCurveCurve,
	segEnd,
	segmentLineCurve,
	segmentLineCurveOpp,
	segStart,
	splitAtWinding,
	unsectableStart,
	unsectableEnd,
	unsectableOppStart,
	unsectableOppEnd,
	// testing only
	opTestEdgeZero1,
	opTestEdgeZero2,
	opTestEdgeZero3,
	opTestEdgeZero4,
};

#define SECT_MAKER(maker) IntersectMaker::maker, __LINE__, std::string(__FILE__)

#endif

struct CoinPair {
    CoinPair(OpIntersection* s, OpIntersection* os, OpEdge* e, OpEdge* o, int ID  
			OP_DEBUG_PARAMS(OpEdge* l))
        : start(s)
        , end(nullptr)
        , oStart(os)
        , oEnd(nullptr)
        , edge(e)
        , oppEdge(o)
        , id(ID)
        OP_DEBUG_PARAMS(lastEdge(l)) {
    }

#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	OpIntersection* start;
    OpIntersection* end;
    OpIntersection* oStart;
    OpIntersection* oEnd;
    OpEdge* edge;
    OpEdge* oppEdge;
    int id;
    OP_DEBUG_CODE(OpEdge* lastEdge);
};

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
	OpIntersection() {
#if OP_DEBUG
	segment = nullptr;
	opp = nullptr;
	coincidenceID = 0;
	unsectID = 0;
	betweenID = 0;
	coinEnd = MatchEnds::none;
	unsectEnd = MatchEnds::none;
	coincidenceProcessed = false;
	id = 0;
	debugID = 0;
	debugOppID = 0;
	debugCoincidenceID = 0;
	debugMaker = IntersectMaker::opTestEdgeZero4;
	debugSetMaker = { "", 0 };
	debugReason = SectReason::test;	// reason intersection was found
	debugErased = false;
#endif
    }

	void betweenPair(OpIntersection* );

	void pair(OpIntersection* o) {
		opp = o;
		o->opp = this;
	}

	void set(const OpPtT& t, OpSegment* seg
			OP_DEBUG_PARAMS(IntersectMaker maker, int line, std::string file, SectReason reason, 
			int ID, int oppID)) {
		segment = seg;
		OP_DEBUG_CODE(debugSetID());  // debug for now
		opp = nullptr; // SectFlavor::none == flavor_ ? nullptr : this;		!!! if we need this, comment why
		OP_ASSERT(OpMath::Between(0, t.t, 1));
		ptT = t;
		betweenID = 0;
		coincidenceID = 0;  // 0 if no coincidence; negative if coincident pairs are reversed
		unsectID = 0;  // 0 if not unsectable; negative if curves are reversed
		coinEnd = MatchEnds::none;
		unsectEnd = MatchEnds::none;
		coincidenceProcessed = false;
#if OP_DEBUG
		debugID = ID;
		debugOppID = oppID;
		debugCoincidenceID = 0;
		debugMaker = maker;
		debugSetMaker = { file, line };
		debugReason = reason;
		debugErased = false;
#endif
	}

	void zeroCoincidenceID() {
		OP_ASSERT(coincidenceID);
		OP_ASSERT(opp->coincidenceID);
#if OP_DEBUG
		debugCoincidenceID = coincidenceID;
		opp->debugCoincidenceID = opp->coincidenceID;
#endif
		coincidenceID = 0;
		opp->coincidenceID = 0;
	}

#if OP_DEBUG
	void debugSetID();
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	OpIntersection(std::string);
	void debugCompare(std::string) const;
	std::string debugDump(bool fromDumpFull, bool fromDumpDetail) const;
	std::string debugDumpDetail(bool fromDumpIntersections) const;
	std::string debugDumpBrief() const;
#include "OpDebugDeclarations.h"
#endif

	OpSegment* segment;
	OpIntersection* opp;
	OpPtT ptT;
	int coincidenceID;
	int unsectID;	// !!! may be able to be merged with coincident ID; keep separate for now
	int betweenID;  // is on unsectable with this id, between ends; use to mark edge unsortable
	MatchEnds coinEnd;  // used to put start before end on sect sort
	MatchEnds unsectEnd;
	bool coincidenceProcessed;
#if OP_DEBUG
	int id;
	int debugID;	// pair of edges or segments that intersected
	int debugOppID;
	int debugCoincidenceID;	// this one does not get erased
	IntersectMaker debugMaker;	// where intersection was made
	OpDebugMaker debugSetMaker;
	SectReason debugReason;	// reason intersection was found
	mutable bool debugErased;
#endif
};

struct OpIntersections {
	OpIntersections();
	OpIntersection* add(OpIntersection* );
    OpIntersection* contains(const OpPtT& ptT, const OpSegment* opp) const {
		OpIntersection* const * result = entry(ptT, opp); return result ? *result : nullptr; }
    OpIntersection* const * entry(const OpPtT& , const OpSegment* opp) const;  // match opp + ptT
	void makeEdges(OpSegment* );
	const OpIntersection* nearly(const OpPtT& ptT, OpSegment* oSeg) const;  // near match of pt or t
    std::vector<OpIntersection*> range(const OpSegment* );
    void sort();
	void windCoincidences(std::vector<OpEdge>& edges  OP_DEBUG_PARAMS(OpVector tangent));
#if OP_DEBUG
    OpIntersection* debugAlreadyContains(const OpPoint& , const OpSegment* opp) const;
    bool debugContains(const OpPtT& , const OpSegment* opp) const;  // check for duplicates
#endif
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	// all intersections are stored here before edges are rewritten
    std::vector<OpIntersection*> i;
    bool resort;
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
