// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpIntersection_DEFINED
#define OpIntersection_DEFINED

#include "OpTightBounds.h"

struct OpPtAliases;
struct OpEdge;
struct OpSegment;

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

	DUMP_DECLARATIONS

	OpIntersection* start;
	OpIntersection* end;
	OpIntersection* oStart;
	OpIntersection* oEnd;
	OpEdge* edge;
	OpEdge* oppEdge;
	int id;
	OP_DEBUG_CODE(OpEdge* lastEdge);
};

#define CoinOpp_Enums \
	OP_ENUM_MEMBER(no), \
	OP_ENUM_MEMBER(yes)  /* set for the one of coin pair that will be disabled */

enum class CoinOpp {
	CoinOpp_Enums
};

inline CoinOpp operator!(CoinOpp m) {
	OP_ASSERT(CoinOpp::no == m || CoinOpp::yes == m);
	return static_cast<CoinOpp>(!static_cast<int>(m));
}

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
	void pair(OpIntersection* o) {
		OP_ASSERT(abs(unsectID) == abs(o->unsectID)); 
		OP_ASSERT(coincidenceID == o->coincidenceID); 
		OP_ASSERT(ptT.pt == o->ptT.pt || (!!unsectID && !!o->unsectID) || !opp);
		opp = o;
		o->opp = this;
	}

	void set(const OpPtT& t, OpSegment* seg  OP_LINE_FILE_DEF(int srcID, int oppID)) {
		segment = seg;
		OP_DEBUG_CODE(debugSetID());  // debug for now
		OP_ASSERT(OpMath::Between(0, t.t, 1));
		ptT = t;
        OP_LINE_FILE_SET(debugSetMaker);
#if OP_DEBUG
		debugSrcID = srcID;
		debugOppID = oppID;
#endif
	}

	void setCoin(int id, MatchEnds end, CoinOpp );  // setter to help debugging
	void setUnsect(int id, MatchEnds end);  // setter to help debugging

	void zeroCoincidence() {
		OP_ASSERT(!debugCoincidenceID);  // !!! should this always be zero?
		OP_DEBUG_CODE(debugCoincidenceID = coincidenceID);
		coincidenceID = 0;
		coinEnd = MatchEnds::none;
	}

	void zeroCoincidencePair() {
		zeroCoincidence();
		opp->zeroCoincidence();
	}

	void zeroUnsect() {
		unsectID = 0;
		unsectEnd = MatchEnds::none;
		ccUnsectable = false;
	}

	void zeroUnsectPair() {
		zeroUnsect();
		opp->zeroUnsect();
	}

#if OP_DEBUG
	void debugSetID();
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
	void debugCoinValidate() const;
#endif
#if OP_DEBUG_DUMP
	void debugCompare(std::string) const;
	#define OP_X(Thing) \
	std::string debugDump##Thing() const; \
	void dump##Thing() const;
	DEBUG_DUMP
	#undef OP_X
#include "OpDebugDeclarations.h"
#endif

	OpSegment* segment  OP_DEBUG_CODE(=nullptr);
	OpIntersection* opp = nullptr;
	OpPtT ptT;
	int coincidenceID = 0;  // if non-zero, intersection marks range where edges completely overlap
	int unsectID = 0;  // if non-zero, intersection marks range where edges are too close to call
	// !!! why does coin makes both negative but unsect only makes one negative...
	MatchEnds coinEnd = MatchEnds::none;  // puts start before end on sort (neg. if pair flipped)
	MatchEnds unsectEnd = MatchEnds::none;  // one side is negative if pair are flipped
	CoinOpp coinOpp = CoinOpp::no;  // set if coincident segment or edge will be disabled
	bool betweenCoins = false;  // used to find unsortable edges between coincident pairs
	bool ccSect = false;  // set if curve-curve created coins/unsectables (if possibly out-of-order)
	bool ccUnsectable = false;  // set if curve-curve created or set unsectables (to treat as coin)
	bool collapsed = false;  // set if coincidence or unsect pair collapsed to a point
	bool mergeProcessed = false;
	bool moved = false;
#if OP_DEBUG
	int id = 0;
	int debugSrcID = 0;	// pair of edges or segments that intersected (!!! only useful if edges?)
	int debugOppID = 0;
	int debugCoincidenceID = 0;	// this one does not get erased
	bool debugErased = false;
#endif
    OP_LINE_FILE_DECLARE(debugSetMaker)
};

enum class SectCleanup {
	none,
	sectsRemoved,
	segmentCollapsed,
};

struct OpIntersections {
	OpIntersection* add(OpIntersection* );
	OpIntersection* coinContains(OpPoint pt, const OpSegment* opp) const;
	OpIntersection* coinContains(OpPoint pt, const OpSegment* opp, OpPtT* nearby) const;
	void coinRange(OpEdge& , OpSegment* opp, bool reversed);
	OpIntersection* contains(const OpPtT& ptT, const OpSegment* opp);  // nearby ptT
//	OpIntersection* const * entry(const OpPtT& , const OpSegment* opp) const;  // exact opp + ptT
	std::vector<int> findPals(float t) const;
	void makeEdges(OpSegment* );
	void markInCoincidence();
	float matchT(const OpPtT& , OpPoint destination, MatchEnds ) const;
	void mergeNear(OpPtAliases& );
	SectCleanup moveSects(const OpPtT& match, OpPoint destination, MatchEnds );
	void orderPairs();
//	const OpIntersection* nearly(const OpPtT& ptT, OpSegment* oSeg) const;  // near match of pt or t
//	void range(const OpSegment* , std::vector<OpIntersection*>& );
	bool simpleEnd() const;  // true if array has only one entry with t equal to one
	bool simpleStart() const;  // true if array has only one entry with t equal to zero
	void sort();  // 
// return intersections that delineate unsectable runs that contain this edge
	std::vector<OpIntersection*> unsectables(OpPoint );
	static bool UnsectablesOverlap(std::vector<OpIntersection*> set1,
			std::vector<OpIntersection*> set2);
//	void windCoincidences(std::vector<OpEdge>& edges);
	void zeroPairs(OpIntersection* );
#if OP_DEBUG
	OpIntersection* debugAlreadyContains(const OpPoint& , const OpSegment* opp) const;
	bool debugContains(const OpPtT& , const OpSegment* opp) const;  // check for duplicates
#endif
#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
	DUMP_DECLARATIONS

	// all intersections are stored here before edges are rewritten
	std::vector<OpIntersection*> i;
	bool unsorted = false;
	bool hasCCSects = false;
	bool hasPairs = false;
};

// allocating storage separately allows intersections to be immobile and have reliable pointers
struct OpSectStorage {
	OpSectStorage()
		: next(nullptr)
		, used(0) {
	}
#if OP_DEBUG_DUMP
	int debugCount() const;
	OpIntersection* debugFind(int id) const;
	OpIntersection* debugIndex(int index) const;
	static void DumpSet(const char*& , OpContext* );
	DUMP_DECLARATIONS
#endif

	OpSectStorage* next;
	OpIntersection storage[256];
	int used;
};

#endif
