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
	OpIntersection()
		: opp(nullptr)
		, coincidenceID(0)
		, unsectID(0)
		, coinEnd(MatchEnds::none)
		, unsectEnd(MatchEnds::none)
		, mergeProcessed(false)
		, moved(false)
		, collapsed(false) 
		, ccSect(false) {
#if OP_DEBUG
	segment = nullptr;
	id = 0;
	debugSrcID = 0;
	debugOppID = 0;
	debugCoincidenceID = 0;
	debugErased = false;
#endif
	}

	// void betweenPair(OpIntersection* );
	// OpIntersection* coinOtherEnd();

	void pair(OpIntersection* o) {
		OP_ASSERT(abs(unsectID) == abs(o->unsectID)); 
		OP_ASSERT(coincidenceID == o->coincidenceID); 
		OP_ASSERT(ptT.pt == o->ptT.pt || (!!unsectID && !!o->unsectID));
		opp = o;
		o->opp = this;
	}

	void set(const OpPtT& t, OpSegment* seg  OP_LINE_FILE_DEF(int srcID, int oppID)) {
		segment = seg;
		OP_DEBUG_CODE(debugSetID());  // debug for now
		OP_ASSERT(OpMath::Between(0, t.t, 1));
		ptT = t;
#if OP_DEBUG_MAKER
		debugSetMaker = { fileName, lineNo };
#endif
#if OP_DEBUG
		debugSrcID = srcID;
		debugOppID = oppID;
#endif
	}

	void setCoin(int id, MatchEnds end);  // setter to help debugging
	void setUnsect(int id, MatchEnds end);  // setter to help debugging

	void zeroCoincidenceID() {
		OP_ASSERT(coincidenceID);
		OP_ASSERT(opp->coincidenceID);
#if OP_DEBUG
		debugCoincidenceID = coincidenceID;
		opp->debugCoincidenceID = opp->coincidenceID;
#endif
		coincidenceID = 0;
		coinEnd = MatchEnds::none;
		opp->coincidenceID = 0;
		opp->coinEnd = MatchEnds::none;
	}

	void zeroUnsectID() {
		OP_ASSERT(unsectID);
		OP_ASSERT(opp->unsectID);
		unsectID = 0;
		unsectEnd = MatchEnds::none;
		opp->unsectID = 0;
		opp->unsectEnd = MatchEnds::none;
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

	OpSegment* segment;
	OpIntersection* opp;
	OpPtT ptT;
	int coincidenceID;  // if non-zero, intersection marks range where edges completely overlap
	int unsectID;  // if non-zero, intersection marks range where edges are too close to call
	// !!! why does coin makes both negative but unsect only makes one negative...
	MatchEnds coinEnd;  // used to put start before end on sect sort (negative if pair flipped)
	MatchEnds unsectEnd;  // one side is negative if pair are flipped
	bool mergeProcessed;
	bool moved;
	bool collapsed;  // set if coincidence or unsect pair collapsed to a point
	bool ccSect;
#if OP_DEBUG
	int id;
	int debugSrcID;	// pair of edges or segments that intersected (!!! only useful if edges?)
	int debugOppID;
	int debugCoincidenceID;	// this one does not get erased
	mutable bool debugErased;
#endif
#if OP_DEBUG_MAKER
	OpDebugMaker debugSetMaker;
#endif
};

enum class MoveSects {
	zeroCoins,
	zeroAll
};

struct OpIntersections {
	OpIntersection* add(OpIntersection* );
	bool checkCollapse(OpIntersection* , MoveSects );
	OpIntersection* coinContains(OpPoint pt, const OpSegment* opp);
	OpIntersection* coinContains(OpPoint pt, const OpSegment* opp, OpPtT* nearby);
	void coinRange(OpEdge& , OpSegment* opp, bool reversed);
	OpIntersection* contains(const OpPtT& ptT, const OpSegment* opp);  // nearby ptT
//	OpIntersection* const * entry(const OpPtT& , const OpSegment* opp) const;  // exact opp + ptT
	void eraseCollapsed();
	void makeEdges(OpSegment* );
//	const OpIntersection* nearly(const OpPtT& ptT, OpSegment* oSeg) const;  // near match of pt or t
//	void range(const OpSegment* , std::vector<OpIntersection*>& );
	bool simpleEnd() const;  // true if array has only one entry with t equal to one
	bool simpleStart() const;  // true if array has only one entry with t equal to zero
	void sort();  // 
	void mergeNear(OpPtAliases& );
	void moveSects(OpPtT match, OpPoint destination, MoveSects );
// return intersections that delineate unsectable runs that contain this edge
	std::vector<OpIntersection*> unsectables(OpPoint );
	static bool UnsectablesOverlap(std::vector<OpIntersection*> set1,
			std::vector<OpIntersection*> set2);
//	void windCoincidences(std::vector<OpEdge>& edges);
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
	static void DumpSet(const char*& , OpContours* );
	DUMP_DECLARATIONS
#endif

	OpSectStorage* next;
	OpIntersection storage[256];
	int used;
};

#endif
