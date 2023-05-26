#ifndef OpIntersection_DEFINED
#define OpIntersection_DEFINED

#include "OpMath.h"

struct OpSegment;

enum class SelfIntersect {
	none,
	self,
	missing,
	split,
};

#if OP_DEBUG
enum class IntersectMaker {
	addCoincidentCheck1,
	addCoincidentCheck2,
	addCurveCoincidence1,
	addCurveCoincidence2,
	addCurveCoincidence3,
	addCurveCoincidence4,
	addIntersection1,
	addIntersection2,
	addIntersection3,
	addIntersection4,
	addIntersection5,
	addIntersection6,
	addIntersection7,
	addIntersection8,
	addMatchingEnds1,
	addMatchingEnds2,
	addMatchingEnds3,
	addMatchingEnds4,
	addMix1,
	addMix2,
	addPair1,
	addPair2,
	addPair3,
	addPair4,
	coincidentCheck1,
	coincidentCheck2,
	curveCenter1,
	curveCenter2,
	findIntersections1,
	findIntersections2,
	findIntersections3,
	findIntersections4,
	findIntersections5,
	findIntersections6,
	findIntersections7,
	findIntersections8,
	makeEdges,
	missingCoincidence1,
	missingCoincidence2,
	opCubicErrorTest1,
	opCubicErrorTest2,
	opTestEdgeZero1,
	opTestEdgeZero2,
	opTestEdgeZero3,
	opTestEdgeZero4,
	splitAtWinding,
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
			OP_DEBUG_PARAMS(IntersectMaker maker)) {
		segment = seg;
		opp = SelfIntersect::none == self_ ? nullptr : this;
		ptT = t;
		coincidenceID = cID;	// 0 if no coincidence; negative if coincident pairs are reversed
		self = self_;
		aliased = false;
//		unsortable = false;
#if OP_DEBUG
		debugMaker = maker;
		debugCoincidenceID = 0;
		debugAliasID = 0;
		debugSetID();		// debug for now
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
	std::string debugDump(bool fromDumpFull) const;
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
//	bool unsortable;  // !!! check the validity of whether this needs to be computed or not
#if OP_DEBUG
	IntersectMaker debugMaker;
	int id;
	int debugCoincidenceID;	// this one does not get erased
	int debugAliasID;
	OpPoint debugOriginal;	// point value prior to aliasing
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
