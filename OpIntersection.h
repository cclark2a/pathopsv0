#ifndef OpIntersection_DEFINED
#define OpIntersection_DEFINED

#include "OpMath.h"

struct OpSegment;

enum class SelfIntersect {
	none,
	self,
	missing
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
	missingCoincidence,
	opCubicErrorTest1,
	opCubicErrorTest2,
	opTestEdgeZero1,
	opTestEdgeZero2,
	opTestEdgeZero3,
	opTestEdgeZero4,
};
#endif

// Places where a pair of segments cross are recorded as intersections.
// Pairs of intersections, along with segments' ends, extremas, and inflections,
// are used to create edges. Edges may then be subdivided so that each edge has
// a unique winding contribution, but it is not necessary to record those 
// additional subdivisions as intersections. For that matter, it isn't necessary
// to record extremas or inflections in the intersection array; it is done as
// a matter of convenience when initially constructing the edge list.
struct OpIntersection {
	OpIntersection(const OpPtT& t, OpSegment* seg, SelfIntersect self_ , int cID
			OP_DEBUG_PARAMS(IntersectMaker maker))
		: segment(seg)
		, opp(nullptr)
		, ptT(t)
		, coincidenceID(cID)	// 0 if no coincidence; negative if coincident pairs are reversed
		, self(self_)
		, aliased(false)
		, unsortable(false) {
#if OP_DEBUG
		debugMaker = maker;
		debugCoincidenceID = 0;
		debugAliasID = 0;
		debugSetID();		// debug for now
		debugMatched = false;
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

	OpSegment* segment;	// at first: segment this intersects with; later: parent (once opp is set)
	OpIntersection* opp;
	OpPtT ptT;
	int coincidenceID;
	SelfIntersect self;	// inflection, or extrema -- doesn't reference another intersection
	bool aliased;     // true if point value was changed to match an intersection with the same t
	bool unsortable;  // !!! check the validity of whether this needs to be computed or not
#if OP_DEBUG
	IntersectMaker debugMaker;
	int id;
	int debugCoincidenceID;	// this one does not get erased
	int debugAliasID;
	OpPoint debugOriginal;	// point value prior to aliasing
    bool debugMatched;
#endif
};

#endif
