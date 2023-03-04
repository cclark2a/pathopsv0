#ifndef OpIntersection_DEFINED
#define OpIntersection_DEFINED

#include "OpMath.h"

struct OpSegment;

enum class SelfIntersect {
	self
};

#if OP_DEBUG
enum class IntersectMaker {
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
	opCubicErrorTest1,
	opCubicErrorTest2,
	opTestEdgeZero1,
	opTestEdgeZero2,
	opTestEdgeZero3,
	opTestEdgeZero4,
};
#endif

struct OpIntersection {
	OpIntersection(const OpPtT& t, OpSegment* seg, SelfIntersect  
			OP_DEBUG_PARAMS(IntersectMaker maker))
		: segment(seg)
		, opp(nullptr)
		, ptT(t)
		, coincidenceID(0)	// no coincidence
		, self(true)
		, unsortable(false) {
#if OP_DEBUG
		debugMaker = maker;
		debugComplete();		// debug for now
#endif
	}

	OpIntersection(const OpPtT& t, OpSegment* seg, int cID    
			OP_DEBUG_PARAMS(IntersectMaker maker))
		: segment(seg)
		, opp(nullptr)
		, ptT(t)
		, coincidenceID(cID)	// 0 if no coincidence; negative if coincident pairs are reversed
		, self(false)
		, unsortable(false) {
#if OP_DEBUG
		debugMaker = maker;
		debugComplete();		// debug for now
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
	void debugComplete();
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	OpIntersection(std::string);
	void debugCompare(std::string) const;
	std::string debugDumpBrief() const;
	void dumpPt() const;
	DEBUG_COMMON_DECLARATIONS();
	DUMP_COMMON_DECLARATIONS();
#endif

	OpSegment* segment;	// at first: segment this intersects with; later: parent (once opp is set)
	OpIntersection* opp;
	OpPtT ptT;
	int coincidenceID;
	bool self;	// inflection, or extrema -- doesn't reference another intersection
	bool unsortable;  // 
#if OP_DEBUG
	IntersectMaker debugMaker;
	int id;
	int debugCoincidenceID;	// this one does not get erased
#endif
};

#endif
