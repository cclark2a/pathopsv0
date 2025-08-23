// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef OpWinding_DEFINED
#define OpWinding_DEFINED

#include "OpDebug.h"

struct OpContour;
struct OpContext;
struct OpEdge;

enum class WindingEdge {
	dummy
};

enum class WindingUninitialized {
	dummy
};

enum class WindingSum {
	dummy
};

enum class WindingTemp {	// used to accumulate winding sum before it is applied
	dummy
};

#define WindingType_Enums \
	OP_ENUM_MEMBER(uninitialized), \
	OP_ENUM_MEMBER(caller), \
	OP_ENUM_MEMBER(copy)

enum class WindingType  {
    WindingType_Enums
};

namespace PathOpsV0Lib {
	struct Winding;
}

#if OP_DEBUG

#define DebugWindingType_Enums \
	OP_ENUM_MEMBER(uninitialized), \
	OP_ENUM_MEMBER(temp), \
	OP_ENUM_MEMBER(winding), \
	OP_ENUM_MEMBER(sum) \

// !!! this has mostly fallen into disrepair; either fix it or delete it
enum class DebugWindingType {
    DebugWindingType_Enums
};
#endif

struct OpWinding {
	OpWinding(WindingUninitialized );
	OpWinding(OpEdge* edge, WindingSum );
	OpWinding(OpContext* c, PathOpsV0Lib::Winding );
	OpWinding(OpContext* context, const OpWinding& );
	void add(OpContext* , const PathOpsV0Lib::Winding& );
	void add(OpContext* , const OpWinding& );
	PathOpsV0Lib::Winding copyData(OpContext* ) const;
	void copyOnDemand(OpContext* );
	bool equal(const PathOpsV0Lib::Winding ) const;
	bool isSet() const { return WindingType::uninitialized != type; }
	void subtract(OpContext* , const PathOpsV0Lib::Winding& );
	void subtract(OpContext* , const OpWinding& );
	void move(OpContext* , const OpWinding& opp, bool backwards);
	void setWind(const OpWinding& fromSegment);
	int sum() const;
	bool visible(OpContext* ) const;
	void zero(OpContext* );
	void zeroUninitialized(OpContext* , const PathOpsV0Lib::Winding& );
	void zeroUninitialized(OpContext* , const OpWinding& );
#if OP_DEBUG_DUMP
	DUMP_DECLARATIONS
#endif

	PathOpsV0Lib::Winding w;
	WindingType type;
#if OP_DEBUG
	DebugWindingType debugType = DebugWindingType::uninitialized;
#endif
};

// An edge that can contribute to the answer has a zero winding on one side
// For a pair of edges to connect, they have to have zero windings on the same side
// If they have zero windings that do not match, there should be a third (and fourth)
// edge at the same point that is a better match.
// A pair of edges that are nearly coincident may be mis-sorted so that the zero
// winding is wrong.
// The normal zero winding is computed before the edge orientation (e.g., whichEnd) 
// is known, so it may be reversed if the edge is to be connected backwards.
#define WindZero_Enums \
	OP_ENUM_MEMBER(unset), \
	OP_ENUM_MEMBER(zero), \
	OP_ENUM_MEMBER(nonZero),

enum class WindZero : int8_t {
	WindZero_Enums
};

inline void OpDebugCheckSingleZero(WindZero left, WindZero right) {
	OP_ASSERT(WindZero::unset != left || WindZero::unset != right);
	OP_ASSERT(left == right);	// not normal and opp at same time
}

inline WindZero operator!(const WindZero& a) {
	switch (a) {
		case WindZero::unset: return WindZero::unset;
		case WindZero::zero: return WindZero::nonZero;
		case WindZero::nonZero: return WindZero::zero;
	}
	OP_ASSERT(0);
	return WindZero::unset;
}

#endif
