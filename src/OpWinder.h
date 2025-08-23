// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpWinder_DEFINED
#define OpWinder_DEFINED

#include "OpEdge.h"

enum class ChainFail {
	none,
	betweenUnsectables,
	containerAdded,
	failIntercept,
	noNormal,
	normalizeOverflow,
	normalizeUnderflow,
	recurse
};

#if 0
enum class EdgesToSort {
	byBox,		// when walking to intersect, use box only
	byCenter 	// when walking to determine winding, use box and center ray
};
#endif

enum class FoundIntersections {
	fail,
	yes
};

enum class FoundIntercept {
	fail,
	overflow,
	recurse,  // recursion happens if simple sort isn't sufficient to compute rays in order
	set,
	yes
};

enum class FoundWindings {
	fail,
	yes
};

struct CoinEnd {
//	void addSect(int coinID, OpSegment* baseSeg, MatchReverse m, XyChoice  OP_LINE_FILE_ARGS());
//	void aliasPtT(XyChoice );
	bool onBothEnds(XyChoice ) const;
//	bool ptsAreClose(CoinEnd& , XyChoice );
	DUMP_DECLARATIONS

	OpSegment* seg;
	OpSegment* opp;
	OpPtT ptT;
	OpVector oppT;
};

struct OpWinder {
	static ChainFail AddContainers(OpEdge* top, OpEdge* child, std::vector<OpEdge*>& );
	static IntersectResult CoincidentCheck(OpSegment* seg, OpSegment* opp);
	static IntersectResult CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversed,
			XyChoice* );
	static FoundIntercept FindACept(OpEdge* );
	static ChainFail SetCept(OpEdge* );
	static FoundWindings SetPriors(OpEdge*   OP_DEBUG_PARAMS(std::vector<OpEdge*>& debugVisited));
	static ResolveWinding SetWindingByDistance(OpEdge* );
	static FoundWindings SetWindings(OpContext& );
	static void Sort();
};

#endif
