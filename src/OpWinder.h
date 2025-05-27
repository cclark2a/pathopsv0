// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpWinder_DEFINED
#define OpWinder_DEFINED

#include "OpEdge.h"

enum class MatchEnds;  // for coin intersections
struct OpContext;
struct OpEdge;

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
	OpWinder(OpContext& );
	ChainFail addContainers(OpEdge* top, OpEdge* child, std::vector<OpEdge*>& );
	void addEdge(OpEdge* );
	static IntersectResult CoincidentCheck(OpSegment* seg, OpSegment* opp);
	static IntersectResult CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversed,
			XyChoice* );
	FoundIntercept findCept(OpEdge* );
	FoundIntercept findRayIntercept(OpVector tangent, float normal, float homeCept);
//	void markUnsortable(OpEdge* , Unsortable );
	ChainFail setCept(OpEdge* );
	FoundWindings setPriors(OpEdge*   OP_DEBUG_PARAMS(std::vector<OpEdge*>& debugVisited));
//	ChainFail setSumChain();
	ResolveWinding setWindingByDistance(OpEdge* );
	FoundWindings setWindings(OpContext& );
	void sort();

#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
#endif
//		start here;
		// findRayIntercept may need to add to targets any contour that
		// can be used by an earlier edge in the ray distances, if the contour bounds is to the
		// right of the earlier edge bounds
		// requires recursive ray finding so that earlier edge's distances are known
//	RayTargets targets;
//	std::vector<OpEdge*> recurse;
//	OpEdge* home;
//	Axis workingAxis;
//	int byDistanceDepth;  // !!! replace this with debug flag to detect infinite recursion
};

#endif
