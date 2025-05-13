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
	failIntercept,
	noNormal,
	normalizeOverflow,
	normalizeUnderflow,
	// recurse  // some member of the chain needs to be evaluated earlier !!! delay to sum
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
//	recurse,  // recursion happens when sums are computed instead of when rays intersect
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

// target bounds is intersection of contour bounds, parent bounds, and home/ray bounds
struct RayTarget {
	OpContour* contour;
	OpPointBounds bounds;
};

struct RayTargets {
	void addContainer(OpContour* container, OpRect& bounds);
	void build(OpWinder* );
	bool match(OpContour* ) const;
	OpEdge* next(float homeCept);
	void reset();
	void set();

	std::vector<RayTarget> t;
	std::vector<OpEdge*>* edges;
	OpRect chainBounds;
	size_t edgeIndex;
	size_t index;
	Axis axis;
};

struct OpWinder {
	OpWinder(OpContext& );
	void addEdge(OpEdge* );
	static IntersectResult CoincidentCheck(OpSegment* seg, OpSegment* opp);
	static IntersectResult CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversed,
			XyChoice* );
	FoundIntercept findRayIntercept(OpVector tangent, float normal, float homeCept);
	void markUnsortable(Unsortable );
	FoundWindings setPriors(OpContext& , OpEdge* );
	ChainFail setSumChain();
	ResolveWinding setWindingByDistance(OpContext& );
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
	RayTargets targets;
	OpEdge* home;
	Axis workingAxis;
	float interceptLimit;
	float minCeptDiff;
//	int byDistanceDepth;  // !!! replace this with debug flag to detect infinite recursion
};

#endif
