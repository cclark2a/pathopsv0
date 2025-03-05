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
	normalizeUnderflow
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
struct OpTarget {
	OpContour* contour;
	OpPointBounds bounds;
};

struct OpWinder {
	OpWinder(OpContext& contours);
	void addEdge(OpEdge* );
	void buildTargets();
	static IntersectResult CoincidentCheck(OpSegment* seg, OpSegment* opp);
	static IntersectResult CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversed,
			XyChoice* );
	FoundIntercept findRayIntercept(OpVector tangent, float normal, float homeCept);
	void markUnsortable(Unsortable );
	OpEdge* nextTarget(float homeCept);
#if 0 // WINDER_CONTOUR_EXPERIMENT
	// too specialized; needs to consider all edges in ray, not just coin pals pointed to by edge
	OpEdge* partiallyCoincident(OpEdge *);  // returns partially coincident prior edge on ray 
#endif
	void resetTarget();
	ChainFail setSumChain();
	void setTarget();
	ResolveWinding setWindingByDistance(OpContext* );
	FoundWindings setWindings(OpContext* );
	void sort();

#if OP_DEBUG_DUMP
#include "OpDebugDeclarations.h"
#endif

#if WINDER_CONTOUR_EXPERIMENT  // instead, pass edge list to functions that need it
	// std::vector<OpEdge*>* inXPtr;
	// std::vector<OpEdge*>* inYPtr;
#else
	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> inY;
#endif
	std::vector<OpTarget> targets;
	std::vector<OpEdge*>* targetEdges;
	OpRect chainBounds;
	OpRect* targetBounds;
	size_t targetIndex;
	size_t targetEdge;
	OpEdge* home;
	Axis workingAxis;
	float interceptLimit;
};

#endif
