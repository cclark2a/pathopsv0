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

struct OpWinder {
	OpWinder(OpContext& contours);
	void addEdge(OpEdge* );
	static IntersectResult CoincidentCheck(OpSegment* seg, OpSegment* opp);
	static IntersectResult CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversed,
			XyChoice* );
	FoundIntercept findRayIntercept(size_t inIndex, OpVector tangent, float normal, float homeCept);
	void markUnsortable(Unsortable );
	size_t setInIndex(size_t homeIndex, float homeCept, std::vector<OpEdge*>& inArray);
	ChainFail setSumChain(size_t inIndex);
	ResolveWinding setWindingByDistance(OpContext* );
	FoundWindings setWindings(OpContext* );
	void sort();

#if OP_DEBUG_VALIDATE
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	std::string debugDumpAxis(Axis , DebugLevel , DebugBase ) const;
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void debugDraw();
#endif

#if WINDER_CONTOUR_EXPERIMENT
	std::vector<OpEdge*>* inXPtr;
	std::vector<OpEdge*>* inYPtr;
#else
	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> inY;
#endif
	OpEdge* home;
	Axis workingAxis;
	float interceptLimit;
};

#endif
