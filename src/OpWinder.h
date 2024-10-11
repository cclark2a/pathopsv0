// (c) 2023, Cary Clark cclark2@gmail.com
#ifndef OpWinder_DEFINED
#define OpWinder_DEFINED

#include "OpEdge.h"

enum class MatchEnds;  // for coin intersections
struct OpContours;
struct OpEdge;

enum class ChainFail {
	none,
	betweenUnsectables,
	failIntercept,
	noNormal,
	normalizeOverflow,
	normalizeUnderflow
};

enum class EdgesToSort {
	byBox,		// when walking to intersect, use box only
	byCenter 	// when walking to determine winding, use box and center ray
};

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

#if OP_NEW_COINCIDENCE
struct CoinEnd {
	void addSect(int coinID, OpSegment* baseSeg, MatchReverse m  OP_LINE_FILE_DEF());
	void aliasPtT();
	bool onBothEnds() const;
    DUMP_DECLARATIONS

	OpSegment* seg;
	OpSegment* opp;
	OpPtT ptT;
	float oppT = OpNaN;
};
#endif

struct OpWinder {
	OpWinder(OpContours& contours, EdgesToSort edgesToSort);
	OpWinder(OpEdge* sEdge, OpEdge* oEdge);
	void addEdge(OpEdge* , EdgesToSort );
	static IntersectResult AddLineCurveIntersection(OpEdge& opp, OpEdge& edge, 
			bool secondAttempt = false);
#if !OP_NEW_COINCIDENCE
	static void AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped, OpPtT cPtT, OpPtT dPtT,
			OpSegment* segment, OpSegment* oppSegment, int coinID, MatchEnds );
	static IntersectResult AddPair(XyChoice offset, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
			bool flipped, OpSegment* segment, OpSegment* oppSegment);
	static IntersectResult CoincidentCheck(OpPtT ptTa, OpPtT ptTb, OpPtT ptTc, OpPtT ptTd,
			OpSegment* segment, OpSegment* oppSegment);
#else
	static IntersectResult CoincidentCheck(OpSegment* seg, OpSegment* opp);
	static IntersectResult CoincidentCheck(std::array<CoinEnd, 4>& ends, bool* oppReversed);
#endif
	static IntersectResult CoincidentCheck(const OpEdge& edge, const OpEdge& opp);
	FoundIntercept findRayIntercept(size_t inIndex, OpVector tangent, float normal, float homeCept);
	void markUnsortable(Unsortable );
	size_t setInIndex(size_t homeIndex, float homeCept, std::vector<OpEdge*>& inArray);
	ChainFail setSumChain(size_t inIndex);
	ResolveWinding setWindingByDistance(OpContours* );
	FoundWindings setWindings(OpContours* );
	void sort(EdgesToSort);

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

	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> inY;
	OpEdge* home;
	Axis workingAxis;
};

#endif
