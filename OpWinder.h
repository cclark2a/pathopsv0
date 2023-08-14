#ifndef OpWinder_DEFINED
#define OpWinder_DEFINED

#include "OpMath.h"

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

enum class DistMult {
	none,
	first,
	mid,
	last,
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

// !!! is it worth wrapping a vector of these in a structure so methods can be associated?
struct EdgeDistance {
	EdgeDistance(OpEdge* e, float c, float tIn)
		: edge(e)
		, cept(c)
		, t(tIn)
		, multiple(DistMult::none)
		, edgeMultiple(false) {
	}

#if OP_DEBUG_DUMP
	void dump() const;
	void dumpDetail() const;
#endif

	OpEdge* edge;
	float cept;		// where normal intersects edge (for home edge, equals center)
	float t;
	DistMult multiple;
	bool edgeMultiple;
};

struct OpWinder {
	OpWinder(OpContours& contours, EdgesToSort edgesToSort);
	OpWinder(OpEdge* sEdge, OpEdge* oEdge);
	void addEdge(OpEdge* , EdgesToSort );
	static void AddLineCurveIntersection(OpEdge& opp, const OpEdge& edge);
	static void AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped, OpPtT cPtT, OpPtT dPtT,
			OpSegment* segment, OpSegment* oppSegment, int coinID);
	static IntersectResult AddPair(XyChoice offset, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
			bool flipped, OpSegment* segment, OpSegment* oppSegment);
	bool betweenUnsectables();
	static IntersectResult CoincidentCheck(OpPtT ptTa, OpPtT ptTb, OpPtT ptTc, OpPtT ptTd,
			OpSegment* segment, OpSegment* oppSegment);
	static IntersectResult CoincidentCheck(const OpEdge& edge, const OpEdge& opp);
	FoundIntercept findRayIntercept(size_t inIndex);
	void markPairUnsectable(EdgeDistance& dist1, EdgeDistance& dist2);
	void markUnsectableGroups();
	void markUnsortable();
	void setEdgeMultiple(EdgeDistance* edgeDist);
	ChainFail setSumChain(size_t inIndex);
	ResolveWinding setWindingByDistance();
	FoundWindings setWindings(OpContours* );
	void sort(EdgesToSort);

#if OP_DEBUG
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	void dumpAxis(Axis ) const;
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void debugDraw() const;
#endif

	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> inY;
	std::vector<EdgeDistance> distances;
	OpEdge* home;
	Axis axis;
	float normal;  // ray used to find windings on home edge
	float homeCept;  // intersection of normal on home edge
};

#endif
