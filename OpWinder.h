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
	EdgeDistance(OpEdge* e, float d, float n, float tIn)
		: edge(e)
		, distance(d)
		, normal(n)
		, t(tIn)
		, multiple(DistMult::none)
		, edgeMultiple(false) {
	}

#if OP_DEBUG_DUMP
	void dump() const;
	void dumpDetail() const;
#endif

	OpEdge* edge;
	float distance;
	float normal;
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
	static bool BetweenUnsectables(OpEdge* , Axis , std::vector<EdgeDistance>& );
	static IntersectResult CoincidentCheck(OpPtT ptTa, OpPtT ptTb, OpPtT ptTc, OpPtT ptTd,
			OpSegment* segment, OpSegment* oppSegment);
	static IntersectResult CoincidentCheck(const OpEdge& edge, const OpEdge& opp);
	FoundIntercept findRayIntercept(size_t inIndex, Axis , OpEdge* edge, float center, 
			float normal, float edgeCenterT, std::vector<EdgeDistance>* );
	static void MarkPairUnsectable(Axis axis, EdgeDistance& dist1, EdgeDistance& dist2);
	static void MarkUnsectableGroups(Axis axis, std::vector<EdgeDistance>& distance);
	void markUnsortable();
	void markUnsortable(OpEdge* edge, Axis );
	static void SetEdgeMultiple(Axis axis, EdgeDistance* edgeDist  
			OP_DEBUG_PARAMS(std::vector<EdgeDistance>& distance));
	ChainFail setSumChain(size_t inIndex, Axis );
	ResolveWinding setWindingByDistance(OpEdge* edge, Axis , std::vector<EdgeDistance>& );
	FoundWindings setWindings(OpContours* );
	void sort(EdgesToSort);

#if OP_DEBUG
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	void dumpAxis(Axis axis) const;
#include "OpDebugDeclarations.h"
#endif
#if OP_DEBUG_IMAGE
	void debugDraw() const;
#endif

	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> inY;
};

#endif
