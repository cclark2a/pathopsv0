#ifndef OpEdges_DEFINED
#define OpEdges_DEFINED

#include "OpMath.h"

struct OpContours;
struct OpEdge;

enum class EdgesToSort {
	byBox,		// when walking to intersect, use box only
	byCenter	// when walking to determing winding, use box and center ray
};

enum class FoundIntersections {
	fail,
	yes
};

enum class FoundIntercept {
	fail,
	set,
	yes
};

enum class FoundWindings {
	fail,
	yes
};

struct EdgeDistance {
	EdgeDistance(OpEdge* e, float d, float _t)
		: edge(e)
		, distance(d)
		, t(_t) {
	}

	OpEdge* edge;
	float distance;
	float t;
};

struct OpEdges {
	OpEdges(OpContours& contours, EdgesToSort);
	OpEdges(OpEdge* sEdge, OpEdge* oEdge);
	void addEdge(OpEdge* , EdgesToSort );
	static void AddIntersection(OpEdge& opp, const OpEdge& edge);
	static void AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped, OpPtT cPtT, OpPtT dPtT,
			OpSegment* segment, OpSegment* oppSegment, int coinID);
	static IntersectResult AddPair(XyChoice offset, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
			bool flipped, OpSegment* segment, OpSegment* oppSegment);
	static IntersectResult CoincidentCheck(OpPtT ptTa, OpPtT ptTb, OpPtT ptTc, OpPtT ptTd,
			OpSegment* segment, OpSegment* oppSegment);
	static IntersectResult CoincidentCheck(const OpEdge& edge, const OpEdge& opp);
	FoundIntersections findIntersections();
	FoundIntercept findRayIntercept(size_t inIndex, Axis , OpEdge* edge, float center, 
			float normal, float edgeCenterT, std::vector<EdgeDistance>* );
	void markUnsortable(OpEdge* edge, Axis , ZeroReason);
	void setSumChain(size_t inIndex, Axis );
	FoundWindings setWindings(OpContours* );
	void sort(EdgesToSort);

#if OP_DEBUG
	void debugValidate() const;
#endif
#if OP_DEBUG_DUMP
	void dump() const;
	void dumpAxis(Axis ) const;
	DUMP_COMMON_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif

	// assist to find axis-aligned ray intersection from edge center to next curve
	std::vector<OpEdge*> inX;
	std::vector<OpEdge*> inY;
};

#endif
