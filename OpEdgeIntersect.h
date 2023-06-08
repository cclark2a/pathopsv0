#ifndef OpEdgeIntersect_DEFINED
#define OpEdgeIntersect_DEFINED

#include "OpEdge.h"
#include "OpIntersection.h"

enum class SectFound {
	no,
	fail,
	intersects,
	overflow,
	split
};

enum class DoSplit {
	marked,
	all
};

struct OpEdgeIntersect {
	static constexpr int maxSplits = 8;   // !!! no idea what this should be 
	static constexpr int maxDepth = 24;  // !!! no idea what this should be

	OpEdgeIntersect(const OpEdge* edge, const OpEdge* opp)
		: originalEdge(edge)
		, originalOpp(opp) {
		edgeCurves.emplace_back(*edge);
		oppCurves.emplace_back(*opp);
#if OP_DEBUG_DUMP
		debugActive = this;
#endif
	}

	SectFound addCoincidence(); // if curve is flat
	void addCurveCoin(OpEdge& edge, OpEdge& oppEdge);
	SectFound addCurveCoincidence(); // if curve doesn't devolve into line segments
	SectFound addIntersections(OP_DEBUG_CODE(SectReason ));	// adds zero, one, or more
	static void CullCurves(std::vector<OpEdge>& edgeParts);
	static IntersectResult CurveCenter(const OpEdge& edge, OpEdge&);
	static SectFound CurvesIntersect(std::vector<OpEdge>& edgeParts,
			std::vector<OpEdge>& oppParts, std::vector<float>& oppTs);
	SectFound divideAndConquer();
	static void LinearIntersect(std::vector<OpEdge>& edgeParts,
			std::vector<OpEdge>& oppParts);
	static bool Split(std::vector<OpEdge>& curves, std::vector<OpEdge>& lines, DoSplit );

#if OP_DEBUG_DUMP
	static const OpEdgeIntersect* debugActive;

	~OpEdgeIntersect() {
		debugActive = nullptr;
	}
	void dump(bool detail) const;
	void dump() const { dump(false); }
	void dumpDetail() const { dump(true); }
	DUMP_COMMON_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
	const OpEdge* originalEdge;
	const OpEdge* originalOpp;
	std::vector<OpEdge> edgeCurves;
	std::vector<OpEdge> edgeLines;
	std::vector<float> edgeTs;
	std::vector<OpEdge> oppCurves;
	std::vector<OpEdge> oppLines;
	std::vector<float> oppTs;
	std::vector<OpEdge> edgeResults;
	std::vector<OpEdge> oppResults;
};

#endif
