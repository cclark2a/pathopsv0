#ifndef OpEdgeIntersect_DEFINED
#define OpEdgeIntersect_DEFINED

#include "OpEdge.h"

enum class SectFound {
	no,
	fail,
	intersects,
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
		edgeParts.emplace_back(*edge);
		oppParts.emplace_back(*opp);
#if OP_DEBUG_DUMP
		debugActive = this;
#endif
	}

	SectFound addCoincidence(); // if curve is flat
	void addCurveCoin(OpEdge& edge, OpEdge& oppEdge);
	void addCurveCoincidence(); // if curve doesn't devolve into line segments
	SectFound addIntersection();
	bool atMaxSplits();
	static IntersectResult CurveCenter(const OpEdge& edge, OpEdge&);
	static SectFound CurvesIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts, std::vector<float>& oppTs);
	SectFound divideAndConquer();
	static void Split(std::vector<OpEdge>& parts, DoSplit );

#if OP_DEBUG_EDGE_INTERSECT
	#define OP_DEBUG_FIND_EDGE_CROSSINGS() debugFindEdgeCrossings()
	void debugFindEdgeCrossings();
#else
	#define OP_DEBUG_FIND_EDGE_CROSSINGS()
#endif
#if OP_DEBUG_DUMP
	static const OpEdgeIntersect* debugActive;

	~OpEdgeIntersect() {
		debugActive = nullptr;
	}
	void dump() const;
	void dumpDetail() const;
	DUMP_COMMON_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
	const OpEdge* originalEdge;
	const OpEdge* originalOpp;
	std::vector<OpEdge> edgeParts;
	std::vector<float> edgeTs;
	std::vector<OpEdge> oppParts;
	std::vector<float> oppTs;
};

#endif
