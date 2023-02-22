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

	void addCoincidence(); // if curve is flat
	void addCurveCoin(OpEdge& edge, OpEdge& oppEdge);
	void addCurveCoincidence(); // if curve doesn't devolve into line segments
	bool atMaxSplits();
	static IntersectResult CurveCenter(const OpEdge& edge, OpEdge&);
	static SectFound CurvesIntersect(std::vector<OpEdge>& edgeParts,
		std::vector<OpEdge>& oppParts, std::vector<float>& oppTs);
	SectFound divideAndConquer();
	static void Split(std::vector<OpEdge>& parts, DoSplit );

#if OP_DEBUG_DUMP
	static const OpEdgeIntersect* debugActive;
	OpEdgeIntersect() {
		debugActive = this;
	}

	~OpEdgeIntersect() {
		debugActive = nullptr;
	}
	DUMP_COMMON_DECLARATIONS();
#endif
#if OP_DEBUG_IMAGE
	void draw() const;
#endif
	std::vector<OpEdge> edgeParts;
	std::vector<float> edgeTs;
	std::vector<OpEdge> oppParts;
	std::vector<float> oppTs;
};

#endif
